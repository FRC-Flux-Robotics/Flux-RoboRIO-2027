package frc.lib.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.drive.Drive;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import org.littletonrobotics.junction.Logger;

/**
 * 17-Second "One-Button Pre-Flight" Pit System Check ({@code PitSystemCheckCmd}).
 *
 * <p>Steps all 4 swerve corners and every registered {@link PositionMechanism} and {@link
 * VelocityMechanism} sequentially with real settling windows, asserting connection health,
 * non-zero motion/velocity in the commanded safe direction, and peak stator current {@code <= 60 A}
 * on every checked actuator.
 */
public class PitSystemCheckCmd extends Command {
  public static final int DEFAULT_STEP_LOOPS = 60; // 1.20 s active window per actuator at 50 Hz
  public static final int DEFAULT_SETTLE_LOOPS = 25; // 0.50 s settle/return window per actuator
  public static final int TOTAL_LOOPS = 10 * (DEFAULT_STEP_LOOPS + DEFAULT_SETTLE_LOOPS);
  public static final double MAX_SAFE_STATOR_AMPS = 60.0;
  public static final double MIN_SWERVE_SPEED_MPS = 0.25;
  public static final double MIN_POSITION_DELTA_ROT = 0.15;
  public static final double MIN_POSITION_VELOCITY_RPS = 0.25;

  private static final String[] CORNER_NAMES = {"DriveFL", "DriveFR", "DriveBL", "DriveBR"};

  /** Safe position target and return setpoint for sequential pit testing. */
  public record PositionCheckTarget(
      PositionMechanism mechanism, double activePosRot, double returnPosRot) {}

  /** Safe velocity target (signed RPS) for sequential pit testing. */
  public record VelocityCheckTarget(VelocityMechanism mechanism, double targetRps) {}

  private final Drive drive;
  private final List<PositionCheckTarget> positionTargets;
  private final List<VelocityCheckTarget> velocityTargets;
  private final int activeStepLoops;
  private final int settleLoops;
  private final int loopsPerStep;
  private final int totalSteps;
  private final int totalLoops;
  private final Runnable preCheckHook;

  private final List<String> failedActuators = new ArrayList<>();
  private int loopCount = 0;
  private int completedSteps = 0;
  private boolean passed = false;
  private String summary = "NOT_RUN";

  private boolean stepConnected = false;
  private boolean stepEverDisconnected = false;
  private double stepPeakSpeed = 0.0;
  private double stepReverseSpeed = 0.0;
  private double stepPeakStatorAmps = 0.0;
  private double stepStartPosRot = 0.0;
  private double stepMaxDisplacementRot = 0.0;
  private double stepReverseDisplacementRot = 0.0;
  private double stepPeakVelRps = 0.0;
  private double stepReverseVelRps = 0.0;

  public PitSystemCheckCmd(
      Drive drive,
      List<PositionMechanism> positionMechanisms,
      List<VelocityMechanism> velocityMechanisms) {
    this(
        drive,
        buildDefaultPositionTargets(positionMechanisms),
        buildDefaultVelocityTargets(velocityMechanisms),
        DEFAULT_STEP_LOOPS,
        DEFAULT_SETTLE_LOOPS,
        () -> {});
  }

  public PitSystemCheckCmd(
      Drive drive,
      List<PositionCheckTarget> positionTargets,
      List<VelocityCheckTarget> velocityTargets,
      Runnable preCheckHook,
      Subsystem... extraRequirements) {
    this(
        drive,
        positionTargets,
        velocityTargets,
        DEFAULT_STEP_LOOPS,
        DEFAULT_SETTLE_LOOPS,
        preCheckHook,
        extraRequirements);
  }

  public PitSystemCheckCmd(
      Drive drive,
      List<PositionCheckTarget> positionTargets,
      List<VelocityCheckTarget> velocityTargets,
      int activeStepLoops,
      int settleLoops,
      Runnable preCheckHook,
      Subsystem... extraRequirements) {
    this.drive = drive;
    this.positionTargets = List.copyOf(positionTargets);
    this.velocityTargets = List.copyOf(velocityTargets);
    this.activeStepLoops = Math.max(4, activeStepLoops);
    this.settleLoops = Math.max(2, settleLoops);
    this.loopsPerStep = this.activeStepLoops + this.settleLoops;
    this.totalSteps = drive.getModuleCount() + this.positionTargets.size() + this.velocityTargets.size();
    this.totalLoops = this.totalSteps * this.loopsPerStep;
    this.preCheckHook = (preCheckHook != null) ? preCheckHook : () -> {};

    addRequirements(drive);
    for (PositionCheckTarget pt : this.positionTargets) {
      addRequirements(pt.mechanism());
    }
    for (VelocityCheckTarget vt : this.velocityTargets) {
      addRequirements(vt.mechanism());
    }
    if (extraRequirements != null) {
      for (Subsystem sub : extraRequirements) {
        if (sub != null) {
          addRequirements(sub);
        }
      }
    }
  }

  private static List<PositionCheckTarget> buildDefaultPositionTargets(
      List<PositionMechanism> mechanisms) {
    List<PositionCheckTarget> out = new ArrayList<>(mechanisms.size());
    for (PositionMechanism pm : mechanisms) {
      MechanismSpec spec = pm.getSpec();
      double activeTarget;
      double returnTarget = 0.0;
      if (spec.maxPosRot() <= 0.0 && spec.minPosRot() < 0.0) {
        activeTarget = Math.max(spec.minPosRot(), -2.0);
        returnTarget = Math.min(spec.maxPosRot(), 0.0);
      } else {
        activeTarget = Math.min(spec.maxPosRot(), Math.max(spec.minPosRot(), 2.0));
        returnTarget = Math.max(spec.minPosRot(), Math.min(spec.maxPosRot(), 0.0));
      }
      out.add(new PositionCheckTarget(pm, activeTarget, returnTarget));
    }
    return out;
  }

  private static List<VelocityCheckTarget> buildDefaultVelocityTargets(
      List<VelocityMechanism> mechanisms) {
    List<VelocityCheckTarget> out = new ArrayList<>(mechanisms.size());
    for (VelocityMechanism vm : mechanisms) {
      String lower = vm.getSpec().mechanism().toLowerCase(Locale.ROOT);
      double targetRps = 20.0;
      if (lower.contains("feeder")) {
        targetRps = -20.0;
      } else if (lower.contains("shooter") || lower.contains("flywheel")) {
        targetRps = 25.0;
      }
      out.add(new VelocityCheckTarget(vm, targetRps));
    }
    return out;
  }

  @Override
  public void initialize() {
    loopCount = 0;
    completedSteps = 0;
    passed = false;
    failedActuators.clear();
    summary = "RUNNING";
    preCheckHook.run();
    stopAllActuators();
    resetStepAccumulators();
    Logger.recordOutput("PitCheck/Passed", false);
    Logger.recordOutput("PitCheck/Summary", summary);
    Logger.recordOutput("PitCheck/ActiveStep", "STARTING");
    Logger.recordOutput("PitCheck/FailedActuators", "");
  }

  @Override
  public void execute() {
    if (loopCount >= totalLoops) {
      return;
    }
    int stepIdx = loopCount / loopsPerStep;
    int localLoop = (loopCount % loopsPerStep) + 1;
    loopCount++;

    if (localLoop == 1) {
      stopAllActuators();
      resetStepAccumulators();
    }

    int cornerCount = drive.getModuleCount();
    int posCount = positionTargets.size();
    boolean activePhase = localLoop <= activeStepLoops;
    boolean sampleConnection = localLoop >= 2;

    if (stepIdx < cornerCount) {
      String label = (stepIdx < CORNER_NAMES.length) ? CORNER_NAMES[stepIdx] : "Drive" + stepIdx;
      Logger.recordOutput("PitCheck/ActiveStep", label);
      if (activePhase) {
        SwerveModuleState cmd = new SwerveModuleState(1.2, Rotation2d.fromDegrees(45.0));
        drive.runModuleTest(stepIdx, cmd);
        if (Math.abs(cmd.speedMetersPerSecond) >= 0.05) {
          double signedSpeed =
              drive.getModuleVelocityMps(stepIdx) * Math.signum(cmd.speedMetersPerSecond);
          stepPeakSpeed = Math.max(stepPeakSpeed, signedSpeed);
          stepReverseSpeed = Math.max(stepReverseSpeed, -signedSpeed);
        }
      } else {
        drive.stop();
      }
      if (sampleConnection) {
        boolean connected = drive.isModuleConnected(stepIdx);
        stepConnected |= connected;
        if (!connected) {
          stepEverDisconnected = true;
        }
      }
      double cornerStator =
          Math.max(
              Math.abs(drive.getModuleDriveStatorCurrentAmps(stepIdx)),
              Math.abs(drive.getModuleTurnStatorCurrentAmps(stepIdx)));
      stepPeakStatorAmps = Math.max(stepPeakStatorAmps, cornerStator);

      if (localLoop == loopsPerStep) {
        drive.stop();
        evaluateStep(
            label,
            stepConnected && !stepEverDisconnected,
            stepPeakSpeed >= MIN_SWERVE_SPEED_MPS,
            stepReverseSpeed >= MIN_SWERVE_SPEED_MPS,
            stepPeakStatorAmps);
      }
    } else if (stepIdx < cornerCount + posCount) {
      PositionCheckTarget target = positionTargets.get(stepIdx - cornerCount);
      PositionMechanism pm = target.mechanism();
      String label = pm.getSpec().mechanism();
      Logger.recordOutput("PitCheck/ActiveStep", label);
      if (localLoop == 1) {
        stepStartPosRot = pm.getPosition();
      }
      double expectedDelta = target.activePosRot() - stepStartPosRot;
      double dirSign =
          (Math.abs(expectedDelta) > 1e-4)
              ? Math.signum(expectedDelta)
              : (Math.abs(target.activePosRot()) > 1e-4 ? Math.signum(target.activePosRot()) : 1.0);
      if (activePhase) {
        pm.setPosition(target.activePosRot());
        double signedDisp = (pm.getPosition() - stepStartPosRot) * dirSign;
        double signedVel = pm.getVelocity() * dirSign;
        stepMaxDisplacementRot = Math.max(stepMaxDisplacementRot, signedDisp);
        stepReverseDisplacementRot = Math.max(stepReverseDisplacementRot, -signedDisp);
        stepPeakVelRps = Math.max(stepPeakVelRps, signedVel);
        stepReverseVelRps = Math.max(stepReverseVelRps, -signedVel);
      } else {
        pm.setPosition(target.returnPosRot());
      }
      if (sampleConnection) {
        boolean connected = pm.isConnected();
        stepConnected |= connected;
        if (!connected) {
          stepEverDisconnected = true;
        }
      }
      stepPeakStatorAmps = Math.max(stepPeakStatorAmps, pm.getMaxMotorStatorCurrentAmps());

      if (localLoop == loopsPerStep) {
        pm.stop();
        boolean moved =
            stepMaxDisplacementRot >= MIN_POSITION_DELTA_ROT
                || stepPeakVelRps >= MIN_POSITION_VELOCITY_RPS;
        // Require opposite-direction displacement past start so healthy closed-loop overshoot
        // damping back toward setpoint never falsely triggers inverted-direction.
        boolean inverted = stepReverseDisplacementRot >= MIN_POSITION_DELTA_ROT;
        evaluateStep(
            label, stepConnected && !stepEverDisconnected, moved, inverted, stepPeakStatorAmps);
      }
    } else {
      VelocityCheckTarget target = velocityTargets.get(stepIdx - cornerCount - posCount);
      VelocityMechanism vm = target.mechanism();
      String label = vm.getSpec().mechanism();
      Logger.recordOutput("PitCheck/ActiveStep", label);
      if (activePhase) {
        vm.setVelocity(target.targetRps());
        double signedVel = vm.getVelocity() * Math.signum(target.targetRps());
        stepPeakSpeed = Math.max(stepPeakSpeed, signedVel);
        stepReverseSpeed = Math.max(stepReverseSpeed, -signedVel);
      } else {
        vm.stop();
      }
      if (sampleConnection) {
        boolean connected = vm.isConnected();
        stepConnected |= connected;
        if (!connected) {
          stepEverDisconnected = true;
        }
      }
      stepPeakStatorAmps = Math.max(stepPeakStatorAmps, vm.getMaxMotorStatorCurrentAmps());

      if (localLoop == loopsPerStep) {
        vm.stop();
        double minExpectedRps = Math.min(3.0, Math.abs(target.targetRps()) * 0.20);
        evaluateStep(
            label,
            stepConnected && !stepEverDisconnected,
            stepPeakSpeed >= minExpectedRps,
            stepReverseSpeed >= minExpectedRps,
            stepPeakStatorAmps);
      }
    }
  }

  private void evaluateStep(
      String label,
      boolean connectedOk,
      boolean motionOk,
      boolean invertedMotion,
      double peakStatorAmps) {
    completedSteps++;
    boolean currentOk = peakStatorAmps <= MAX_SAFE_STATOR_AMPS;
    boolean stepPassed = connectedOk && motionOk && currentOk && !invertedMotion;

    Logger.recordOutput("PitCheck/Actuator/" + label + "/Passed", stepPassed);
    Logger.recordOutput("PitCheck/Actuator/" + label + "/Connected", connectedOk);
    Logger.recordOutput("PitCheck/Actuator/" + label + "/PeakStatorAmps", peakStatorAmps);

    if (!stepPassed) {
      List<String> reasons = new ArrayList<>();
      if (!connectedOk) {
        reasons.add("disconnected");
      }
      if (invertedMotion) {
        reasons.add("inverted-direction");
      } else if (!motionOk) {
        reasons.add("no-motion");
      }
      if (!currentOk) {
        reasons.add(String.format(Locale.US, "overcurrent=%.1fA", peakStatorAmps));
      }
      failedActuators.add(label + "[" + String.join(",", reasons) + "]");
    }
  }

  private void resetStepAccumulators() {
    stepConnected = false;
    stepEverDisconnected = false;
    stepPeakSpeed = 0.0;
    stepReverseSpeed = 0.0;
    stepPeakStatorAmps = 0.0;
    stepStartPosRot = 0.0;
    stepMaxDisplacementRot = 0.0;
    stepReverseDisplacementRot = 0.0;
    stepPeakVelRps = 0.0;
    stepReverseVelRps = 0.0;
  }

  private void stopAllActuators() {
    drive.stop();
    for (PositionCheckTarget pt : positionTargets) {
      pt.mechanism().stop();
    }
    for (VelocityCheckTarget vt : velocityTargets) {
      vt.mechanism().stop();
    }
  }

  @Override
  public boolean isFinished() {
    return loopCount >= totalLoops;
  }

  @Override
  public void end(boolean interrupted) {
    stopAllActuators();
    passed = !interrupted && completedSteps == totalSteps && failedActuators.isEmpty();
    if (passed) {
      summary = "PASS (" + totalSteps + "/" + totalSteps + " actuators)";
    } else if (interrupted) {
      summary = "FAIL (Interrupted at step " + completedSteps + "/" + totalSteps + ")";
    } else {
      summary = "FAIL (" + String.join("; ", failedActuators) + ")";
    }
    Logger.recordOutput("PitCheck/Passed", passed);
    Logger.recordOutput("PitCheck/Summary", summary);
    Logger.recordOutput("PitCheck/ActiveStep", "DONE");
    Logger.recordOutput("PitCheck/FailedActuators", String.join("; ", failedActuators));
  }

  public boolean hasPassed() {
    return passed;
  }

  public String getSummary() {
    return summary;
  }

  public List<String> getFailedActuators() {
    return Collections.unmodifiableList(failedActuators);
  }

  public int getTotalLoops() {
    return totalLoops;
  }
}
