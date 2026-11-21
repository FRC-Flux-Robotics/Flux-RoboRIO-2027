package frc.lib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO.MechanismIOInputs;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Generic config-driven position mechanism subsystem backed by {@link MechanismIO} with {@link
 * LoggedTunableNumber} PID/gravity gains.
 */
public class PositionMechanism extends SubsystemBase {

  private final MechanismIO io;
  private final MechanismSpec spec;
  private final MechanismIOInputs inputs = new MechanismIOInputs();

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kG;

  public static final double MAX_MANUAL_JOG_VOLTS = 2.5;

  private double targetPosition = 0.0;
  private boolean zeroConfirmed = false;
  private final double toleranceRot;

  public PositionMechanism(MechanismIO io, MechanismSpec spec) {
    this(io, spec, 0.25);
  }

  public PositionMechanism(MechanismIO io, MechanismSpec spec, double toleranceRot) {
    this.io = io;
    this.spec = spec;
    this.toleranceRot = toleranceRot;

    String prefix = spec.mechanism();
    kP = new LoggedTunableNumber(prefix + "/kP", spec.kP());
    kI = new LoggedTunableNumber(prefix + "/kI", spec.kI());
    kD = new LoggedTunableNumber(prefix + "/kD", spec.kD());
    kS = new LoggedTunableNumber(prefix + "/kS", spec.kS());
    kV = new LoggedTunableNumber(prefix + "/kV", spec.kV());
    kG = new LoggedTunableNumber(prefix + "/kG", spec.kG());
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        vals -> io.setPID(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]),
        kP,
        kI,
        kD,
        kS,
        kV,
        kG);

    io.updateInputs(inputs);
    Logger.processInputs(spec.mechanism(), inputs);
    Logger.recordOutput(spec.mechanism() + "/TargetPosition", targetPosition);
    Logger.recordOutput(spec.mechanism() + "/AtTarget", atTarget());
    Logger.recordOutput(spec.mechanism() + "/ZeroConfirmed", zeroConfirmed);
  }

  /**
   * Commands the mechanism to a target position in rotations (clamped to CSV soft limits). Stays
   * neutral until {@link #resetPosition()} succeeds so un-homed relative encoders never slam into a
   * hard stop on enable.
   */
  public void setPosition(double positionRotations) {
    if (!zeroConfirmed) {
      io.stop();
      return;
    }
    targetPosition = MathUtil.clamp(positionRotations, spec.minPosRot(), spec.maxPosRot());
    io.setPosition(targetPosition);
  }

  /** Commands open-loop voltage directly (for characterization). */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Commands open-loop voltage while ignoring software limits (for hard-stop homing). */
  public void setHomingVoltage(double volts) {
    io.setHomingVoltage(volts);
  }

  /**
   * Commands safe low-voltage manual jog in Teleop: respects CSV/hardware soft limits once zeroed,
   * while still allowing operators to jog toward the physical hard stop prior to re-zeroing.
   */
  public void jogVoltage(double volts) {
    double clampedVolts = MathUtil.clamp(volts, -MAX_MANUAL_JOG_VOLTS, MAX_MANUAL_JOG_VOLTS);
    if (zeroConfirmed) {
      double pos = getPosition();
      if ((clampedVolts > 0.0 && pos >= spec.maxPosRot())
          || (clampedVolts < 0.0 && pos <= spec.minPosRot())) {
        io.stop();
        return;
      }
      io.setVoltage(clampedVolts);
    } else {
      io.setHomingVoltage(clampedVolts);
    }
  }

  public void stop() {
    io.stop();
  }

  public boolean atTarget() {
    return zeroConfirmed && Math.abs(getPosition() - targetPosition) <= toleranceRot;
  }

  public boolean isZeroConfirmed() {
    return zeroConfirmed;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  public double getVelocity() {
    return inputs.velocityRPS;
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentA;
  }

  /** Returns peak per-motor stator current so dual-motor mechanisms are checked per motor. */
  public double getMaxMotorStatorCurrentAmps() {
    double leader = Math.abs(inputs.leaderStatorCurrentA);
    double follower = Math.abs(inputs.followerStatorCurrentA);
    if (leader > 0.0 || follower > 0.0) {
      return Math.max(leader, follower);
    }
    int motorCount = (spec.followerMotorId() >= 0) ? 2 : 1;
    return Math.abs(inputs.statorCurrentA) / motorCount;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentA;
  }

  public double getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public boolean isConnected() {
    return inputs.motorConnected;
  }

  /** Zeroes the mechanism encoder and confirms zero only when the hardware write succeeds. */
  public boolean resetPosition() {
    io.stop();
    boolean ok = io.resetPosition();
    if (ok) {
      targetPosition = 0.0;
      zeroConfirmed = true;
    }
    Logger.recordOutput(spec.mechanism() + "/ZeroConfirmed", zeroConfirmed);
    return ok;
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public MechanismSpec getSpec() {
    return spec;
  }
}
