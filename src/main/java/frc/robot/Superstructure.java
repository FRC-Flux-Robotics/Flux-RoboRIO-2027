package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.util.ShotCalculator.ShotSolution;
import org.littletonrobotics.junction.Logger;

/**
 * 2026 {@code FUEL} / {@code CORAL} Superstructure state machine coordinating all 6 mechanisms
 * ({@code Intake}, {@code Tilter}, {@code Indexer}, {@code Feeder}, dual counter-rotating {@code
 * Shooter}, and {@code Hood}) with safety interlocks, {@link FaultSupervisor} automatic reverse
 * unjamming, and 3D {@link Pose3d} visualization for Ghost Superstructure mode.
 */
public class Superstructure extends SubsystemBase {

  /** High-level 2026 robot superstructure operational states. */
  public enum State {
    IDLE,
    INTAKING,
    PREPPING_SHOT,
    SHOOTING,
    OUTTAKING,
    CLIMBING
  }

  public static final double DEFAULT_INTAKE_IN_RPS = 60.0;
  public static final double DEFAULT_INTAKE_OUT_RPS = -60.0;
  public static final double DEFAULT_INDEXER_RPS = 45.0;
  public static final double DEFAULT_FEEDER_RPS = -40.0;
  public static final double DEFAULT_TILTER_STOW_ROT = 0.0;
  public static final double DEFAULT_TILTER_DEPLOY_ROT = -17.0;
  public static final double DEFAULT_TILTER_CLIMB_ROT = -6.0;
  public static final double DEFAULT_SHOOTER_IDLE_RPS = 0.0;
  public static final double DEFAULT_SHORT_SHOOTER_RPS = 33.33;
  public static final double DEFAULT_SHORT_HOOD_ROT = 2.80;
  public static final double DEFAULT_MID_SHOOTER_RPS = 36.67;
  public static final double DEFAULT_MID_HOOD_ROT = 3.90;
  public static final double DEFAULT_LONG_SHOOTER_RPS = 40.0;
  public static final double DEFAULT_LONG_HOOD_ROT = 8.85;
  public static final double SHOOTER_TOLERANCE_RPS = 2.0;
  public static final double HOOD_TOLERANCE_ROT = 0.15;
  public static final double AIM_HEADING_TOLERANCE_RAD = Math.toRadians(5.0);
  public static final double TILTER_MAX_PITCH_DEG = 65.0;
  public static final double HOOD_MAX_ROT = 12.5;
  public static final double HOOD_MAX_PITCH_DEG = 38.0;

  private final VelocityMechanism intake;
  private final PositionMechanism tilter;
  private final VelocityMechanism indexer;
  private final VelocityMechanism feeder;
  private final VelocityMechanism shooter;
  private final PositionMechanism hood;
  private final FaultSupervisor faultSupervisor;

  private final LoggedTunableNumber intakeInRps =
      new LoggedTunableNumber("Superstructure/IntakeInRPS", DEFAULT_INTAKE_IN_RPS);
  private final LoggedTunableNumber indexerRps =
      new LoggedTunableNumber("Superstructure/IndexerRPS", DEFAULT_INDEXER_RPS);
  private final LoggedTunableNumber feederRps =
      new LoggedTunableNumber("Superstructure/FeederRPS", DEFAULT_FEEDER_RPS);
  private final LoggedTunableNumber tilterDeployRot =
      new LoggedTunableNumber("Superstructure/TilterDeployRot", DEFAULT_TILTER_DEPLOY_ROT);
  private final LoggedTunableNumber tilterStowRot =
      new LoggedTunableNumber("Superstructure/TilterStowRot", DEFAULT_TILTER_STOW_ROT);

  private final Pose3d[] mechPoses = new Pose3d[2];

  private State state = State.IDLE;
  private double targetShooterRps = 0.0;
  private double targetHoodRot = 0.0;
  private double aimHeadingErrorRad = 0.0;
  private boolean shotInRange = true;
  private int virtualGamePieceCount = 1;
  private boolean shotDischargedInCurrentBurst = false;
  private boolean pendingTunableRefresh = false;
  private boolean intakeUnjamming = false;
  private double intakeUnjamVolts = 0.0;
  private boolean indexerUnjamming = false;
  private double indexerUnjamVolts = 0.0;
  private boolean feederUnjamming = false;
  private double feederUnjamVolts = 0.0;

  public Superstructure(
      VelocityMechanism intake,
      PositionMechanism tilter,
      VelocityMechanism indexer,
      VelocityMechanism feeder,
      VelocityMechanism shooter,
      PositionMechanism hood,
      FaultSupervisor faultSupervisor) {
    this.intake = intake;
    this.tilter = tilter;
    this.indexer = indexer;
    this.feeder = feeder;
    this.shooter = shooter;
    this.hood = hood;
    this.faultSupervisor = faultSupervisor;
  }

  /** Transitions the 2026 superstructure to the requested state and applies safety interlocks. */
  public void requestState(State desiredState) {
    if (desiredState == State.INTAKING && state != State.INTAKING && virtualGamePieceCount < 3) {
      virtualGamePieceCount++;
    }
    if (desiredState != State.SHOOTING) {
      shotDischargedInCurrentBurst = false;
    }
    if (desiredState == State.IDLE) {
      targetShooterRps = 0.0;
      targetHoodRot = 0.0;
    }
    state = desiredState;
    applyStateSetpoints(false);
  }

  /** Updates the latest chassis heading error and ballistic range gate for {@link #isReadyToFire()}. */
  public void updateShotAimStatus(double headingErrorRad, boolean inRange) {
    aimHeadingErrorRad = Math.abs(headingErrorRad);
    shotInRange = inRange;
  }

  /**
   * Fail-closed overload: without an explicit chassis aim error and range check, {@link
   * #isReadyToFire()} remains blocked until {@link #applyShotSolution(ShotSolution, boolean,
   * double, boolean)} or {@link #updateShotAimStatus(double, boolean)} confirms alignment.
   */
  public void applyShotSolution(ShotSolution solution, boolean fireWhenReady) {
    applyShotSolution(solution, fireWhenReady, Double.POSITIVE_INFINITY, false);
  }

  public void applyShotSolution(
      ShotSolution solution, boolean fireWhenReady, double headingErrorRad, boolean inRange) {
    targetShooterRps = solution.shooterRps();
    targetHoodRot = solution.hoodRotations();
    aimHeadingErrorRad = Math.abs(headingErrorRad);
    shotInRange = inRange;
    State nextState = fireWhenReady ? State.SHOOTING : State.PREPPING_SHOT;
    if (nextState != State.SHOOTING) {
      shotDischargedInCurrentBurst = false;
    }
    state = nextState;
    applyStateSetpoints(false);
  }

  public void prepShot(double shooterRps, double hoodRotations) {
    targetShooterRps = shooterRps;
    targetHoodRot = hoodRotations;
    shotDischargedInCurrentBurst = false;
    state = State.PREPPING_SHOT;
    applyStateSetpoints(false);
  }

  public void shoot(double shooterRps, double hoodRotations) {
    shoot(shooterRps, hoodRotations, 0.0, true);
  }

  public void shoot(
      double shooterRps, double hoodRotations, double headingErrorRad, boolean inRange) {
    targetShooterRps = shooterRps;
    targetHoodRot = hoodRotations;
    aimHeadingErrorRad = Math.abs(headingErrorRad);
    shotInRange = inRange;
    if (state != State.SHOOTING) {
      shotDischargedInCurrentBurst = false;
    }
    state = State.SHOOTING;
    applyStateSetpoints(false);
  }

  private void clearJamChannel(int channel) {
    faultSupervisor.filterMechanismCommandVoltage(channel, 0.0, 0.0, 0.0);
    if (channel == FaultSupervisor.JAM_CHANNEL_INTAKE) {
      intakeUnjamming = false;
      intakeUnjamVolts = 0.0;
    } else if (channel == FaultSupervisor.JAM_CHANNEL_INDEXER) {
      indexerUnjamming = false;
      indexerUnjamVolts = 0.0;
    } else if (channel == FaultSupervisor.JAM_CHANNEL_FEEDER) {
      feederUnjamming = false;
      feederUnjamVolts = 0.0;
    }
  }

  private void clearAllJamChannels() {
    clearJamChannel(FaultSupervisor.JAM_CHANNEL_INTAKE);
    clearJamChannel(FaultSupervisor.JAM_CHANNEL_INDEXER);
    clearJamChannel(FaultSupervisor.JAM_CHANNEL_FEEDER);
  }

  private void applyStateSetpoints(boolean tickJamSupervisor) {
    switch (state) {
      case IDLE -> {
        clearAllJamChannels();
        intake.stop();
        indexer.stop();
        feeder.stop();
        shooter.stop();
        tilter.setPosition(tilterStowRot.get());
        hood.setPosition(0.0);
      }
      case INTAKING -> {
        clearJamChannel(FaultSupervisor.JAM_CHANNEL_FEEDER);
        tilter.setPosition(tilterDeployRot.get());
        if (tickJamSupervisor) {
          double filteredIntakeV =
              faultSupervisor.filterMechanismCommandVoltage(
                  FaultSupervisor.JAM_CHANNEL_INTAKE,
                  8.0,
                  intake.getStatorCurrentAmps(),
                  intake.getVelocity());
          intakeUnjamming =
              faultSupervisor.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_INTAKE)
                  || filteredIntakeV < 0.0;
          intakeUnjamVolts = filteredIntakeV;

          double filteredIndexerV =
              faultSupervisor.filterMechanismCommandVoltage(
                  FaultSupervisor.JAM_CHANNEL_INDEXER,
                  8.0,
                  indexer.getStatorCurrentAmps(),
                  indexer.getVelocity());
          indexerUnjamming =
              faultSupervisor.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_INDEXER)
                  || filteredIndexerV < 0.0;
          indexerUnjamVolts = filteredIndexerV;
        }
        if (intakeUnjamming) {
          intake.setVoltage(intakeUnjamVolts);
        } else {
          intake.setVelocity(intakeInRps.get());
        }
        if (indexerUnjamming) {
          indexer.setVoltage(indexerUnjamVolts);
        } else {
          indexer.setVelocity(indexerRps.get());
        }
        feeder.stop();
        shooter.stop();
        hood.setPosition(0.0);
      }
      case OUTTAKING -> {
        clearAllJamChannels();
        tilter.setPosition(tilterDeployRot.get());
        intake.setVelocity(DEFAULT_INTAKE_OUT_RPS);
        indexer.setVelocity(-indexerRps.get());
        feeder.setVelocity(-feederRps.get());
        shooter.stop();
      }
      case PREPPING_SHOT -> {
        clearAllJamChannels();
        intake.stop();
        tilter.setPosition(tilterStowRot.get());
        indexer.stop();
        feeder.stop();
        shooter.setVelocity(targetShooterRps);
        hood.setPosition(targetHoodRot);
      }
      case SHOOTING -> {
        clearJamChannel(FaultSupervisor.JAM_CHANNEL_INTAKE);
        intake.stop();
        tilter.setPosition(tilterStowRot.get());
        shooter.setVelocity(targetShooterRps);
        hood.setPosition(targetHoodRot);
        // Safety interlock: only feed once shooter, hood, chassis aim heading, and range are ready
        if (isReadyToFire()) {
          if (tickJamSupervisor) {
            double filteredIndexerV =
                faultSupervisor.filterMechanismCommandVoltage(
                    FaultSupervisor.JAM_CHANNEL_INDEXER,
                    8.0,
                    indexer.getStatorCurrentAmps(),
                    indexer.getVelocity());
            indexerUnjamming =
                faultSupervisor.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_INDEXER)
                    || filteredIndexerV < 0.0;
            indexerUnjamVolts = filteredIndexerV;
            if (!indexerUnjamming) {
              double desiredFeederSignV = Math.copySign(8.0, feederRps.get());
              double filteredFeederV =
                  faultSupervisor.filterMechanismCommandVoltage(
                      FaultSupervisor.JAM_CHANNEL_FEEDER,
                      desiredFeederSignV,
                      feeder.getStatorCurrentAmps(),
                      feeder.getVelocity());
              feederUnjamming =
                  faultSupervisor.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_FEEDER)
                      || Math.signum(filteredFeederV) != Math.signum(desiredFeederSignV);
              feederUnjamVolts = filteredFeederV;
            }
          }
          if (indexerUnjamming) {
            indexer.setVoltage(indexerUnjamVolts);
            feeder.stop();
          } else if (feederUnjamming) {
            feeder.setVoltage(feederUnjamVolts);
            indexer.stop();
          } else {
            indexer.setVelocity(indexerRps.get());
            feeder.setVelocity(feederRps.get());
            if (!shotDischargedInCurrentBurst && virtualGamePieceCount > 0) {
              virtualGamePieceCount--;
              shotDischargedInCurrentBurst = true;
            }
          }
        } else {
          clearJamChannel(FaultSupervisor.JAM_CHANNEL_INDEXER);
          clearJamChannel(FaultSupervisor.JAM_CHANNEL_FEEDER);
          shotDischargedInCurrentBurst = false;
          indexer.stop();
          feeder.stop();
        }
      }
      case CLIMBING -> {
        clearAllJamChannels();
        intake.stop();
        indexer.stop();
        feeder.stop();
        shooter.stop();
        hood.setPosition(0.0);
        tilter.setPosition(DEFAULT_TILTER_CLIMB_ROT);
      }
    }
  }

  /**
   * Returns true when the dual shooter, hood, chassis aim heading error, and ballistic range are
   * all within tolerance (enforcing the shot-release interlock).
   */
  public boolean isReadyToFire() {
    return targetShooterRps > 10.0
        && shooter.atTarget()
        && hood.atTarget()
        && shotInRange
        && Double.isFinite(aimHeadingErrorRad)
        && Math.abs(aimHeadingErrorRad) <= AIM_HEADING_TOLERANCE_RAD;
  }

  @Override
  public void periodic() {
    pendingTunableRefresh = false;
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> pendingTunableRefresh = true,
        intakeInRps,
        indexerRps,
        feederRps,
        tilterDeployRot,
        tilterStowRot);
    if (state == State.SHOOTING || state == State.INTAKING) {
      applyStateSetpoints(true);
    } else if (pendingTunableRefresh) {
      applyStateSetpoints(false);
    }

    double tilterPitchRad =
        (tilter.getPosition() / DEFAULT_TILTER_DEPLOY_ROT)
            * Math.toRadians(TILTER_MAX_PITCH_DEG);
    double hoodPitchRad =
        (hood.getPosition() / HOOD_MAX_ROT) * Math.toRadians(HOOD_MAX_PITCH_DEG);
    mechPoses[0] = new Pose3d(0.32, 0.0, 0.18, new Rotation3d(0.0, tilterPitchRad, 0.0));
    mechPoses[1] = new Pose3d(-0.12, 0.0, 0.46, new Rotation3d(0.0, -hoodPitchRad, 0.0));

    Logger.recordOutput("Superstructure/State", state.name());
    Logger.recordOutput("Superstructure/ReadyToFire", isReadyToFire());
    Logger.recordOutput("Superstructure/ShotInRange", shotInRange);
    Logger.recordOutput(
        "Superstructure/AimHeadingErrorDeg",
        Double.isFinite(aimHeadingErrorRad) ? Math.toDegrees(aimHeadingErrorRad) : 999.0);
    Logger.recordOutput("Superstructure/TargetShooterRPS", targetShooterRps);
    Logger.recordOutput("Superstructure/TargetHoodRot", targetHoodRot);
    Logger.recordOutput("Superstructure/VirtualGamePieces", virtualGamePieceCount);
    Logger.recordOutput("Superstructure/MechanismPoses3d", mechPoses);
  }

  /** Sums the total supply current across all 6 mechanisms (for VirtualBatterySim / Ghost mode). */
  public double getTotalMechanismSupplyCurrentAmps() {
    return intake.getSupplyCurrentAmps()
        + tilter.getSupplyCurrentAmps()
        + indexer.getSupplyCurrentAmps()
        + feeder.getSupplyCurrentAmps()
        + shooter.getSupplyCurrentAmps()
        + hood.getSupplyCurrentAmps();
  }

  public State getState() {
    return state;
  }

  public VelocityMechanism getIntake() {
    return intake;
  }

  public PositionMechanism getTilter() {
    return tilter;
  }

  public VelocityMechanism getIndexer() {
    return indexer;
  }

  public VelocityMechanism getFeeder() {
    return feeder;
  }

  public VelocityMechanism getShooter() {
    return shooter;
  }

  public PositionMechanism getHood() {
    return hood;
  }
}
