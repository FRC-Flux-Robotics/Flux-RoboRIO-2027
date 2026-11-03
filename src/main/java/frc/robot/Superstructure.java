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
  public static final double DEFAULT_SHORT_SHOOTER_RPS = 52.0;
  public static final double DEFAULT_SHORT_HOOD_ROT = 1.80;
  public static final double DEFAULT_MID_SHOOTER_RPS = 69.0;
  public static final double DEFAULT_MID_HOOD_ROT = 5.10;
  public static final double DEFAULT_LONG_SHOOTER_RPS = 88.0;
  public static final double DEFAULT_LONG_HOOD_ROT = 8.40;
  public static final double SHOOTER_TOLERANCE_RPS = 2.0;
  public static final double HOOD_TOLERANCE_ROT = 0.15;
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

  private State state = State.IDLE;
  private double targetShooterRps = 0.0;
  private double targetHoodRot = 0.0;
  private int virtualGamePieceCount = 1;
  private boolean shotDischargedInCurrentBurst = false;

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
    state = desiredState;
    applyStateSetpoints();
  }

  public void applyShotSolution(ShotSolution solution, boolean fireWhenReady) {
    targetShooterRps = solution.shooterRps();
    targetHoodRot = solution.hoodRotations();
    State nextState = fireWhenReady ? State.SHOOTING : State.PREPPING_SHOT;
    if (nextState != State.SHOOTING) {
      shotDischargedInCurrentBurst = false;
    }
    state = nextState;
    applyStateSetpoints();
  }

  public void prepShot(double shooterRps, double hoodRotations) {
    targetShooterRps = shooterRps;
    targetHoodRot = hoodRotations;
    shotDischargedInCurrentBurst = false;
    state = State.PREPPING_SHOT;
    applyStateSetpoints();
  }

  public void shoot(double shooterRps, double hoodRotations) {
    targetShooterRps = shooterRps;
    targetHoodRot = hoodRotations;
    if (state != State.SHOOTING) {
      shotDischargedInCurrentBurst = false;
    }
    state = State.SHOOTING;
    applyStateSetpoints();
  }

  private void applyStateSetpoints() {
    switch (state) {
      case IDLE -> {
        intake.stop();
        indexer.stop();
        feeder.stop();
        shooter.stop();
        tilter.setPosition(tilterStowRot.get());
        hood.setPosition(0.0);
      }
      case INTAKING -> {
        tilter.setPosition(tilterDeployRot.get());
        intake.setVelocity(intakeInRps.get());
        indexer.setVelocity(indexerRps.get());
        feeder.stop();
        shooter.stop();
        hood.setPosition(0.0);
      }
      case OUTTAKING -> {
        tilter.setPosition(tilterDeployRot.get());
        intake.setVelocity(DEFAULT_INTAKE_OUT_RPS);
        indexer.setVelocity(-indexerRps.get());
        feeder.setVelocity(-feederRps.get());
        shooter.stop();
      }
      case PREPPING_SHOT -> {
        intake.stop();
        indexer.stop();
        feeder.stop();
        shooter.setVelocity(targetShooterRps);
        hood.setPosition(targetHoodRot);
      }
      case SHOOTING -> {
        shooter.setVelocity(targetShooterRps);
        hood.setPosition(targetHoodRot);
        // Safety interlock: only feed game piece once shooter & hood are within tolerance
        if (isReadyToFire()) {
          double filteredIndexerV =
              faultSupervisor.filterMechanismCommandVoltage(
                  8.0, indexer.getStatorCurrentAmps(), indexer.getVelocity());
          if (filteredIndexerV < 0.0) {
            indexer.setVoltage(filteredIndexerV);
            feeder.stop();
          } else {
            indexer.setVelocity(indexerRps.get());
            feeder.setVelocity(feederRps.get());
            if (!shotDischargedInCurrentBurst && virtualGamePieceCount > 0) {
              virtualGamePieceCount--;
              shotDischargedInCurrentBurst = true;
            }
          }
        } else {
          shotDischargedInCurrentBurst = false;
          indexer.stop();
          feeder.stop();
        }
      }
      case CLIMBING -> {
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
   * Returns true when both the dual counter-rotating shooter and hood have reached their target
   * setpoints (enforcing the shot-release interlock).
   */
  public boolean isReadyToFire() {
    return targetShooterRps > 10.0 && shooter.atTarget() && hood.atTarget();
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        this::applyStateSetpoints,
        intakeInRps,
        indexerRps,
        feederRps,
        tilterDeployRot,
        tilterStowRot);
    if (state == State.SHOOTING) {
      applyStateSetpoints();
    }

    double tilterPitchRad =
        (tilter.getPosition() / DEFAULT_TILTER_DEPLOY_ROT)
            * Math.toRadians(TILTER_MAX_PITCH_DEG);
    double hoodPitchRad =
        (hood.getPosition() / HOOD_MAX_ROT) * Math.toRadians(HOOD_MAX_PITCH_DEG);
    Pose3d[] mechPoses =
        new Pose3d[] {
          new Pose3d(0.32, 0.0, 0.18, new Rotation3d(0.0, tilterPitchRad, 0.0)),
          new Pose3d(-0.12, 0.0, 0.46, new Rotation3d(0.0, -hoodPitchRad, 0.0))
        };

    Logger.recordOutput("Superstructure/State", state.name());
    Logger.recordOutput("Superstructure/ReadyToFire", isReadyToFire());
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
