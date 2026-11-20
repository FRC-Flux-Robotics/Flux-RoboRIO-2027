package frc.robot.exercises;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.DriveToPose;
import frc.lib.drive.Drive;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.autos.StudentAutoRoutines;
import frc.robot.util.ShotCalculator;

/**
 * Progressive 4-Level Hands-On Student Exercise Kit for {@code Flux-RoboRIO-2027} ({@code FUEL} &amp;
 * {@code CORAL}).
 *
 * <p>Run {@code ./gradlew checkExercises} in the terminal (or VS Code Test Explorer) to grade all 4
 * exercises in under 3 seconds with zero physical robot required.
 */
public final class StudentExercises {

  public static final double TRANSLATION_DEADBAND_METERS = 0.020; // 2.0 cm
  public static final double HEADING_DEADBAND_RAD = Math.toRadians(1.0); // 1.0 deg
  public static final double SHOOTER_READY_TOLERANCE_RPS = Superstructure.SHOOTER_TOLERANCE_RPS;
  public static final double HOOD_READY_TOLERANCE_ROT = Superstructure.HOOD_TOLERANCE_ROT;

  /** Output record for Exercise 2 (2026 {@code FUEL} shot-release readiness gate). */
  public record ShotGateDecision(
      boolean readyToFire, double feederCommandRps, double indexerCommandRps) {}

  private StudentExercises() {}

  /**
   * Returns an intentionally broken {@link MechanismSpec} containing the 4 classic hardware config
   * bugs (forbidden odd SystemCore port {@code can_s1}, {@code 120 A} stator/slip limit causing
   * NYTR brownouts, {@code 70 A} supply limit, and inverted soft limits).
   */
  public static MechanismSpec createBrokenHoodSpecExample() {
    return new MechanismSpec(
        "FUEL",
        "Hood",
        12,
        "can_s1", // BUG 1: Invalid odd CAN port instead of "Mech" CANivore
        120.0, // BUG 2: > 60A stator limit causes battery brownouts
        70.0, // BUG 3: > 40A supply limit trips breaker / sags bus
        120.0, // BUG 4: > 60A slip current overwrites stator limit on TalonFX
        "ARM_COSINE",
        4.2,
        0.0,
        0.12,
        0.18,
        0.35,
        0.015,
        0.22,
        12.5, // BUG 5: minPosRot > maxPosRot
        0.0,
        -1,
        false);
  }

  /**
   * Returns true if {@code spec} satisfies all FLUX 10413 hardware safety rules:
   *
   * <ul>
   *   <li>Never uses odd SystemCore CAN FD ports ({@code can_s1} or {@code can_s3})
   *   <li>{@code statorLimitAmps in (0, 60.0]}, {@code supplyLimitAmps in (0, 40.0]}, {@code
   *       slipCurrentAmps in (0, 60.0]}
   *   <li>{@code minPosRot <= maxPosRot}
   * </ul>
   */
  public static boolean isMechanismSpecSafe(MechanismSpec spec) {
    if (spec == null || spec.canBus() == null) {
      return false;
    }
    String bus = spec.canBus().trim();
    if ("can_s1".equalsIgnoreCase(bus) || "can_s3".equalsIgnoreCase(bus)) {
      return false;
    }
    if (spec.statorLimitAmps() <= 0.0 || spec.statorLimitAmps() > 60.0) {
      return false;
    }
    if (spec.supplyLimitAmps() <= 0.0 || spec.supplyLimitAmps() > 40.0) {
      return false;
    }
    if (spec.slipCurrentAmps() <= 0.0 || spec.slipCurrentAmps() > 60.0) {
      return false;
    }
    return spec.minPosRot() <= spec.maxPosRot();
  }

  /**
   * Exercise 1 (Level 1 — 10 min): Fix a broken {@link MechanismSpec} so it passes all CAN bus and
   * current-limit safety rules.
   */
  public static MechanismSpec exercise1_fixFuelMechanismSpec(MechanismSpec broken) {
    // TODO(student): Exercise 1 (Level 1) — Sanitize invalid CAN ports (can_s1/can_s3 -> Mech), current limits (<=60A stator/slip, <=40A supply), and soft limits, then run `./gradlew checkExercises`
    String safeBus = broken.canBus();
    if ("can_s1".equalsIgnoreCase(safeBus) || "can_s3".equalsIgnoreCase(safeBus)) {
      safeBus = "Mech";
    }
    double safeStator = MathUtil.clamp(broken.statorLimitAmps(), 10.0, 60.0);
    double safeSupply = MathUtil.clamp(broken.supplyLimitAmps(), 10.0, 40.0);
    double safeSlip = MathUtil.clamp(broken.slipCurrentAmps(), 10.0, 60.0);
    double minRot = Math.min(broken.minPosRot(), broken.maxPosRot());
    double maxRot = Math.max(broken.minPosRot(), broken.maxPosRot());

    return new MechanismSpec(
        broken.robot(),
        broken.mechanism(),
        broken.motorId(),
        safeBus,
        safeStator,
        safeSupply,
        safeSlip,
        broken.gravityType(),
        broken.kP(),
        broken.kI(),
        broken.kD(),
        broken.kS(),
        broken.kV(),
        broken.kA(),
        broken.kG(),
        minRot,
        maxRot,
        broken.followerMotorId(),
        broken.invertFollower(),
        broken.motorInverted(),
        broken.gearRatio());
  }

  /**
   * Exercise 2 (Level 2 — 15 min): Implement the 2026 {@code FUEL} shot-release readiness gate so
   * {@code Feeder} and {@code Indexer} only feed a ball once {@code Shooter} RPS and {@code Hood}
   * angle have settled within tolerance.
   */
  public static ShotGateDecision exercise2_computeShotReadinessAndFeederCommand(
      double currentShooterRps,
      double targetShooterRps,
      double currentHoodRot,
      double targetHoodRot,
      double desiredFeederRps,
      double desiredIndexerRps) {
    // TODO(student): Exercise 2 (Level 2) — Gate Feeder & Indexer at 0.0 RPS until Shooter is within +/-2.0 RPS and Hood is within +/-0.15 rot, then run `./gradlew checkExercises`
    boolean shooterReady =
        targetShooterRps > 10.0
            && Math.abs(currentShooterRps - targetShooterRps) <= SHOOTER_READY_TOLERANCE_RPS;
    boolean hoodReady = Math.abs(currentHoodRot - targetHoodRot) <= HOOD_READY_TOLERANCE_ROT;
    boolean readyToFire = shooterReady && hoodReady;

    return new ShotGateDecision(
        readyToFire,
        readyToFire ? desiredFeederRps : 0.0,
        readyToFire ? desiredIndexerRps : 0.0);
  }

  /**
   * Exercise 3 (Level 3 — 20 min): Compute robot-relative {@link ChassisSpeeds} to align the swerve
   * chassis from {@code currentPose} to {@code targetPose} with shortest-path angle wrapping and
   * deadbands.
   */
  public static ChassisSpeeds exercise3_computePoseAlignmentSpeeds(
      Pose2d currentPose,
      Pose2d targetPose,
      double kPTranslation,
      double kPTheta,
      double maxLinearSpeedMps,
      double maxOmegaRadPerSec) {
    // TODO(student): Exercise 3 (Level 3) — Compute swerve alignment ChassisSpeeds with 2.0 cm / 1.0 deg deadbands and `MathUtil.angleModulus` shortest-path heading wrap, then run `./gradlew checkExercises`
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double distMeters = Math.hypot(dx, dy);

    double vxField = 0.0;
    double vyField = 0.0;
    if (distMeters > TRANSLATION_DEADBAND_METERS) {
      double speedCmd = Math.min(maxLinearSpeedMps, distMeters * kPTranslation);
      vxField = (dx / distMeters) * speedCmd;
      vyField = (dy / distMeters) * speedCmd;
    }

    double headingErrRad =
        MathUtil.angleModulus(
            targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
    double omegaCmd = 0.0;
    if (Math.abs(headingErrRad) > HEADING_DEADBAND_RAD) {
      omegaCmd = MathUtil.clamp(headingErrRad * kPTheta, -maxOmegaRadPerSec, maxOmegaRadPerSec);
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        vxField, vyField, omegaCmd, currentPose.getRotation());
  }

  /**
   * Exercise 4 (Level 4 — 25 min): Build a 15-second 3-Volley 2026 Autonomous Routine (Preload +
   * PathPlanner {@code "Hub to Depot"} + South Wing Sweep) and visualize it in 3D via {@code
   * ./gradlew renderAuto}.
   */
  public static Command exercise4_createMultiVolleyHubAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    // TODO(student): Exercise 4 (Level 4) — Customize this 15s 3-volley 2026 Hub + PathPlanner "Hub to Depot" + South Wing autonomous cycle, then run `./gradlew checkExercises` and `./gradlew renderAuto`
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(3.10, 4.05, Rotation2d.kZero));
                  superstructure.prepShot(
                      Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                      Superstructure.DEFAULT_SHORT_HOOD_ROT);
                }),
            // Volley 1: Fire preload into Blue Hub (4.626m, 4.035m)
            StudentAutoRoutines.createAimAndShootHubCommand(
                drive, superstructure, shotCalculator, 2.0),
            // PathPlanner "Hub to Depot": Collect 3 Depot FUEL balls and return to Hub
            StudentAutoRoutines.withLoopTimeout(drive.followPathCommand("Hub to Depot"), 5.4),
            // Volley 2: Fire Depot volley into Blue Hub
            StudentAutoRoutines.createAimAndShootHubCommand(
                drive, superstructure, shotCalculator, 2.4),
            // Volley 3: Sweep South Wing & Shoot-on-the-Move orbit
            Commands.runOnce(() -> superstructure.requestState(State.INTAKING)),
            StudentAutoRoutines.withLoopTimeout(
                new DriveToPose(drive, new Pose2d(2.55, 2.55, Rotation2d.fromDegrees(40.0))), 2.2),
            StudentAutoRoutines.createShootOnTheMoveCircleCommand(
                drive, superstructure, shotCalculator, 0.80, 3.0))
        .withName("STUDENT EXERCISE 4: 2026 Hub + Depot + Wing 3-Volley Auto");
  }
}
