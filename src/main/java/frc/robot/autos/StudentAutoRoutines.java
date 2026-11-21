package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.DriveToPose;
import frc.lib.drive.Drive;
import frc.lib.util.FieldPositions;
import frc.lib.util.SafeAutoBuilder;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Pure WPILib Autonomous {@link Command} Definitions for {@code Flux-RoboRIO-2027} ({@code FUEL}).
 *
 * <p>Provides:
 *
 * <ol>
 *   <li>{@link #createStudentDefaultAuto(Drive, Superstructure, ShotCalculator)} — Empty starter
 *       autonomous routine (selected by default in {@code Auto/Chooser} and rendered by {@code
 *       ./gradlew renderAuto}) where students build and visualize their own 2026 autonomous
 *       routine.
 *   <li><b>Track A (Fixed Code Moves):</b> {@link #createFixedMovesExampleAuto(Drive,
 *       Superstructure, ShotCalculator)} and {@link #createExampleSquareCircleAndShootAuto(Drive,
 *       Superstructure, ShotCalculator)} demonstrating relative moves ({@link #driveRelative}),
 *       timed velocity moves ({@link #driveTimedVelocity}), loop-counted waits ({@link
 *       #waitLoopSeconds}), and field-pose PID ({@link DriveToPose}).
 *   <li><b>Track B (PathPlanner {@code .path} Moves):</b> {@link #createFullCyclePathPlannerAuto},
 *       {@link #createHubToDepotShowcaseAuto}, {@link #createHubToDepotAuto}, {@link
 *       #createLeftToDepotAuto}, and {@link #createRightToDepotAuto} demonstrating {@link
 *       #resetPoseToPathStart} and single/multi-path {@link Drive#followPathCommand(String)}
 *       chaining across {@code "Hub to Depot"}, {@code "Left to Depot"}, {@code "Right to Depot"},
 *       and {@code "Collect"}.
 * </ol>
 */
public final class StudentAutoRoutines {
  public static final String DEFAULT_STUDENT_AUTO_NAME = "STUDENT: Left-Right-Left-Shoot Autonomous";
  public static final String FIXED_MOVES_AUTO_NAME = "EXAMPLE: 2026 Fixed Code Moves + Hub Shoot";
  public static final String FULL_CYCLE_PATHPLANNER_AUTO_NAME =
      "EXAMPLE: 2026 PathPlanner 2-Path (Hub to Depot + Collect)";
  public static final String HUB_TO_DEPOT_SHOWCASE_AUTO_NAME =
      "EXAMPLE: 2026 Hub to Depot + Wing 3-Volley";
  public static final String HUB_TO_DEPOT_AUTO_NAME = "EXAMPLE: 2026 Hub to Depot 2-Volley";
  public static final String LEFT_TO_DEPOT_AUTO_NAME = "EXAMPLE: 2026 Left to Depot 2-Volley";
  public static final String RIGHT_TO_DEPOT_AUTO_NAME = "EXAMPLE: 2026 Right to Depot 2-Volley";
  public static final String EXAMPLE_AUTO_NAME = "EXAMPLE: 2026 Wing Square, Circle & Shoot";

  private StudentAutoRoutines() {}

  /**
   * Default Student 2026 Autonomous Routine (selected by default in {@code Auto/Chooser} and
   * rendered by {@code ./gradlew renderAuto}).
   *
   * <p>Starts empty at {@code (3.10m, 4.05m, 0°)} in front of the 2026 Blue Hub so you can build
   * and visualize your own 15-second autonomous routine from scratch.
   */
  public static Command createStudentDefaultAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    // STUDENT: Build your own 15-second 2026 autonomous routine inside Commands.sequence(...) below, then run `./gradlew renderAuto` and open `build/auto-sim/student-auto.html`!
    // Track A — Fixed Code Moves (see createFixedMovesExampleAuto & createExampleSquareCircleAndShootAuto below):
    //   • Relative move:   driveRelative(drive, 1.20, 0.0, 0.0, 2.0)        // forward +X m, left +Y m, turn +CCW deg
    //   • Turn in place:   driveRelative(drive, 0.0, 0.0, -90.0, 1.2)
    //   • Timed velocity:  driveTimedVelocity(drive, -0.80, 0.0, 0.0, 0.8)  // vx m/s, vy m/s, omega deg/s, seconds
    //   • Field pose PID:  withLoopTimeout(new DriveToPose(drive, new Pose2d(2.55, 2.55, Rotation2d.fromDegrees(40.0))), 2.2)
    //   • Wait / timeout:  use waitLoopSeconds(0.5) & withLoopTimeout(cmd, 2.0) — NOT Commands.waitSeconds / .withTimeout!
    // Track B — PathPlanner .path Moves (see createFullCyclePathPlannerAuto & createHubToDepotAuto below):
    //   • Snap start pose: resetPoseToPathStart(drive, "Hub to Depot")      // auto-reads start pose of any .path file
    //   • Follow .path:    withLoopTimeout(drive.followPathCommand("Hub to Depot"), 5.5)
    //                      (paths in deploy/pathplanner/paths/: "Hub to Depot", "Left to Depot", "Right to Depot", "Collect")
    // Shooting & Intake Helpers:
    //   • Toggle intake:   Commands.runOnce(() -> superstructure.requestState(State.INTAKING))
    //   • Aim & shoot Hub: createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.0)
    //   • Orbit & shoot:   createShootOnTheMoveCircleCommand(drive, superstructure, shotCalculator, 0.80, 3.0)
    return Commands.sequence
    (
        Commands.runOnce
        (
            () -> 
            {
                drive.setPose(new Pose2d(3.10, 4.05, Rotation2d.kZero));
                // superstructure.requestState(State.IDLE); // OG code
                superstructure.prepShot
                (
                    Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                    Superstructure.DEFAULT_SHORT_HOOD_ROT
                );
            }
        ),
        createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.0),
        Commands.runOnce(() -> superstructure.requestState(State.INTAKING)),
        driveRelative(drive, 0.0, 0.0, -90.0, 2.0),
        driveRelative(drive, 2.0, 0.0, 0.0, 3.0),
        driveRelative(drive, 0.0, 0.0, 180.0, 2.0),
        driveRelative(drive, 4.0, 0.0, 0.0, 4.0),
        driveRelative(drive, 0.0, 0.0, -90.0, 2.0),
        driveRelative(drive, 0.0, 2.0, 0.0, 3.0),
        createAimAndShootHubCommand(drive, superstructure, shotCalculator, 4.0),
        Commands.runOnce(() -> superstructure.requestState(State.IDLE))
    )
    .withName(DEFAULT_STUDENT_AUTO_NAME);
  }

  /**
   * Track A Reference Example (Fixed Code Moves — zero PathPlanner files or field coordinates
   * needed): fires the 3-ball preload volley, uses {@link #driveRelative}, {@link
   * #waitLoopSeconds}, and {@link #driveTimedVelocity} to collect both South Wing {@code FUEL}
   * balls, turns back toward the Blue Hub, and fires the collected volley (render via {@code
   * ./gradlew renderAuto -Pauto="Fixed Code Moves"}).
   */
  public static Command createFixedMovesExampleAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(3.10, 4.05, Rotation2d.kZero));
                  superstructure.prepShot(
                      Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                      Superstructure.DEFAULT_SHORT_HOOD_ROT);
                }),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.0),
            Commands.runOnce(() -> superstructure.requestState(State.INTAKING)),
            driveRelative(drive, -0.30, -0.80, 0.0, 1.8),
            waitLoopSeconds(0.25),
            driveRelative(drive, 0.0, 0.0, -90.0, 1.2),
            driveRelative(drive, 0.70, 0.0, 0.0, 1.6),
            driveTimedVelocity(drive, -0.80, 0.0, 0.0, 0.8),
            driveRelative(drive, 0.0, 0.0, 90.0, 1.2),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.6),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)))
        .withName(FIXED_MOVES_AUTO_NAME);
  }

  /**
   * Track B Reference Example (Chaining Multiple PathPlanner {@code .path} Files): snaps the robot
   * to {@code "Hub to Depot.path"} start via {@link #resetPoseToPathStart}, fires the preload
   * volley, follows {@code "Hub to Depot"} (triggering {@code "startIntake"} and {@code
   * "stopIntake"} event markers), fires the Depot volley, and chains {@code "Collect.path"} out to
   * the centerline {@code X = 7.68m} to intake 3 neutral-zone {@code FUEL} balls (render via {@code
   * ./gradlew renderAuto -Pauto="PathPlanner 2-Path"}).
   */
  public static Command createFullCyclePathPlannerAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            resetPoseToPathStart(drive, "Hub to Depot"),
            Commands.runOnce(
                () ->
                    superstructure.prepShot(
                        Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                        Superstructure.DEFAULT_SHORT_HOOD_ROT)),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 1.8),
            withLoopTimeout(drive.followPathCommand("Hub to Depot"), 5.2),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.0),
            withLoopTimeout(drive.followPathCommand("Collect"), 6.0))
        .withName(FULL_CYCLE_PATHPLANNER_AUTO_NAME);
  }

  /**
   * Full 3-Volley {@code "Hub to Depot"} + South Wing Shoot-on-the-Move showcase routine (render
   * via {@code ./gradlew renderAuto -Pauto="Hub to Depot + Wing"}):
   *
   * <ol>
   *   <li>Starts at {@code (3.10m, 4.05m, 0°)} in front of the 2026 Blue Hub and fires the 3-ball
   *       preload {@code FUEL} volley.
   *   <li>Follows PathPlanner {@code "Hub to Depot.path"} into the Blue Depot {@code (1.024m,
   *       6.40m) -> (0.774m, 6.40m) -> (0.774m, 5.44m)} facing {@code 180°}, triggering {@code
   *       "startIntake"} and {@code "stopIntake"} {@code NamedCommands} to collect 3 Depot {@code
   *       FUEL} balls and return to {@code (3.047m, 4.05m, 0°)}.
   *   <li>Aims and fires the collected Depot {@code FUEL} volley into the Blue Hub.
   *   <li>Sweeps the South Wing {@code (2.55m, 2.55m)} for bonus {@code FUEL} balls and finishes
   *       with a 360° Shoot-on-the-Move orbit into the Blue Hub.
   * </ol>
   */
  public static Command createHubToDepotShowcaseAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            resetPoseToPathStart(drive, "Hub to Depot"),
            Commands.runOnce(
                () ->
                    superstructure.prepShot(
                        Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                        Superstructure.DEFAULT_SHORT_HOOD_ROT)),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.0),
            withLoopTimeout(drive.followPathCommand("Hub to Depot"), 5.4),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.4),
            Commands.runOnce(() -> superstructure.requestState(State.INTAKING)),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(2.55, 2.55, Rotation2d.fromDegrees(40.0))), 2.2),
            createShootOnTheMoveCircleCommand(drive, superstructure, shotCalculator, 0.80, 3.0))
        .withName(HUB_TO_DEPOT_SHOWCASE_AUTO_NAME);
  }

  /**
   * Focused 2-Volley {@code "Hub to Depot"} reference routine: snaps to {@code "Hub to Depot.path"}
   * start, fires the 3-ball preload volley, follows PathPlanner {@code "Hub to Depot.path"} to
   * intake 3 Depot {@code FUEL} balls, and fires the Depot volley into the Blue Hub.
   */
  public static Command createHubToDepotAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            resetPoseToPathStart(drive, "Hub to Depot"),
            Commands.runOnce(
                () ->
                    superstructure.prepShot(
                        Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                        Superstructure.DEFAULT_SHORT_HOOD_ROT)),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.2),
            withLoopTimeout(drive.followPathCommand("Hub to Depot"), 5.8),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.6),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)))
        .withName(HUB_TO_DEPOT_AUTO_NAME);
  }

  /**
   * Left-Start 2-Volley {@code "Left to Depot"} reference routine: snaps to {@code "Left to
   * Depot.path"} start {@code (3.10m, 6.55m, 0°)}, fires preload into the Blue Hub, follows
   * PathPlanner {@code "Left to Depot.path"} into the Blue Depot, and fires the collected Depot
   * volley.
   */
  public static Command createLeftToDepotAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            resetPoseToPathStart(drive, "Left to Depot"),
            Commands.runOnce(
                () ->
                    superstructure.prepShot(
                        Superstructure.DEFAULT_MID_SHOOTER_RPS,
                        Superstructure.DEFAULT_MID_HOOD_ROT)),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.2),
            withLoopTimeout(drive.followPathCommand("Left to Depot"), 6.2),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.6),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)))
        .withName(LEFT_TO_DEPOT_AUTO_NAME);
  }

  /**
   * Right-Start 2-Volley {@code "Right to Depot"} reference routine: snaps to {@code "Right to
   * Depot.path"} start {@code (3.10m, 1.55m, 0°)}, fires preload into the Blue Hub, follows
   * PathPlanner {@code "Right to Depot.path"} into the Blue Depot, and fires the collected Depot
   * volley.
   */
  public static Command createRightToDepotAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            resetPoseToPathStart(drive, "Right to Depot"),
            Commands.runOnce(
                () ->
                    superstructure.prepShot(
                        Superstructure.DEFAULT_MID_SHOOTER_RPS,
                        Superstructure.DEFAULT_MID_HOOD_ROT)),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.2),
            withLoopTimeout(drive.followPathCommand("Right to Depot"), 6.8),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.6),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)))
        .withName(RIGHT_TO_DEPOT_AUTO_NAME);
  }

  /**
   * Reference Example Autonomous Routine on the 2026 REBUILT field: starts at {@code (2.0, 2.6,
   * 0°)}, drives a rectangle on the Blue Wing while intaking floor {@code FUEL}, aims and fires at
   * the Blue Hub {@code (4.626, 4.035)}, and drives a 360° circle orbit.
   */
  public static Command createExampleSquareCircleAndShootAuto(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(2.0, 2.6, Rotation2d.kZero));
                  superstructure.requestState(State.INTAKING);
                }),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(3.3, 2.6, Rotation2d.kZero)), 2.0),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(3.3, 5.3, Rotation2d.fromDegrees(90.0))), 2.2),
            withLoopTimeout(
                new DriveToPose(drive, new Pose2d(1.8, 5.3, Rotation2d.fromDegrees(180.0))), 2.0),
            withLoopTimeout(new DriveToPose(drive, new Pose2d(2.1, 4.21, Rotation2d.kZero)), 2.0),
            createAimAndShootHubCommand(drive, superstructure, shotCalculator, 2.4),
            createShootOnTheMoveCircleCommand(drive, superstructure, shotCalculator, 0.85, 4.4))
        .withName(EXAMPLE_AUTO_NAME);
  }

  /**
   * Returns all named 2026 autonomous routines registered for {@code Auto/Chooser} and {@code
   * ./gradlew renderAuto -Pauto="<name>"}. Students can register their own named routines here to
   * compare and visualize multiple autos without overwriting {@link #createStudentDefaultAuto}.
   */
  public static Map<String, Command> createNamedAutos(
      Drive drive, Superstructure superstructure, ShotCalculator shotCalculator) {
    Map<String, Command> autos = new LinkedHashMap<>();
    autos.put(
        DEFAULT_STUDENT_AUTO_NAME, createStudentDefaultAuto(drive, superstructure, shotCalculator));
    autos.put(
        FIXED_MOVES_AUTO_NAME, createFixedMovesExampleAuto(drive, superstructure, shotCalculator));
    autos.put(
        FULL_CYCLE_PATHPLANNER_AUTO_NAME,
        createFullCyclePathPlannerAuto(drive, superstructure, shotCalculator));
    autos.put(
        HUB_TO_DEPOT_SHOWCASE_AUTO_NAME,
        createHubToDepotShowcaseAuto(drive, superstructure, shotCalculator));
    autos.put(
        HUB_TO_DEPOT_AUTO_NAME, createHubToDepotAuto(drive, superstructure, shotCalculator));
    autos.put(
        LEFT_TO_DEPOT_AUTO_NAME, createLeftToDepotAuto(drive, superstructure, shotCalculator));
    autos.put(
        RIGHT_TO_DEPOT_AUTO_NAME, createRightToDepotAuto(drive, superstructure, shotCalculator));
    autos.put(
        EXAMPLE_AUTO_NAME,
        createExampleSquareCircleAndShootAuto(drive, superstructure, shotCalculator));
    return autos;
  }

  /**
   * Closed-loop relative swerve move from the robot's current pose at step start (forward {@code
   * +X} meters, left {@code +Y} meters, turn {@code +CCW} degrees in robot coordinates), finishing
   * when within {@code 3 cm / 1.5°} or after {@code timeoutSeconds}.
   */
  public static Command driveRelative(
      Drive drive,
      double forwardMeters,
      double leftMeters,
      double turnDegrees,
      double timeoutSeconds) {
    final Pose2d[] goalPose = {Pose2d.kZero};
    Transform2d delta =
        new Transform2d(forwardMeters, leftMeters, Rotation2d.fromDegrees(turnDegrees));
    return withLoopTimeout(
        new DriveToPose(drive, () -> goalPose[0], () -> true)
            .beforeStarting(() -> goalPose[0] = drive.getPose().plus(delta)),
        timeoutSeconds);
  }

  /**
   * Open-loop timed robot-relative velocity move ({@code vxRobotMps} forward, {@code vyRobotMps}
   * left, {@code omegaDegPerSec} counter-clockwise) for {@code durationSeconds}, stopping cleanly
   * at the end.
   */
  public static Command driveTimedVelocity(
      Drive drive,
      double vxRobotMps,
      double vyRobotMps,
      double omegaDegPerSec,
      double durationSeconds) {
    ChassisSpeeds speeds =
        new ChassisSpeeds(vxRobotMps, vyRobotMps, Math.toRadians(omegaDegPerSec));
    return withLoopTimeout(Commands.run(() -> drive.runVelocity(speeds), drive), durationSeconds)
        .finallyDo(drive::stop);
  }

  /**
   * Resets {@link Drive} odometry to the starting pose of {@code pathName} ({@code
   * src/main/deploy/pathplanner/paths/<pathName>.path}) so newly drawn PathPlanner GUI paths pass
   * {@link SafeAutoBuilder} start-pose checks without hardcoding coordinates.
   */
  public static Command resetPoseToPathStart(Drive drive, String pathName) {
    return Commands.runOnce(() -> drive.getPathStartingPose(pathName).ifPresent(drive::setPose));
  }

  /**
   * Pauses a command sequence for {@code durationSeconds} using deterministic 50 Hz (20 ms) loop
   * counting (safe in both real-robot execution and fast-forward {@code ./gradlew renderAuto}).
   */
  public static Command waitLoopSeconds(double durationSeconds) {
    return SafeAutoBuilder.waitLoopSeconds(durationSeconds);
  }

  /**
   * Stationary Hub aiming and shooting command: aligns robot heading to {@link
   * ShotCalculator#calculate} and feeds {@code FUEL} game pieces once {@link
   * Superstructure#isReadyToFire()} is true.
   */
  public static Command createAimAndShootHubCommand(
      Drive drive,
      Superstructure superstructure,
      ShotCalculator shotCalculator,
      double durationSeconds) {
    final int maxLoops = Math.max(1, (int) Math.round(durationSeconds / 0.020));
    final int[] loops = {0};

    return Commands.run(
            () -> {
              Pose2d pose = drive.getPose();
              ChassisSpeeds fieldSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      drive.getChassisSpeeds(), pose.getRotation());
              ShotSolution sol =
                  shotCalculator.calculate(
                      pose,
                      fieldSpeeds,
                      FieldPositions.forAlliance(RobotContainer.HUB_TARGET_METERS));
              double yawErrRad =
                  MathUtil.angleModulus(
                      sol.driveAimHeading().minus(pose.getRotation()).getRadians());
              boolean inRange = shotCalculator.isInRange(sol.effectiveDistanceMeters());
              superstructure.applyShotSolution(sol, true, Math.abs(yawErrRad), inRange);

              double omegaCmd = MathUtil.clamp(yawErrRad * 6.5, -6.0, 6.0);
              drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omegaCmd));
              loops[0]++;
            },
            drive,
            superstructure)
        .beforeStarting(() -> loops[0] = 0)
        .until(() -> loops[0] >= maxLoops)
        .finallyDo(
            () -> {
              drive.stop();
              superstructure.requestState(State.IDLE);
            });
  }

  /**
   * Drives a continuous 360° holonomic circle of radius {@code radiusMeters} while locking robot
   * heading onto the 2026 Blue Hub with {@link ShotCalculator} Shoot-on-the-Move lead compensation
   * and firing {@code FUEL} balls.
   */
  public static Command createShootOnTheMoveCircleCommand(
      Drive drive,
      Superstructure superstructure,
      ShotCalculator shotCalculator,
      double radiusMeters,
      double durationSeconds) {
    final int maxLoops = Math.max(1, (int) Math.round(durationSeconds / 0.020));
    final int[] loops = {0};
    final double omegaOrbit = (2.0 * Math.PI) / Math.max(0.5, durationSeconds);
    final double tangentialSpeed = radiusMeters * omegaOrbit;

    return Commands.run(
            () -> {
              double elapsedSec = loops[0] * 0.020;
              double angle = omegaOrbit * elapsedSec;
              double vxField = tangentialSpeed * Math.cos(angle);
              double vyField = -tangentialSpeed * Math.sin(angle);

              Pose2d pose = drive.getPose();
              ShotSolution sol =
                  shotCalculator.calculate(
                      pose,
                      new ChassisSpeeds(vxField, vyField, 0.0),
                      FieldPositions.forAlliance(RobotContainer.HUB_TARGET_METERS));
              double yawErrRad =
                  MathUtil.angleModulus(
                      sol.driveAimHeading().minus(pose.getRotation()).getRadians());
              boolean inRange = shotCalculator.isInRange(sol.effectiveDistanceMeters());
              superstructure.applyShotSolution(sol, true, Math.abs(yawErrRad), inRange);

              double omegaCmd =
                  MathUtil.clamp(
                      yawErrRad * 6.5 + sol.headingFeedforwardRadPerSec(), -7.5, 7.5);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      vxField, vyField, omegaCmd, pose.getRotation()));
              loops[0]++;
            },
            drive,
            superstructure)
        .beforeStarting(() -> loops[0] = 0)
        .until(() -> loops[0] >= maxLoops)
        .finallyDo(
            () -> {
              drive.stop();
              superstructure.requestState(State.IDLE);
            });
  }

  /**
   * Wraps {@code command} with a deterministic 50 Hz (20 ms) loop-count timeout via {@link
   * SafeAutoBuilder#withLoopTimeout(Command, double)}.
   */
  public static Command withLoopTimeout(Command command, double timeoutSeconds) {
    return SafeAutoBuilder.withLoopTimeout(command, timeoutSeconds);
  }
}
