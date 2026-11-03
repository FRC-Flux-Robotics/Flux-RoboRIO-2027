package frc.simharness;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.CoordinateSystem;
import frc.lib.util.FieldPositions.FieldProfile;
import frc.lib.util.SafeAutoBuilder;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * JUnit test-scope autonomous simulation harness that steps {@link RobotContainer} through a
 * 15.0-second (750-loop at 20 ms) 2026 REBUILT field autonomous routine and exports {@code
 * student-auto-frames.json} for the standalone {@code FluxProcessor} video renderer ({@code java
 * -jar flux-processor.jar --render-auto}).
 *
 * <p>Kept strictly in {@code src/test/java/} so zero simulation harness or video-rendering code is
 * compiled into {@code frcUserProgram.jar} on the robot.
 */
public final class StudentAutoSimExporter {

  /** 2026 Blue Alliance Hexagonal Hub top height in meters. */
  public static final double HUB_TOP_Z = 1.83;

  /** Preloaded {@code FUEL} game pieces inside the robot hopper at the start of autonomous. */
  public static final int PRELOAD_FUEL_BALLS = 3;

  /** Maximum {@code FUEL} game pieces held simultaneously in the hopper + indexer. */
  public static final int MAX_HOPPER_CAPACITY = 8;

  /**
   * Default floor {@code FUEL} game piece locations across the 2026 REBUILT Blue Half-Field: 3 in
   * the Blue Depot strip {@code (1.02m, 6.40m) -> (0.77m, 5.44m)}, 2 on the South Wing, 2 on the
   * North Wing, and 3 along the Centerline ({@code Collect.path} at {@code X = 7.68m}).
   */
  private static final double[][] INITIAL_WING_BALLS = {
    {1.02, 6.38},
    {0.78, 6.02},
    {0.78, 5.50},
    {2.80, 3.25},
    {2.55, 2.55},
    {2.65, 5.35},
    {2.55, 6.15},
    {7.68, 1.50},
    {7.68, 2.45},
    {7.68, 3.35}
  };

  /** Single 25 Hz (40 ms) snapshot of the simulated robot during an autonomous run. */
  public record AutoFrame(
      double timeSec,
      double xMeters,
      double yMeters,
      double headingRad,
      double vxMps,
      double vyMps,
      double omegaRadPerSec,
      double elevatorHeightM,
      double armAngleRad,
      double intakeRollerRps,
      double batteryVoltage,
      double driveStatorAmps,
      double accelScale,
      String stateLabel) {}

  /** 3D parabolic ballistic trajectory of a fired 2026 {@code FUEL} game piece. */
  public record ShotArc(
      int shotId,
      double launchTimeSec,
      double flightTimeSec,
      double startX,
      double startY,
      double startZ,
      double targetX,
      double targetY,
      double targetZ,
      double apexZ,
      boolean scored) {}

  /** Floor {@code FUEL} game piece that disappears once picked up at {@code pickedUpTimeSec}. */
  public record FloorFuelBall(double xMeters, double yMeters, double pickedUpTimeSec) {}

  private StudentAutoSimExporter() {}

  /**
   * Steps {@code container} through {@code autoCommand} for 15.0 seconds (750 loops at 20 ms) on
   * the 2026 REBUILT field and writes {@code student-auto-frames.json} to {@code outputDir}.
   */
  public static List<AutoFrame> simulateAndExportJson(
      RobotContainer container, Command autoCommand, String routineName, Path outputDir)
      throws IOException {
    Drive drive = container.getDrive();
    Superstructure superstructure = container.getSuperstructure();
    ShotCalculator shotCalculator = container.getShotCalculator();

    FieldProfile previousProfile = FieldPositions.getFieldProfile();
    CoordinateSystem previousCoord = FieldPositions.getCoordinateSystem();
    FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);
    FieldPositions.setCoordinateSystem(CoordinateSystem.WALL_BLUE_2026);
    SafeAutoBuilder.clearLastDiagnostic();
    try {
      drive.setPose(new Pose2d(3.10, 4.05, Rotation2d.kZero));
      superstructure.requestState(State.IDLE);

      double[] pickupTimes = new double[INITIAL_WING_BALLS.length];
      for (int i = 0; i < pickupTimes.length; i++) {
        pickupTimes[i] = 999.0;
      }

      List<AutoFrame> frames = new ArrayList<>(380);
      List<ShotArc> shots = new ArrayList<>();
      double lastShotTimeSec = -1.0;
      int shotCounter = 0;
      int hopperBalls = PRELOAD_FUEL_BALLS;

      autoCommand.initialize();

      int totalLoops = 750; // 15.0 seconds @ 50 Hz (20 ms)
      for (int step = 0; step <= totalLoops; step++) {
        double t = step * 0.020;
        if (!autoCommand.isFinished()) {
          autoCommand.execute();
        }
        drive.periodic();
        container.getIntake().periodic();
        container.getTilter().periodic();
        container.getIndexer().periodic();
        container.getFeeder().periodic();
        container.getShooter().periodic();
        container.getHood().periodic();
        superstructure.periodic();
        container.periodicUpdate();

        Pose2d p = drive.getPose();
        ChassisSpeeds cs =
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), p.getRotation());

        if (superstructure.getState() == State.INTAKING
            || Math.abs(container.getIntake().getVelocity()) > 10.0) {
          for (int b = 0; b < INITIAL_WING_BALLS.length; b++) {
            if (pickupTimes[b] > 900.0 && hopperBalls < MAX_HOPPER_CAPACITY) {
              double dist =
                  Math.hypot(
                      p.getX() - INITIAL_WING_BALLS[b][0], p.getY() - INITIAL_WING_BALLS[b][1]);
              if (dist <= 0.72) {
                pickupTimes[b] = t;
                hopperBalls++;
              }
            }
          }
        }

        if (superstructure.getState() == State.SHOOTING
            && superstructure.isReadyToFire()
            && hopperBalls > 0
            && (t - lastShotTimeSec) >= 0.42) {
          Translation2d hub = FieldPositions.forAlliance(RobotContainer.HUB_TARGET_METERS);
          ShotSolution sol = shotCalculator.calculate(p, cs, hub);
          double aimErrDeg =
              Math.abs(sol.driveAimHeading().minus(p.getRotation()).getDegrees());
          boolean scored = aimErrDeg <= 16.0 && sol.effectiveDistanceMeters() <= 6.2;
          double tof = Math.max(0.28, Math.min(0.68, sol.timeOfFlightSeconds()));
          double landX =
              scored
                  ? hub.getX()
                  : p.getX() + sol.effectiveDistanceMeters() * p.getRotation().getCos();
          double landY =
              scored
                  ? hub.getY()
                  : p.getY() + sol.effectiveDistanceMeters() * p.getRotation().getSin();
          double apexZ = HUB_TOP_Z + 0.42 + 0.08 * sol.effectiveDistanceMeters();

          hopperBalls--;
          shotCounter++;
          shots.add(
              new ShotArc(
                  shotCounter,
                  t,
                  tof,
                  p.getX(),
                  p.getY(),
                  0.48,
                  landX,
                  landY,
                  HUB_TOP_Z,
                  apexZ,
                  scored));
          lastShotTimeSec = t;
        }

        if (step % 2 == 0) {
          frames.add(
              new AutoFrame(
                  t,
                  p.getX(),
                  p.getY(),
                  p.getRotation().getRadians(),
                  cs.vxMetersPerSecond,
                  cs.vyMetersPerSecond,
                  cs.omegaRadiansPerSecond,
                  container.getHood().getPosition(),
                  container.getTilter().getPosition(),
                  container.getIntake().getVelocity(),
                  container.getVirtualBatterySim().terminalVoltageV(),
                  drive.getTotalSupplyCurrentAmps()
                      + superstructure.getTotalMechanismSupplyCurrentAmps(),
                  container.getShooter().getVelocity(),
                  superstructure.getState().name()));
        }
      }
      autoCommand.end(false);

      List<FloorFuelBall> floorBalls = new ArrayList<>(INITIAL_WING_BALLS.length);
      for (int i = 0; i < INITIAL_WING_BALLS.length; i++) {
        floorBalls.add(
            new FloorFuelBall(INITIAL_WING_BALLS[i][0], INITIAL_WING_BALLS[i][1], pickupTimes[i]));
      }

      String autoWarning = SafeAutoBuilder.getLastCancelReason();
      Files.createDirectories(outputDir);
      writeFramesJson(
          outputDir.resolve("student-auto-frames.json"),
          routineName,
          container.getIdentity().name(),
          frames,
          shots,
          floorBalls,
          autoWarning);
      return frames;
    } finally {
      FieldPositions.setFieldProfile(previousProfile);
      FieldPositions.setCoordinateSystem(previousCoord);
    }
  }

  private static void writeFramesJson(
      Path jsonFile,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      String autoWarning)
      throws IOException {
    StringBuilder sb = new StringBuilder(32768);
    sb.append("{\n");
    sb.append("  \"routineName\": \"").append(escapeJson(routineName)).append("\",\n");
    sb.append("  \"robotName\": \"").append(escapeJson(robotName)).append("\",\n");
    sb.append("  \"fieldMode\": \"REBUILT_2026_SHOOTING\",\n");
    if (autoWarning != null && !autoWarning.isBlank()) {
      sb.append("  \"autoWarning\": \"").append(escapeJson(autoWarning)).append("\",\n");
    }
    sb.append("  \"fps\": 25,\n");
    sb.append("  \"floorBalls\": [\n");
    for (int i = 0; i < floorBalls.size(); i++) {
      FloorFuelBall b = floorBalls.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"x\":%.3f,\"y\":%.3f,\"tPick\":%.3f}",
              b.xMeters(),
              b.yMeters(),
              b.pickedUpTimeSec()));
      if (i + 1 < floorBalls.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ],\n");
    sb.append("  \"shots\": [\n");
    for (int i = 0; i < shots.size(); i++) {
      ShotArc s = shots.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"id\":%d,\"t0\":%.3f,\"tof\":%.3f,\"x0\":%.3f,\"y0\":%.3f,\"z0\":%.3f,\"x1\":%.3f,\"y1\":%.3f,\"z1\":%.3f,\"apexZ\":%.3f,\"scored\":%s}",
              s.shotId(),
              s.launchTimeSec(),
              s.flightTimeSec(),
              s.startX(),
              s.startY(),
              s.startZ(),
              s.targetX(),
              s.targetY(),
              s.targetZ(),
              s.apexZ(),
              s.scored() ? "true" : "false"));
      if (i + 1 < shots.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ],\n");
    sb.append("  \"frames\": [\n");
    for (int i = 0; i < frames.size(); i++) {
      AutoFrame f = frames.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"t\":%.3f,\"x\":%.4f,\"y\":%.4f,\"h\":%.4f,\"vx\":%.3f,\"vy\":%.3f,\"om\":%.3f,\"elev\":%.3f,\"arm\":%.3f,\"rol\":%.2f,\"vBat\":%.2f,\"iDrv\":%.1f,\"scale\":%.2f,\"st\":\"%s\"}",
              f.timeSec(),
              f.xMeters(),
              f.yMeters(),
              f.headingRad(),
              f.vxMps(),
              f.vyMps(),
              f.omegaRadPerSec(),
              f.elevatorHeightM(),
              f.armAngleRad(),
              f.intakeRollerRps(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              f.accelScale(),
              escapeJson(f.stateLabel())));
      if (i + 1 < frames.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ]\n}\n");
    Files.writeString(jsonFile, sb.toString(), StandardCharsets.UTF_8);
  }

  private static String escapeJson(String s) {
    return (s == null) ? "" : s.replace("\\", "\\\\").replace("\"", "\\\"");
  }
}
