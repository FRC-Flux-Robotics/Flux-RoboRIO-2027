package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.Blitz15MinCmd;
import frc.lib.commands.DriveCommands;
import frc.lib.commands.DriveToPose;
import frc.lib.commands.HardStopHomingCmd;
import frc.lib.commands.PitSystemCheckCmd;
import frc.lib.commands.PitSystemCheckCmd.PositionCheckTarget;
import frc.lib.commands.PitSystemCheckCmd.VelocityCheckTarget;
import frc.lib.commands.PreMatchCalibrator;
import frc.lib.commands.ShopStressSuiteCmd;
import frc.lib.commands.ZeroPriorBootstrapCmd;
import frc.lib.drive.Drive;
import frc.lib.drive.GyroIO;
import frc.lib.drive.Module;
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIOSim;
import frc.lib.drive.PhoenixOdometryThread;
import frc.lib.drive.SwerveConfig;
import frc.lib.drive.VirtualGeofence;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIOSim;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.BatteryIO;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.CoordinateSystem;
import frc.lib.util.FieldPositions.FieldProfile;
import frc.lib.util.LogFileManager;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PhoenixUtil;
import frc.lib.util.PiecewiseSensitivity;
import frc.lib.util.SafeAutoBuilder;
import frc.lib.util.VirtualBatterySim;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIO.ObservationType;
import frc.lib.vision.VisionIO.PoseObservation;
import frc.lib.vision.VisionIOPhotonVision;
import frc.lib.vision.VisionIOSim;
import frc.lib.vision.ZeroPriorBootstrapSolver;
import frc.lib.vision.ZeroPriorBootstrapSolver.PivotObservationPair;
import frc.lib.vision.ZeroPriorBootstrapSolver.SolvedCameraExtrinsics;
import frc.lib.vision.ZeroPriorBootstrapSolver.TranslationObservationPair;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.autos.StudentAutoRoutines;
import frc.robot.commands.ShotCalibrationCmd;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Tier 2 Verification Gate: Deterministic math, FluxFig 1-cell SystemCore/roboRIO fallback,
 * FieldPositions bijection & profiles, 10,000-vector VirtualGeofence braking, AX=XB camera solver,
 * 2026 FUEL Superstructure state machine & shot interlocks, Task 1.4 utilities
 * (LoggedTunableNumber, PiecewiseSensitivity, VirtualBatterySim, SafeAutoBuilder, ShotCalculator),
 * and Task 2.1 subsystems (250 Hz Drive + ModuleIOSim, full-queue Vision + VisionIOSim,
 * MechanismIOSim, and Ghost Superstructure hybrid mode on CORAL/CHASSIS_3).
 */
public class Tier2DeterministicSolversTest {

  @BeforeAll
  public static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testFluxFigControllerFallbackAndSha256() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    assertEquals(8, fig.getConfigSha256().length());

    // Scenario 1: COMP_2027 on SystemCore (locked at 50 Hz / 20 ms until 5 real SC runs prove 200 Hz)
    fig.setActiveProfile("COMP_2027");
    assertEquals("SYSTEMCORE_2027", fig.getString("controllerProfile"));
    assertEquals("can_s0", fig.getString("swerveCanBus"));
    assertEquals(20, fig.getInt("loopPeriodMs"));

    // Scenario 2: 1-Cell Fallback to roboRIO 2.0 on COMP_2027
    fig.setActiveProfile("COMP_2027:roborio_fallback");
    assertEquals("ROBORIO_CANIVORE_2026", fig.getString("controllerProfile"));
    assertEquals("Drivetrain", fig.getString("swerveCanBus"));
    assertEquals("PHOTONVISION_NT4", fig.getString("visionBackend"));
    assertEquals(20, fig.getInt("loopPeriodMs"));
    assertEquals(60.0, fig.getDouble("driveStatorLimitAmps"), 1e-6);

    for (String profile : List.of("DEFAULT", "PRACTICE_2027", "FUEL", "CORAL", "CHASSIS_3")) {
      fig.setActiveProfile(profile);
      assertEquals(20, fig.getInt("loopPeriodMs"), "loopPeriodMs must be 20 ms for " + profile);
    }
  }

  @Test
  public void testFieldPositionsCoordinateBijectionAndProfiles() {
    Random rng = new Random(10413L);
    double maxRoundTripErrMm = 0.0;
    for (int i = 0; i < 1000; i++) {
      Pose2d orig =
          new Pose2d(
              rng.nextDouble() * FieldPositions.FIELD_LENGTH_METERS,
              rng.nextDouble() * FieldPositions.FIELD_WIDTH_METERS,
              Rotation2d.fromDegrees(rng.nextDouble() * 360.0 - 180.0));
      Pose2d centerRed = FieldPositions.wallBlueToCenterRed(orig);
      Pose2d roundTrip = FieldPositions.centerRedToWallBlue(centerRed);
      double errMm = orig.getTranslation().getDistance(roundTrip.getTranslation()) * 1000.0;
      maxRoundTripErrMm = Math.max(maxRoundTripErrMm, errMm);
    }

    // Verify stateful CoordinateSystem and FieldProfile
    FieldPositions.setCoordinateSystem(CoordinateSystem.WALL_BLUE_2026);
    FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);
    FieldPositions.setAllianceOverride(false);
    assertTrue(FieldPositions.isInsideField(4.0, 4.0, 0.25));
    assertEquals(0.0, FieldPositions.operatorForward().getRadians(), 1e-6);
    FieldPositions.setAllianceOverride(null);

    FieldPositions.setFieldProfile(FieldProfile.SHOP_5X3);
    assertTrue(FieldPositions.isInsideField(1.0, 0.5, 0.1));
    assertFalse(FieldPositions.isInsideField(6.5, 0.5, 0.0));

    // Verify dynamic auto-measured non-standard room bounds (e.g. 4.4m x 2.7m asymmetric garage)
    FieldPositions.setCustomArenaBounds(-2.10, 2.30, -1.30, 1.40);
    assertEquals(4.40, FieldPositions.getShopLengthMeters(), 1e-6);
    assertEquals(2.70, FieldPositions.getShopWidthMeters(), 1e-6);
    assertTrue(FieldPositions.isInsideField(2.25, 1.35, 0.0));
    assertFalse(FieldPositions.isInsideField(2.40, 1.35, 0.0));
    ChassisSpeeds customClamped =
        VirtualGeofence.clampFieldSpeeds(
            2.30 - VirtualGeofence.TOTAL_CLEARANCE_METERS,
            0.0,
            new ChassisSpeeds(2.0, 0.0, 0.0),
            true,
            0.020);
    assertEquals(0.0, customClamped.vxMetersPerSecond, 1e-6, "Must brake to 0 at custom +X wall");
    FieldPositions.resetShopDimensions();

    // Verify HALF_FIELD_BLUE midfield centerline braking in 2026 Wall/Blue (midfield at X = L/2, safe maxX = L/2 - 0.85m)
    ChassisSpeeds halfBlueClamped =
        VirtualGeofence.clampProfileSpeeds(
            FieldPositions.HALF_FIELD_LENGTH_METERS - VirtualGeofence.TOTAL_CLEARANCE_METERS,
            4.0,
            new ChassisSpeeds(2.0, 0.0, 0.0),
            true,
            0.020,
            CoordinateSystem.WALL_BLUE_2026,
            FieldProfile.HALF_FIELD_BLUE);
    assertEquals(0.0, halfBlueClamped.vxMetersPerSecond, 1e-6, "Must brake before crossing midfield on HALF_FIELD_BLUE");
    FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_field_coord_bijection_mm",
        "2026 <-> 2027 Coord Bijection Error",
        maxRoundTripErrMm,
        0.0,
        0.01,
        "mm");
    assertTrue(maxRoundTripErrMm < 0.01);
  }

  @Test
  public void testVirtualGeofenceTenThousandVectorsZeroBreaches() {
    Random rng = new Random(202710413L);
    int breaches = 0;
    double dt = 0.005;
    for (int i = 0; i < 10000; i++) {
      double x = (rng.nextDouble() * 2.0 - 1.0) * VirtualGeofence.SAFE_HALF_X_METERS;
      double y = (rng.nextDouble() * 2.0 - 1.0) * VirtualGeofence.SAFE_HALF_Y_METERS;
      double vxCmd = (rng.nextDouble() * 2.0 - 1.0) * 4.8;
      double vyCmd = (rng.nextDouble() * 2.0 - 1.0) * 4.8;
      ChassisSpeeds safe =
          VirtualGeofence.clampFieldSpeeds(
              x, y, new ChassisSpeeds(vxCmd, vyCmd, 3.0), true, 0.020);
      double nextX = x + safe.vxMetersPerSecond * dt;
      double nextY = y + safe.vyMetersPerSecond * dt;
      if (Math.abs(nextX) > VirtualGeofence.SAFE_HALF_X_METERS + 1e-4
          || Math.abs(nextY) > VirtualGeofence.SAFE_HALF_Y_METERS + 1e-4) {
        breaches++;
      }
    }
    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_geofence_breaches_10k",
        "VirtualGeofence 10k-Vector Breaches",
        breaches,
        0.0,
        0.0,
        "breaches");
    assertEquals(0, breaches, "VirtualGeofence must prevent all wall breaches across 10k vectors");
  }

  @Test
  public void testZeroPriorBootstrapHandEyeSolverWithGaussianNoise() {
    Random rng = new Random(4210413L);
    double trueCamX = 0.279;
    double trueCamY = -0.185;
    double trueCamZ = 0.508;
    double trueCamYawRad = Math.toRadians(24.5);
    Rotation2d trueCamRot = new Rotation2d(trueCamYawRad);
    Translation2d trueCamPos = new Translation2d(trueCamX, trueCamY);
    Translation2d wallTagField = new Translation2d(2.45, 0.35);

    List<TranslationObservationPair> transPairs = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      Translation2d deltaRobot = new Translation2d(0.70, (i % 2 == 0) ? 0.65 : -0.65);
      Translation2d noise =
          new Translation2d(rng.nextGaussian() * 0.008, rng.nextGaussian() * 0.008);
      Translation2d deltaTagInCam =
          deltaRobot.times(-1.0).rotateBy(trueCamRot.unaryMinus()).plus(noise);
      transPairs.add(new TranslationObservationPair(deltaRobot, deltaTagInCam));
    }

    List<PivotObservationPair> pivotPairs = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      Rotation2d dTheta = Rotation2d.fromDegrees((i % 2 == 0) ? 28.0 : -28.0);
      Translation2d tagInRobot0 = wallTagField.minus(trueCamPos);
      Translation2d tagInCam0 =
          tagInRobot0
              .rotateBy(trueCamRot.unaryMinus())
              .plus(new Translation2d(rng.nextGaussian() * 0.006, rng.nextGaussian() * 0.006));

      Translation2d tagInRobot1 = wallTagField.rotateBy(dTheta.unaryMinus()).minus(trueCamPos);
      Translation2d tagInCam1 =
          tagInRobot1
              .rotateBy(trueCamRot.unaryMinus())
              .plus(new Translation2d(rng.nextGaussian() * 0.006, rng.nextGaussian() * 0.006));

      pivotPairs.add(new PivotObservationPair(dTheta, tagInCam0, tagInCam1));
    }

    SolvedCameraExtrinsics solved =
        ZeroPriorBootstrapSolver.solveExtrinsics(transPairs, pivotPairs, trueCamZ);
    double posErrMm =
        Math.hypot(solved.xMeters() - trueCamX, solved.yMeters() - trueCamY) * 1000.0;
    double yawErrDeg = Math.abs(Math.toDegrees(solved.yawRad() - trueCamYawRad));

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_axxb_pos_error_mm",
        "AX=XB Camera Lever-Arm Error",
        posErrMm,
        0.5,
        5.0,
        "mm");
    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_axxb_yaw_error_deg",
        "AX=XB Camera Yaw Solver Error",
        yawErrDeg,
        0.05,
        0.50,
        "deg");

    assertTrue(posErrMm < 5.0, "Camera (X,Y) error must be < 5.0 mm under Gaussian tag noise");
    assertTrue(yawErrDeg < 0.50, "Camera yaw error must be < 0.50 deg under Gaussian tag noise");

    SolvedCameraExtrinsics degenerate =
        ZeroPriorBootstrapSolver.solveExtrinsics(List.of(), List.of(), trueCamZ);
    assertFalse(
        Double.isNaN(degenerate.xMeters()) || Double.isNaN(degenerate.yMeters()),
        "Degenerate empty pivot list must return finite 0.0 extrinsics instead of NaN");
  }

  @Test
  public void testFuelSuperstructureInterlocksAndStates() {
    RobotContainer fuelContainer =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    assertFalse(fuelContainer.isGhostSuperstructureActive());
    assertEquals(
        RobotContainer.OperatingMode.COMPETITION,
        fuelContainer.getOperatingMode(),
        "Default OperatingMode must be COMPETITION (FluxFig.Mode.PRODUCTION)");
    fuelContainer.getFluxFig().setMode(FluxFig.Mode.TUNING);
    assertEquals(
        RobotContainer.OperatingMode.TRAINING,
        fuelContainer.getOperatingMode(),
        "OperatingMode must switch to TRAINING when FluxFig is in TUNING mode off FMS");
    fuelContainer.getFluxFig().lockProductionOnFms(true);
    assertEquals(
        RobotContainer.OperatingMode.COMPETITION,
        fuelContainer.getOperatingMode(),
        "FMS lock must force OperatingMode back to COMPETITION");
    fuelContainer.getFluxFig().lockProductionOnFms(false);
    Superstructure ss = fuelContainer.getSuperstructure();
    assertEquals(State.IDLE, ss.getState());

    ss.requestState(State.INTAKING);
    assertEquals(State.INTAKING, ss.getState());
    assertEquals(Superstructure.DEFAULT_TILTER_DEPLOY_ROT, ss.getTilter().getTargetPosition(), 1e-4);
    assertEquals(Superstructure.DEFAULT_INTAKE_IN_RPS, ss.getIntake().getTargetVelocity(), 1e-4);

    ss.shoot(Superstructure.DEFAULT_MID_SHOOTER_RPS, Superstructure.DEFAULT_MID_HOOD_ROT);
    assertFalse(ss.isReadyToFire());
    assertEquals(0.0, ss.getFeeder().getTargetVelocity(), 1e-4);

    for (int i = 0; i < 80; i++) {
      ss.getShooter().periodic();
      ss.getHood().periodic();
      ss.getIndexer().periodic();
      ss.getFeeder().periodic();
      ss.periodic();
    }
    assertTrue(
        ss.isReadyToFire(),
        "Shooter and Hood must reach target and unlock firing gate (shooter="
            + ss.getShooter().getVelocity()
            + "/"
            + ss.getShooter().getTargetVelocity()
            + ", hood="
            + ss.getHood().getPosition()
            + "/"
            + ss.getHood().getTargetPosition()
            + ")");
    assertEquals(Superstructure.DEFAULT_FEEDER_RPS, ss.getFeeder().getTargetVelocity(), 1e-4);

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_elevator_keepout_violations",
        "Superstructure Interlock Violations",
        0.0,
        0.0,
        0.0,
        "hits");
  }

  @Test
  public void testTask14UtilitiesShotCalculatorTunableSensitivityAndBattery() throws Exception {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    fig.setActiveProfile("FUEL");

    LoggedTunableNumber.setTuningMode(true);
    LoggedTunableNumber tunable = new LoggedTunableNumber("Test/GainKp", 1.25);
    assertEquals(1.25, tunable.get(), 1e-6);
    assertFalse(tunable.hasChanged(99), "Tick 1 must not trigger redundant CAN write when unchanged");
    assertFalse(tunable.hasChanged(99));
    tunable.setTestOverride(2.50);
    assertEquals(2.50, tunable.get(), 1e-6);
    assertTrue(tunable.hasChanged(99));
    LoggedTunableNumber.setTuningMode(false);
    assertEquals(1.25, tunable.get(), 1e-6);
    assertTrue(tunable.hasChanged(99), "Switching back to production mode must revert tuned gain");
    assertFalse(tunable.hasChanged(99));

    PiecewiseSensitivity curve =
        new PiecewiseSensitivity("Test/Joystick", 0.08, 0.50, 0.05, 0.45, 1.0);
    assertEquals(0.0, curve.transfer(0.05), 1e-6); // inside deadzone
    assertEquals(1.0, curve.transfer(1.0), 1e-5); // full stick reaches 1.0
    assertEquals(-1.0, curve.transfer(-1.0), 1e-5);

    ShotCalculator shotCalc = ShotCalculator.fromFluxFig(fig);
    Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    Translation2d target = new Translation2d(4.0, 0.0);
    ShotSolution stationaryShot =
        shotCalc.calculate(robotPose, new ChassisSpeeds(0.0, 0.0, 0.0), target);
    ShotSolution movingLaterallyShot =
        shotCalc.calculate(robotPose, new ChassisSpeeds(0.0, 2.5, 0.0), target);
    assertTrue(
        stationaryShot.shooterRps() >= 33.0 && stationaryShot.shooterRps() <= 40.0,
        "Proven NYTR FUEL shooter RPS at 4.0m must stay within [33.0, 40.0] RPS (got "
            + stationaryShot.shooterRps()
            + ")");
    assertTrue(movingLaterallyShot.driveAimHeading().getRadians() < -0.05);
    assertTrue(
        movingLaterallyShot.effectiveDistanceMeters() > stationaryShot.effectiveDistanceMeters());

    VirtualBatterySim batSim = new VirtualBatterySim(12.5, 0.022);
    for (int i = 0; i < 40; i++) {
      double currentA = (i % 2 == 0) ? 25.0 : 140.0;
      batSim.update(currentA, 0.0);
    }
    double rIntErrMohm = Math.abs(batSim.estimatedRintMilliOhms() - 22.0);
    assertTrue(rIntErrMohm < 0.5, "Online R_int estimator must converge within 0.5 mOhm");

    List<Pose2d> validWaypoints =
        List.of(
            new Pose2d(2.0, 2.0, Rotation2d.kZero),
            new Pose2d(3.0, 2.5, Rotation2d.kZero),
            new Pose2d(4.0, 2.5, Rotation2d.kZero));
    assertTrue(
        SafeAutoBuilder.auditPath(
                validWaypoints,
                new Pose2d(2.0, 2.0, Rotation2d.kZero),
                3.5,
                SafeAutoBuilder.Limits.DEFAULT,
                CoordinateSystem.WALL_BLUE_2026,
                FieldProfile.FULL_FIELD)
            .passed());
    List<Pose2d> outOfBoundsWaypoints =
        List.of(new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(25.0, 2.0, Rotation2d.kZero));
    assertFalse(
        SafeAutoBuilder.auditPath(
                outOfBoundsWaypoints,
                new Pose2d(2.0, 2.0, Rotation2d.kZero),
                3.5,
                SafeAutoBuilder.Limits.DEFAULT,
                CoordinateSystem.WALL_BLUE_2026,
                FieldProfile.FULL_FIELD)
            .passed());

    Path logDir = Path.of("build/test-logs");
    Path metaFile =
        LogFileManager.writeRunMetaJson(
            logDir, "FUEL", "FUEL", fig.getConfigSha256(), "2026-B01", 20);
    assertTrue(Files.exists(metaFile));

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_battery_rint_est_err_mohm",
        "VirtualBatterySim R_int Est Error",
        rIntErrMohm,
        0.0,
        0.50,
        "mOhm");
  }

  @Test
  public void testTask21Drive250HzVisionFullQueueAndGhostSuperstructure() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    fig.setActiveProfile("FUEL");
    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fig, "FUEL");
    FaultSupervisor faultSupervisor = new FaultSupervisor(3);

    PhoenixOdometryThread.getInstance().drainTimestamps();
    ModuleIO[] moduleIOs =
        new ModuleIO[] {
          new ModuleIOSim(swerveConfig),
          new ModuleIOSim(swerveConfig),
          new ModuleIOSim(swerveConfig),
          new ModuleIOSim(swerveConfig)
        };
    GyroIO simGyro =
        new GyroIO() {
          private Rotation2d yaw = Rotation2d.kZero;

          @Override
          public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.yawPosition = yaw;
            inputs.odometryYawTimestamps =
                new double[] {0.004, 0.008, 0.012, 0.016, 0.020};
            inputs.odometryYawPositions =
                new Rotation2d[] {yaw, yaw, yaw, yaw, yaw};
          }

          @Override
          public void setYaw(Rotation2d newYaw) {
            this.yaw = newYaw;
          }
        };

    Drive drive = new Drive(swerveConfig, simGyro, moduleIOs);
    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    drive.setMaxAccelerationScale(1.0);

    for (int i = 0; i < 25; i++) {
      drive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0));
      drive.periodic();
    }
    assertTrue(
        drive.getPose().getX() > 0.65,
        "250 Hz Drive + ModuleIOSim must integrate forward odometry > 0.65m");

    CameraSpec[] camConfigs = fig.getCamerasForRobot("FUEL").toArray(new CameraSpec[0]);
    assertEquals(3, camConfigs.length);
    VisionIOSim[] visionSims = new VisionIOSim[3];
    for (int i = 0; i < 3; i++) {
      visionSims[i] = new VisionIOSim(camConfigs[i], drive::getPose);
      visionSims[i].setFramesPerCycle(3);
    }
    Vision vision = new Vision(drive::addVisionMeasurement, faultSupervisor, camConfigs, visionSims);

    vision.periodic();
    assertEquals(9, vision.getTotalFramesDrained());
    assertEquals(9, vision.getAcceptedObservationCount());

    visionSims[1].setMountErrorPerturbation(
        new Transform3d(new Translation3d(0.15, 0.0, 0.0), Rotation3d.kZero));
    for (int loop = 0; loop < 6; loop++) {
      vision.periodic();
    }
    assertTrue(
        faultSupervisor.isCameraQuarantined(1),
        "Vision + FaultSupervisor must quarantine camera #1 after +15cm mount error");

    MechanismSpec shooterSpec = fig.getMechanismSpec("FUEL", "Shooter");
    MechanismSpec hoodSpec = fig.getMechanismSpec("FUEL", "Hood");
    MechanismSpec tilterSpec = fig.getMechanismSpec("FUEL", "Tilter");
    VelocityMechanism flywheel =
        new VelocityMechanism(new MechanismIOSim(shooterSpec), shooterSpec);
    PositionMechanism hood = new PositionMechanism(new MechanismIOSim(hoodSpec), hoodSpec);
    PositionMechanism tilter = new PositionMechanism(new MechanismIOSim(tilterSpec), tilterSpec);

    flywheel.setVelocity(60.0);
    hood.setPosition(3.5);
    tilter.setPosition(-10.0);
    for (int i = 0; i < 80; i++) {
      flywheel.periodic();
      hood.periodic();
      tilter.periodic();
    }
    assertEquals(60.0, flywheel.getVelocity(), 3.0);
    assertEquals(3.5, hood.getPosition(), 0.4);
    assertEquals(-10.0, tilter.getPosition(), 1.5);

    RobotContainer coralContainer =
        new RobotContainer(RobotIdentity.CORAL, Paths.get("src/main/deploy/configs"), true);
    assertTrue(coralContainer.isGhostSuperstructureActive());
    coralContainer.updateShootOnTheMove(true);
    for (int i = 0; i < 20; i++) {
      coralContainer.getShooter().periodic();
      coralContainer.getHood().periodic();
      coralContainer.getSuperstructure().periodic();
      coralContainer.periodicUpdate();
    }
    assertNotNull(coralContainer.getDrive());
    assertNotNull(coralContainer.getVision());
  }

  @Test
  public void testSharedFrcLibCommandsBatteryIoAndFuelShotCalibrationCmd() throws Exception {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    RobotContainer fuelContainer =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    Drive drive = fuelContainer.getDrive();
    Vision vision = fuelContainer.getVision();

    BatteryIO batIo = new BatteryIO(fig, "rio");
    assertEquals("UNKNOWN_BATTERY", batIo.getResolvedBatteryName());
    assertFalse(batIo.isHealthy());

    int nick1ArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_NICKNAME_1, 0);
    assertEquals(0x0A0B0F00, nick1ArbId);
    assertTrue(
        batIo.processCanFrame(nick1ArbId, BatteryIO.encodeNickname1Frame("2026-B02"), 8));
    assertEquals("2026-B02", batIo.getResolvedBatteryName());

    int powerArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_POWER, 0);
    assertTrue(
        batIo.processCanFrame(powerArbId, BatteryIO.encodePowerFrame(18.5, 12.640, 420.0), 8));
    double bfgDecodeErrMv = Math.abs(batIo.getVoltageVolts() - 12.640) * 1000.0;
    assertEquals(12.640, batIo.getVoltageVolts(), 1e-4);
    assertEquals(18.5, batIo.getCurrentAmps(), 1e-3);

    int socArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_STATE_OF_CHARGE, 0);
    assertTrue(
        batIo.processCanFrame(
            socArbId, BatteryIO.encodeStateOfChargeFrame(17370.0, 18000.0, 2, 224.0), 8));
    assertEquals(96.5, batIo.getStateOfChargePct(), 1e-3);
    assertEquals(15.8, batIo.getInternalResistanceMilliOhms(), 1e-3);
    assertTrue(batIo.isHealthy());

    assertTrue(
        batIo.processCanFrame(powerArbId, BatteryIO.encodePowerFrame(15.0, 12.500, 425.0), 8));
    assertTrue(
        batIo.processCanFrame(powerArbId, BatteryIO.encodePowerFrame(115.0, 10.850, 440.0), 8));
    assertEquals(16.5, batIo.getInternalResistanceMilliOhms(), 0.2);

    batIo.simulateCanReading("FLUX-BAT-2026-B01", 12.65, 98.0, 96.0, 15.5);
    assertEquals("2026-B01", batIo.getResolvedBatteryName());
    assertTrue(batIo.isHealthy());
    assertThrows(
        IllegalArgumentException.class,
        () -> new BatteryIO(fig, "Drivetrain"),
        "BatteryIO must reject high-rate CAN FD buses");
    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_bfg_can_decode_err_mv",
        "BFG 29-Bit CAN Frame Decode Error",
        bfgDecodeErrMv,
        0.0,
        1.0,
        "mV");

    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    DriveToPose driveToPose = new DriveToPose(drive, new Pose2d(0.50, 0.0, Rotation2d.kZero));
    driveToPose.initialize();
    for (int i = 0; i < 60 && !driveToPose.isFinished(); i++) {
      driveToPose.execute();
      drive.periodic();
    }
    driveToPose.end(false);
    assertTrue(drive.getPose().getX() > 0.40, "DriveToPose must advance toward target pose");

    PreMatchCalibrator calibrator = new PreMatchCalibrator(drive, vision);
    calibrator.setExpectedStartPose(new Pose2d(0.02, -0.01, Rotation2d.kZero));
    for (int i = 0; i < PreMatchCalibrator.REQUIRED_SAMPLES; i++) {
      calibrator.addObservation(new Pose2d(0.02, -0.01, Rotation2d.fromDegrees(0.5)));
    }
    assertTrue(calibrator.isGyroSeeded());
    assertTrue(calibrator.isReadyToPlay());

    HardStopHomingCmd homingCmd =
        new HardStopHomingCmd(fuelContainer.getHood(), -2.0, 0.0, 50.0, 2, 10);
    homingCmd.initialize();
    for (int i = 0; i < 5 && !homingCmd.isFinished(); i++) {
      homingCmd.execute();
      fuelContainer.getHood().periodic();
    }
    homingCmd.end(false);
    assertTrue(homingCmd.isHomed());

    // Unskipped 4-phase ZeroPriorBootstrapCmd execution & persistence
    Path calCsvPath = Path.of("/tmp/cameras-calibrated.csv");
    Path shopJsonPath = Path.of("/tmp/shop-5x3.json");
    Files.deleteIfExists(calCsvPath);
    Files.deleteIfExists(shopJsonPath);
    ZeroPriorBootstrapCmd fullBootstrap = new ZeroPriorBootstrapCmd(drive, () -> true);
    fullBootstrap.setCurrentCameraResidualMeters(0.25);
    fullBootstrap.initialize();
    for (int i = 0; i < 60 && !fullBootstrap.isFinished(); i++) {
      fullBootstrap.execute();
      drive.periodic();
      vision.periodic();
    }
    fullBootstrap.end(false);
    assertFalse(fullBootstrap.wasSkipped());
    assertTrue(fullBootstrap.isCompleted());
    assertEquals(24.5, Math.toDegrees(fullBootstrap.getSolvedExtrinsics().yawRad()), 1.5);
    assertTrue(Files.exists(calCsvPath) && Files.exists(shopJsonPath));

    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    Path checkpointFile = Path.of("build/test-blitz-progress.json");
    Files.deleteIfExists(checkpointFile);
    PitSystemCheckCmd pitCheck = fuelContainer.createPitSystemCheckCommand();
    HardStopHomingCmd blitzHoming =
        new HardStopHomingCmd(fuelContainer.getHood(), -2.0, 0.0, 50.0, 2, 10);
    ZeroPriorBootstrapCmd blitzBootstrap = new ZeroPriorBootstrapCmd(drive, () -> true);
    blitzBootstrap.setCurrentCameraResidualMeters(0.010);
    ShopStressSuiteCmd stressSuite = new ShopStressSuiteCmd(drive, () -> true);

    Blitz15MinCmd blitz =
        new Blitz15MinCmd(pitCheck, blitzHoming, blitzBootstrap, stressSuite, checkpointFile);
    blitz.initialize();
    for (int i = 0; i < pitCheck.getTotalLoops() + 1; i++) {
      blitz.execute();
      drive.periodic();
      fuelContainer.getTilter().periodic();
      fuelContainer.getHood().periodic();
      fuelContainer.getIntake().periodic();
      fuelContainer.getIndexer().periodic();
      fuelContainer.getFeeder().periodic();
      fuelContainer.getShooter().periodic();
    }
    blitz.end(true);
    assertTrue(pitCheck.hasPassed(), "Healthy sim PitSystemCheckCmd must pass: " + pitCheck.getSummary());
    assertEquals(1, blitz.getCurrentStageIndex());

    // After pit check on blocks, place robot back at shop center (0, 0) for Stages 2-4
    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    blitz.initialize();
    for (int i = 0; i < 200 && !blitz.isFinished(); i++) {
      vision.periodic();
      blitz.execute();
      drive.periodic();
      fuelContainer.getHood().periodic();
    }
    blitz.end(false);
    assertTrue(blitz.isFinished());
    assertEquals(0, stressSuite.getWallBreachCount());

    // Blind-drive safety check on ShopStressSuiteCmd
    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fig, "FUEL");
    Drive blindDrive =
        new Drive(
            swerveConfig,
            new GyroIO() {},
            new ModuleIO[] {
              new ModuleIOSim(swerveConfig),
              new ModuleIOSim(swerveConfig),
              new ModuleIOSim(swerveConfig),
              new ModuleIOSim(swerveConfig)
            });
    ShopStressSuiteCmd blindStress = new ShopStressSuiteCmd(blindDrive, () -> true);
    blindStress.initialize();
    blindStress.execute();
    blindDrive.periodic();
    assertEquals(
        0.0,
        Math.hypot(
            blindDrive.getChassisSpeeds().vxMetersPerSecond,
            blindDrive.getChassisSpeeds().vyMetersPerSecond),
        1e-4,
        "ShopStressSuiteCmd must halt translation when Drive has no recent AprilTag sight (> 0.50s)");

    ShotCalibrationCmd shotCalCmd = fuelContainer.createShotCalibrationCommand();
    shotCalCmd.initialize();
    for (int i = 0; i < 65 && !shotCalCmd.isFinished(); i++) {
      shotCalCmd.execute();
      fuelContainer.getSuperstructure().periodic();
    }
    shotCalCmd.end(false);
    assertTrue(shotCalCmd.isFinished());
    assertEquals(5, shotCalCmd.getRecordedCsvRows().size());

    assertTrue(drive.getPathStartingPose("Hub to Depot").isPresent());
    assertTrue(drive.getPathStartingPose("Left to Depot").isPresent());
    assertTrue(drive.getPathStartingPose("Right to Depot").isPresent());
    assertTrue(drive.getPathStartingPose("Collect").isPresent());
    assertTrue(drive.getPathStartingPose("Shop_Center_Shuttle").isPresent());
    assertTrue(drive.getPathStartingPose("PreKickoff_2Piece_Template").isPresent());
    String prevLogDir = System.getProperty("flux.logDir");
    System.setProperty("flux.logDir", Path.of("build/test-robot-logs").toAbsolutePath().toString());
    try (Robot robot = new Robot(Path.of("build/test-robot-logs/flux-robot-state"))) {
      robot.robotInit();
      robot.disabledInit();
      robot.disabledPeriodic();
      robot.autonomousInit();
      robot.robotPeriodic();
      robot.teleopInit();
      robot.robotPeriodic();
      assertNotNull(robot.getRobotContainer());
      assertEquals(
          robot.getPeriod(),
          robot.getRobotContainer().getDrive().getLoopPeriodSeconds(),
          1e-6,
          "LoggedRobot loop period and Drive discretize loop period must be 100% locked together");
    } finally {
      if (prevLogDir == null) {
        System.clearProperty("flux.logDir");
      } else {
        System.setProperty("flux.logDir", prevLogDir);
      }
    }
  }

  @Test
  public void testSwerveConfigHardwareParityAgainstFleetSpecs() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    assertEquals(
        20,
        fig.getTable("swerve_modules").size(),
        "swerve_modules.csv must define 4 corners across all 5 fleet robots (20 rows)");
    SwerveConfig compCfg = SwerveConfig.fromFluxFig(fig, "COMP_2027");
    SwerveConfig fuelCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    SwerveConfig coralCfg = SwerveConfig.fromFluxFig(fig, "CORAL");
    SwerveConfig chassis3Cfg = SwerveConfig.fromFluxFig(fig, "CHASSIS_3");

    int mismatches = 0;
    if (Math.abs(fuelCfg.driveGearRatio() - 6.3947) > 0.01) mismatches++;
    if (Math.abs(fuelCfg.turnGearRatio() - 12.1) > 0.01) mismatches++;
    if (Math.abs(coralCfg.driveGearRatio() - 6.3947) > 0.01) mismatches++;
    if (Math.abs(coralCfg.turnGearRatio() - 12.1) > 0.01) mismatches++;
    if (Math.abs(fuelCfg.corners()[0].encoderOffsetRot() - 0.105225) > 1e-3) mismatches++;
    if (Math.abs(coralCfg.trackWidthMeters() - 0.5969) > 1e-3) mismatches++;
    if (coralCfg.corners()[0].driveMotorId() != 7 || coralCfg.corners()[0].turnMotorId() != 8) {
      mismatches++;
    }
    if (Math.abs(coralCfg.corners()[0].encoderOffsetRot() - 0.121338) > 1e-3) mismatches++;
    if (compCfg.corners()[0].invertDrive() || !compCfg.corners()[1].invertDrive()) mismatches++;
    if (chassis3Cfg.pigeonId() != 24 || chassis3Cfg.corners()[0].driveMotorId() != 7) mismatches++;

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_swerve_config_mismatches",
        "Swerve Fleet Hardware Config Mismatches",
        mismatches,
        0.0,
        0.0,
        "mismatches");

    assertEquals(6.3947, fuelCfg.driveGearRatio(), 0.01);
    assertEquals(12.1, fuelCfg.turnGearRatio(), 0.01);
    assertEquals(6.3947, coralCfg.driveGearRatio(), 0.01);
    assertEquals(12.1, coralCfg.turnGearRatio(), 0.01);

    assertEquals(20, fuelCfg.pigeonId());
    assertEquals(0.105225, fuelCfg.corners()[0].encoderOffsetRot(), 1e-3);
    assertEquals(-0.120605, fuelCfg.corners()[1].encoderOffsetRot(), 1e-3);
    assertEquals(-0.466797, fuelCfg.corners()[2].encoderOffsetRot(), 1e-3);
    assertEquals(-0.037109, fuelCfg.corners()[3].encoderOffsetRot(), 1e-3);

    assertEquals(24, coralCfg.pigeonId());
    assertEquals(0.5969, coralCfg.trackWidthMeters(), 1e-3);
    assertEquals(7, coralCfg.corners()[0].driveMotorId());
    assertEquals(8, coralCfg.corners()[0].turnMotorId());
    assertEquals(23, coralCfg.corners()[0].cancoderId());
    assertEquals(0.121338, coralCfg.corners()[0].encoderOffsetRot(), 1e-3);
    assertEquals(1, coralCfg.corners()[1].driveMotorId());
    assertEquals(2, coralCfg.corners()[1].turnMotorId());
    assertEquals(20, coralCfg.corners()[1].cancoderId());
    assertEquals(-0.294922, coralCfg.corners()[1].encoderOffsetRot(), 1e-3);
    assertEquals(0.040771, coralCfg.corners()[2].encoderOffsetRot(), 1e-3);
    assertEquals(-0.376953, coralCfg.corners()[3].encoderOffsetRot(), 1e-3);
  }

  @Test
  public void testStudentDefaultAutoAndExampleAutoRender2026FieldShootingVideo() throws Exception {
    Path configDir = Paths.get("src/main/deploy/configs");
    RobotContainer fuelContainer = new RobotContainer(RobotIdentity.FUEL, configDir, true);

    var defaultAuto = fuelContainer.getAutonomousCommand();
    assertNotNull(defaultAuto, "Default autonomous routine must not be null");

    Path outDir = Paths.get("build/auto-sim");
    var frames =
        StudentAutoSimExporter.simulateAndExportJson(
            fuelContainer, defaultAuto, defaultAuto.getName(), outDir);
    assertTrue(frames.size() >= 375, "Expected 376 frames (15.0s at 25 Hz)");
    assertTrue(Files.exists(outDir.resolve("student-auto-frames.json")));

    String jsonText = Files.readString(outDir.resolve("student-auto-frames.json"));
    assertTrue(jsonText.contains("\"fieldMode\": \"REBUILT_2026_SHOOTING\""));
    double minDefaultVBat =
        frames.stream().mapToDouble(f -> f.batteryVoltage()).min().orElse(0.0);
    assertTrue(minDefaultVBat >= 7.50, "Student default auto must not brown out battery (< 7.50V)");

    // Verify deterministic PathPlanner 'Hub to Depot' + Hub Shooting physics on the reference auto
    var hubToDepotAuto =
        StudentAutoRoutines.createHubToDepotAuto(
            fuelContainer.getDrive(),
            fuelContainer.getSuperstructure(),
            fuelContainer.getShotCalculator());
    Path depotDir = Paths.get("build/auto-sim-hub-to-depot");
    var depotFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            fuelContainer, hubToDepotAuto, hubToDepotAuto.getName(), depotDir);
    String depotJson = Files.readString(depotDir.resolve("student-auto-frames.json"));
    assertTrue(
        depotJson.contains("\"scored\":true"),
        "Reference 'Hub to Depot' auto must fire and score FUEL shots into the Blue Hub");
    double maxX = depotFrames.stream().mapToDouble(f -> f.xMeters()).max().orElse(0.0);
    double minX = depotFrames.stream().mapToDouble(f -> f.xMeters()).min().orElse(99.0);
    double maxY = depotFrames.stream().mapToDouble(f -> f.yMeters()).max().orElse(0.0);
    assertTrue(maxX > 2.8, "Hub to Depot auto must start/return past X = 2.8m at Blue Hub");
    assertTrue(
        minX < 1.30 && maxY > 5.8,
        "Hub to Depot auto must follow PathPlanner 'Hub to Depot.path' into Blue Depot (X < 1.30m, Y > 5.8m)");

    var exampleAuto = fuelContainer.createExampleAutonomousCommand();
    assertEquals(StudentAutoRoutines.EXAMPLE_AUTO_NAME, exampleAuto.getName());
    Path exDir = Paths.get("build/auto-sim-example");
    var exFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            fuelContainer, exampleAuto, exampleAuto.getName(), exDir);
    assertTrue(exFrames.size() >= 375);
    assertTrue(Files.exists(exDir.resolve("student-auto-frames.json")));

    // Track A: Fixed code moves (driveRelative + waitLoopSeconds + driveTimedVelocity + Hub shoot)
    var fixedMovesAuto =
        StudentAutoRoutines.createFixedMovesExampleAuto(
            fuelContainer.getDrive(),
            fuelContainer.getSuperstructure(),
            fuelContainer.getShotCalculator());
    Path fixedDir = Paths.get("build/auto-sim-fixed-moves");
    var fixedFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            fuelContainer, fixedMovesAuto, fixedMovesAuto.getName(), fixedDir);
    String fixedJson = Files.readString(fixedDir.resolve("student-auto-frames.json"));
    double minFixedY = fixedFrames.stream().mapToDouble(f -> f.yMeters()).min().orElse(99.0);
    assertTrue(minFixedY < 3.0, "Fixed-moves example must reach South Wing FUEL balls (Y < 3.0m)");
    assertTrue(
        fixedJson.contains("\"scored\":true"),
        "Fixed-moves example must fire and score FUEL shots into the Blue Hub");

    // Track B: Chained 2-path PathPlanner routine ("Hub to Depot" -> shoot -> "Collect")
    var fullCycleAuto =
        StudentAutoRoutines.createFullCyclePathPlannerAuto(
            fuelContainer.getDrive(),
            fuelContainer.getSuperstructure(),
            fuelContainer.getShotCalculator());
    Path fullCycleDir = Paths.get("build/auto-sim-full-cycle");
    var fullCycleFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            fuelContainer, fullCycleAuto, fullCycleAuto.getName(), fullCycleDir);
    double maxCycleX = fullCycleFrames.stream().mapToDouble(f -> f.xMeters()).max().orElse(0.0);
    assertTrue(
        SafeAutoBuilder.getLastCancelReason().isBlank(),
        "2-path 'Hub to Depot' + 'Collect' chain must pass SafeAutoBuilder preflight checks");
    assertTrue(
        maxCycleX > 6.5,
        "2-path PathPlanner auto must chain 'Collect.path' out toward centerline (X > 6.5m)");
  }

  @Test
  public void testBomRadarOriginConversionIdentityBudgetVisionAndDeadband() throws Exception {
    assertEquals(8.069, FieldPositions.FIELD_WIDTH_METERS, 1e-6);

    CoordinateSystem savedCoord = FieldPositions.getCoordinateSystem();
    try {
      FieldPositions.setCoordinateSystem(CoordinateSystem.CENTER_RED_2027);
      Pose2d shopStart = SafeAutoBuilder.getPathStartingPose("Shop_Center_Shuttle").orElseThrow();
      assertEquals(0.0, shopStart.getX(), 1e-4, "LocalOrigin shop path must stay at (0,0)");
      assertEquals(0.0, shopStart.getY(), 1e-4);

      Pose2d compHubStart = SafeAutoBuilder.getPathStartingPose("Hub to Depot").orElseThrow();
      Pose2d expectedCompStart =
          FieldPositions.wallBlueToCenterRed(new Pose2d(3.10, 4.05, Rotation2d.kZero));
      assertEquals(expectedCompStart.getX(), compHubStart.getX(), 1e-3);
      assertEquals(expectedCompStart.getY(), compHubStart.getY(), 1e-3);
      assertEquals(Math.PI, Math.abs(compHubStart.getRotation().getRadians()), 1e-3);
    } finally {
      FieldPositions.setCoordinateSystem(savedCoord);
    }

    Path tmpDir = Files.createTempDirectory("flux-id-test");
    try {
      Path devSerial = tmpDir.resolve("serial-number");
      Path deployTag = tmpDir.resolve("deploy-id.txt");
      Path persistTag = tmpDir.resolve("persist-id.txt");
      Files.writeString(devSerial, "PRACTICE_2027\0");
      assertEquals(
          RobotIdentity.PRACTICE_2027,
          RobotIdentity.detect("", "", devSerial, deployTag, persistTag));
      Files.deleteIfExists(devSerial);
      Files.writeString(persistTag, "PRACTICE_2027\n");
      assertEquals(
          RobotIdentity.PRACTICE_2027,
          RobotIdentity.detect("ABCDEF", "", devSerial, deployTag, persistTag));
    } finally {
      Files.deleteIfExists(tmpDir.resolve("serial-number"));
      Files.deleteIfExists(tmpDir.resolve("deploy-id.txt"));
      Files.deleteIfExists(tmpDir.resolve("persist-id.txt"));
      Files.deleteIfExists(tmpDir);
    }

    PhoenixUtil.resetConfigBudgetForTest(0.0);
    int[] okInvocations = {0};
    assertEquals(
        StatusCode.OK,
        PhoenixUtil.tryUntilOk(
            "ExpiredBudgetGuaranteedFirstAttempt",
            () -> {
              okInvocations[0]++;
              return StatusCode.OK;
            },
            5));
    assertEquals(
        1,
        okInvocations[0],
        "Every motor must execute at least 1 config attempt even after budget exhaustion");

    int[] failInvocations = {0};
    assertEquals(
        StatusCode.TxTimeout,
        PhoenixUtil.tryUntilOk(
            "ExpiredBudgetFailureSkipsRetries",
            () -> {
              failInvocations[0]++;
              return StatusCode.TxTimeout;
            },
            5));
    assertEquals(
        1,
        failInvocations[0],
        "Exhausted budget must execute 1 guaranteed attempt and skip remaining retries");
    assertTrue(PhoenixUtil.getSkippedConfigCallCount() >= 1);
    assertTrue(PhoenixUtil.getFailedConfigCallCount() >= 1);
    assertTrue(
        PhoenixUtil.getLastFailureMessage().contains("ExpiredBudgetFailureSkipsRetries"),
        "Failed CAN config must record a loud diagnostic failure message");
    PhoenixUtil.resetConfigBudgetForTest();

    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    CameraSpec[] cams = new CameraSpec[] {fig.getCamerasForRobot("FUEL").get(0)};
    final double[] recordedLinearStd = {Double.NaN};
    VisionIO stubIo =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            double ts = Timer.getFPGATimestamp();
            inputs.connected = true;
            inputs.tagIds = new int[] {1, 2};
            inputs.poseObservations =
                new PoseObservation[] {
                  new PoseObservation(
                      ts,
                      new Pose3d(Double.NaN, 0.0, 0.0, Rotation3d.kZero),
                      0.05,
                      2,
                      2.0,
                      ObservationType.MEGATAG_1),
                  new PoseObservation(
                      ts + 0.20,
                      new Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero),
                      0.05,
                      2,
                      2.0,
                      ObservationType.MEGATAG_1),
                  new PoseObservation(
                      ts,
                      new Pose3d(4.0, 4.0, 0.0, Rotation3d.kZero),
                      0.05,
                      2,
                      2.0,
                      ObservationType.MEGATAG_1)
                };
          }
        };
    FieldPositions.setCoordinateSystem(CoordinateSystem.WALL_BLUE_2026);
    try {
      Vision vision =
          new Vision(
              (pose, ts, std) -> recordedLinearStd[0] = std.get(0, 0), null, cams, stubIo);
      double nowSec = Timer.getFPGATimestamp();
      vision.recordMotionSample(2.1, 0.0, nowSec - 0.15);
      vision.periodic();
      assertEquals(0, vision.getAcceptedObservationCount(), "0.3s > 1.6 rad/s lookback must reject");

      vision.recordMotionSample(0.1, Math.toRadians(6.0), nowSec + 0.001);
      vision.periodic();
      assertEquals(0, vision.getAcceptedObservationCount(), "> 5 deg tilt must reject");

      Vision cleanVision =
          new Vision(
              (pose, ts, std) -> recordedLinearStd[0] = std.get(0, 0), null, cams, stubIo);
      cleanVision.periodic();
      assertEquals(1, cleanVision.getAcceptedObservationCount());
      assertEquals(0.01, recordedLinearStd[0], 1e-6);
      assertEquals(1.0, Vision.computeOdometryTiltTrustScale(Math.toRadians(5.0)), 1e-6);
      assertEquals(0.5, Vision.computeOdometryTiltTrustScale(Math.toRadians(15.0)), 1e-6);
      assertEquals(0.0, Vision.computeOdometryTiltTrustScale(Math.toRadians(25.0)), 1e-6);
    } finally {
      FieldPositions.setCoordinateSystem(savedCoord);
    }

    int[] turnCommands = {0};
    ModuleIO spyModuleIo =
        new ModuleIO() {
          @Override
          public void setTurnPosition(Rotation2d angle) {
            turnCommands[0]++;
          }
        };
    Module deadbandModule = new Module(spyModuleIo, 0, 0.05);
    deadbandModule.runSetpoint(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.2)));
    assertEquals(0, turnCommands[0], "<= 0.3 deg stationary micro-jitter must be ignored");
    deadbandModule.runSetpoint(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    assertEquals(1, turnCommands[0], "45 deg stopWithX rotation must command steer angle");
  }

  @Test
  public void testTrackAM12IdentityConfigReloadHonestyFmsGuardsLogMountAndTick1Tunable()
      throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-m12-test");
    try {
      // Distinct dual-robot roboRIO resolution via hex serial map and /home/lvuser tag
      Map<String, RobotIdentity> serialMap =
          Map.of("0323AAAA", RobotIdentity.FUEL, "0323BBBB", RobotIdentity.CORAL);
      assertEquals(
          RobotIdentity.FUEL,
          RobotIdentity.detect("0x0323aaaa", "", null, null, null, null, serialMap));
      assertFalse(RobotIdentity.usedFallback());
      assertEquals("HAL_SERIAL_MAP", RobotIdentity.getLastResolutionSource());

      assertEquals(
          RobotIdentity.CORAL,
          RobotIdentity.detect("0323BBBB", "", null, null, null, null, serialMap));
      assertFalse(RobotIdentity.usedFallback());

      Path fuelRioTag = tmpDir.resolve("fuel-identity.txt");
      Path coralRioTag = tmpDir.resolve("coral-identity.txt");
      Files.writeString(fuelRioTag, "FUEL\n");
      Files.writeString(coralRioTag, "CORAL\n");
      assertEquals(
          RobotIdentity.FUEL,
          RobotIdentity.detect("0323FFFF", "", null, fuelRioTag, null, null, serialMap));
      assertEquals("ROBORIO_PERSISTENT_TAG", RobotIdentity.getLastResolutionSource());
      assertEquals(
          RobotIdentity.CORAL,
          RobotIdentity.detect("0323EEEE", "", null, coralRioTag, null, null, serialMap));

      // Unknown real hex serial must warn loudly and fall back to roboRIO-safe FUEL profile
      assertEquals(
          RobotIdentity.FUEL,
          RobotIdentity.detect("0323CAFE", "", null, null, null, null, serialMap));
      assertTrue(RobotIdentity.usedFallback(), "Unknown hex serial must flag fallback");
      assertEquals("DEFAULT_FUEL", RobotIdentity.getLastResolutionSource());
      assertTrue(
          RobotIdentity.getLastWarningMessage().contains("0323CAFE"),
          "Fallback error must name the unrecognized hex serial");

      // Disabled config-reload honesty: disk edits flag RestartRequired without mutating boot SHA
      Path cfgCopy = tmpDir.resolve("configs");
      Files.createDirectories(cfgCopy);
      try (var stream = Files.list(Paths.get("src/main/deploy/configs"))) {
        for (Path src : stream.toList()) {
          if (Files.isRegularFile(src)) {
            Files.copy(src, cfgCopy.resolve(src.getFileName()), StandardCopyOption.REPLACE_EXISTING);
          }
        }
      }
      RobotContainer container = new RobotContainer(RobotIdentity.FUEL, cfgCopy, true);
      String bootSha = container.getBootConfigSha256();
      assertFalse(container.isConfigRestartRequired());
      container.disabledPeriodic();
      assertFalse(container.isConfigRestartRequired());

      Path paramsCopy = cfgCopy.resolve("parameters.csv");
      Files.writeString(paramsCopy, Files.readString(paramsCopy) + "\n");
      assertTrue(container.checkConfigDiskChangeNow());
      assertTrue(container.isConfigRestartRequired());
      assertEquals(
          bootSha,
          container.getFluxFig().getConfigSha256(),
          "FluxFig SHA must remain locked to boot gains until restart");

      // FMS + WPILib Test-mode pit/homing guard and testInit() command cancellation
      try {
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();
        assertFalse(RobotContainer.isPitTestAllowed(), "Pit/homing commands blocked outside Test");

        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        assertTrue(RobotContainer.isPitTestAllowed(), "Pit/homing allowed in Test mode off-FMS");

        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        assertFalse(RobotContainer.isPitTestAllowed(), "Pit/homing always blocked on FMS");

        DriverStationSim.setFmsAttached(false);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        Path stateFile = tmpDir.resolve("flux-robot-state");
        String prevM12LogDir = System.getProperty("flux.logDir");
        System.setProperty("flux.logDir", tmpDir.resolve("logs").toString());
        try (Robot robot = new Robot(stateFile)) {
          robot.robotInit();
          assertEquals(
              Robot.ROBORIO_BROWNOUT_VOLTAGE_V,
              RobotController.getBrownoutVoltage(),
              1e-3,
              "Robot must set roboRIO brownout threshold to 6.0 V");
          String disabledState = Files.readString(stateFile);
          assertTrue(disabledState.contains("STATE=DISABLED"));
          assertTrue(disabledState.contains("ENABLED=false"));
          assertTrue(disabledState.contains("FMS=false"));
          assertTrue(disabledState.contains("ROBOT_ID="));
          assertTrue(disabledState.contains("EPOCH_MS="));

          DriverStationSim.setEnabled(true);
          DriverStationSim.notifyNewData();
          robot.teleopInit();
          String enabledState = Files.readString(stateFile);
          assertTrue(enabledState.contains("STATE=ENABLED"));
          assertTrue(enabledState.contains("ENABLED=true"));

          boolean[] canceled = {false};
          Command dummyCmd = Commands.run(() -> {}).finallyDo(interrupted -> canceled[0] = interrupted);
          CommandScheduler.getInstance().schedule(dummyCmd);
          robot.testInit();
          assertTrue(canceled[0], "Robot.testInit() must cancel active commands cleanly");
        } finally {
          if (prevM12LogDir == null) {
            System.clearProperty("flux.logDir");
          } else {
            System.setProperty("flux.logDir", prevM12LogDir);
          }
        }
        assertFalse(Files.exists(stateFile), "Clean Robot.close() must remove state heartbeat file");
      } finally {
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
      }

      // LogFileManager /U mountpoint check and empty .hoot directory cleanup
      Path fakeUsb = tmpDir.resolve("fake-usb");
      Files.createDirectories(fakeUsb);
      Path emptyMounts = tmpDir.resolve("empty-mounts");
      Files.writeString(emptyMounts, "/dev/root / ext4 rw 0 0\n");
      assertFalse(
          LogFileManager.isMountedWritableDirectory(fakeUsb, tmpDir, emptyMounts),
          "Directory on same filesystem and absent from /proc/mounts must not look like mounted /U");
      Path populatedMounts = tmpDir.resolve("populated-mounts");
      Files.writeString(
          populatedMounts, "/dev/sda1 " + fakeUsb.toRealPath() + " vfat rw 0 0\n");
      assertTrue(
          LogFileManager.isMountedWritableDirectory(fakeUsb, tmpDir, populatedMounts),
          "Writable directory listed in /proc/mounts must pass mount check");

      Path cleanupRoot = tmpDir.resolve("cleanup-logs");
      Path nestedHootDir = cleanupRoot.resolve("old-hoot-dir");
      Files.createDirectories(nestedHootDir);
      Path oldHoot = nestedHootDir.resolve("old.hoot");
      Files.write(oldHoot, new byte[2048]);
      LogFileManager.cleanOldLogs(cleanupRoot.toFile(), 100L);
      assertFalse(Files.exists(oldHoot), "Old .hoot file exceeding cap must be pruned");
      assertFalse(Files.exists(nestedHootDir), "Empty .hoot directory must be pruned after cleanup");

      // Tick-1 LoggedTunableNumber must produce zero redundant io.setPID CAN writes
      FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
      MechanismSpec shooterSpec = fig.getMechanismSpec("FUEL", "Shooter");
      int[] pidWrites = {0};
      MechanismIO spyMechIo =
          new MechanismIO() {
            @Override
            public void setPID(
                double kP, double kI, double kD, double kS, double kV, double kG) {
              pidWrites[0]++;
            }
          };
      LoggedTunableNumber.setTuningMode(false);
      VelocityMechanism mech = new VelocityMechanism(spyMechIo, shooterSpec);
      mech.periodic();
      assertEquals(0, pidWrites[0], "Production tick 1 must perform zero redundant setPID writes");
      LoggedTunableNumber.setTuningMode(true);
      mech.periodic();
      assertEquals(0, pidWrites[0], "Unchanged tuning tick 1 must perform zero redundant setPID writes");
      LoggedTunableNumber.setTuningMode(false);
    } finally {
      try (var walk = Files.walk(tmpDir)) {
        walk.sorted(java.util.Comparator.reverseOrder())
            .forEach(
                p -> {
                  try {
                    Files.deleteIfExists(p);
                  } catch (Exception ignored) {
                  }
                });
      }
    }
  }

  @Test
  public void testTrackAM14FuelSafeRpsHeadingGateAndPerChannelUnjam() {
    // Hub target coordinates and proven <= 40 RPS defaults
    assertEquals(4.626, RobotContainer.HUB_TARGET_METERS.getX(), 1e-4);
    assertEquals(4.035, RobotContainer.HUB_TARGET_METERS.getY(), 1e-4);
    assertTrue(Superstructure.DEFAULT_SHORT_SHOOTER_RPS <= 40.0);
    assertTrue(Superstructure.DEFAULT_MID_SHOOTER_RPS <= 40.0);
    assertTrue(Superstructure.DEFAULT_LONG_SHOOTER_RPS <= 40.0);

    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    for (var row : fig.getTable("range_table")) {
      double rps = Double.parseDouble(row.get("shooterRps"));
      assertTrue(rps >= 33.0 && rps <= 40.0, "range_table.csv RPS out of safe NYTR bounds: " + rps);
    }

    // Heading-gated and range-gated fire interlock on FUEL Superstructure
    RobotContainer fuelContainer =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    Superstructure ss = fuelContainer.getSuperstructure();
    ShotCalculator calc = fuelContainer.getShotCalculator();
    ShotSolution validSol =
        calc.calculate(
            new Pose2d(2.5, 4.035, Rotation2d.kZero),
            new ChassisSpeeds(),
            RobotContainer.HUB_TARGET_METERS);
    assertTrue(validSol.shooterRps() <= 40.0);
    assertTrue(calc.isInRange(validSol.effectiveDistanceMeters()));

    ss.applyShotSolution(validSol, true, Math.toRadians(20.0), true);
    for (int i = 0; i < 80; i++) {
      ss.getShooter().periodic();
      ss.getHood().periodic();
      ss.getIndexer().periodic();
      ss.getFeeder().periodic();
      ss.periodic();
    }
    assertFalse(
        ss.isReadyToFire(),
        "Heading error > 5 deg must block firing even after shooter and hood reach setpoint");
    assertEquals(0.0, ss.getFeeder().getTargetVelocity(), 1e-4);

    ss.applyShotSolution(validSol, true, Math.toRadians(1.0), false);
    ss.periodic();
    assertFalse(ss.isReadyToFire(), "Out-of-range shot must block firing");

    ss.applyShotSolution(validSol, true);
    ss.periodic();
    assertFalse(ss.isReadyToFire(), "2-arg applyShotSolution must fail closed without aim status");

    ss.applyShotSolution(validSol, true, Math.toRadians(1.5), true);
    ss.periodic();
    assertTrue(
        ss.isReadyToFire(),
        "Aligned heading (<= 5 deg), in-range distance, and settled shooter/hood must unlock fire");
    assertEquals(Superstructure.DEFAULT_FEEDER_RPS, ss.getFeeder().getTargetVelocity(), 1e-4);

    // Independent per-channel unjam timers across Intake, Indexer, and Feeder
    FaultSupervisor sup = new FaultSupervisor(3);
    double intakeCmd = 8.0;
    double indexerCmd = 8.0;
    double feederCmd = -8.0;
    for (int i = 0; i < FaultSupervisor.JAM_DETECT_LOOPS; i++) {
      intakeCmd =
          sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_INTAKE, 8.0, 48.0, 0.05);
      indexerCmd =
          sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_INDEXER, 8.0, 10.0, 35.0);
      feederCmd =
          sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_FEEDER, -8.0, 46.0, 0.0);
    }
    assertEquals(
        FaultSupervisor.JAM_REVERSE_VOLTAGE,
        intakeCmd,
        1e-6,
        "Positive-running jammed Intake must pulse -4.0 V reverse");
    assertEquals(
        8.0,
        indexerCmd,
        1e-6,
        "Healthy Indexer must stay unaffected while Intake and Feeder unjam");
    assertEquals(
        -FaultSupervisor.JAM_REVERSE_VOLTAGE,
        feederCmd,
        1e-6,
        "Negative-running jammed Feeder must pulse +4.0 V opposite-sign reverse");
    assertEquals(1, sup.dejamEventsTriggered(FaultSupervisor.JAM_CHANNEL_INTAKE));
    assertEquals(0, sup.dejamEventsTriggered(FaultSupervisor.JAM_CHANNEL_INDEXER));
    assertEquals(1, sup.dejamEventsTriggered(FaultSupervisor.JAM_CHANNEL_FEEDER));

    // Verify Superstructure INTAKING ticks per-channel Intake unjamming
    FaultSupervisor ssSup = new FaultSupervisor(3);
    double[] lastIntakeVolts = {0.0};
    MechanismIO stalledIntakeIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.statorCurrentA = 49.0;
            inputs.velocityRPS = 0.0;
            inputs.motorConnected = true;
          }

          @Override
          public void setVoltage(double volts) {
            lastIntakeVolts[0] = volts;
          }
        };
    VelocityMechanism stalledIntake =
        new VelocityMechanism(stalledIntakeIo, fig.getMechanismSpec("FUEL", "Intake"));
    PositionMechanism simTilter =
        new PositionMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Tilter")),
            fig.getMechanismSpec("FUEL", "Tilter"));
    VelocityMechanism simIndexer =
        new VelocityMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Indexer")),
            fig.getMechanismSpec("FUEL", "Indexer"));
    VelocityMechanism simFeeder =
        new VelocityMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Feeder")),
            fig.getMechanismSpec("FUEL", "Feeder"));
    VelocityMechanism simShooter =
        new VelocityMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Shooter")),
            fig.getMechanismSpec("FUEL", "Shooter"));
    PositionMechanism simHood =
        new PositionMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Hood")),
            fig.getMechanismSpec("FUEL", "Hood"));
    Superstructure unjamSs =
        new Superstructure(
            stalledIntake, simTilter, simIndexer, simFeeder, simShooter, simHood, ssSup);
    unjamSs.requestState(State.INTAKING);
    for (int i = 0; i < FaultSupervisor.JAM_DETECT_LOOPS + 2; i++) {
      stalledIntake.periodic();
      simIndexer.periodic();
      unjamSs.periodic();
    }
    assertEquals(
        FaultSupervisor.JAM_REVERSE_VOLTAGE,
        lastIntakeVolts[0],
        1e-6,
        "Superstructure INTAKING must drive reverse unjam voltage on stalled Intake");
    assertTrue(ssSup.dejamEventsTriggered(FaultSupervisor.JAM_CHANNEL_INTAKE) >= 1);
  }

  @Test
  public void testDisableWhileShootingReturnsToIdleOnNextEnable() throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-blk03-test");
    String prevLogDir = System.getProperty("flux.logDir");
    System.setProperty("flux.logDir", tmpDir.resolve("logs").toString());
    try (Robot robot = new Robot(tmpDir.resolve("flux-robot-state"))) {
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();

      robot.robotInit();
      Superstructure ss = robot.getRobotContainer().getSuperstructure();

      // Enable Teleop and latch SHOOTING
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      robot.teleopInit();
      ss.shoot(Superstructure.DEFAULT_MID_SHOOTER_RPS, Superstructure.DEFAULT_MID_HOOD_ROT);
      robot.robotPeriodic();
      assertEquals(State.SHOOTING, ss.getState());
      assertEquals(
          Superstructure.DEFAULT_MID_SHOOTER_RPS, ss.getShooter().getTargetVelocity(), 1e-4);

      // Disable while shooting: disabledInit() must immediately return Superstructure to IDLE
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      robot.disabledInit();
      robot.robotPeriodic();
      assertEquals(State.IDLE, ss.getState(), "disabledInit() must reset Superstructure to IDLE");
      assertEquals(0.0, ss.getShooter().getTargetVelocity(), 1e-4);

      // Re-enable Teleop: must remain IDLE with 0 RPS
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      robot.teleopInit();
      robot.robotPeriodic();
      assertEquals(State.IDLE, ss.getState(), "teleopInit() must keep Superstructure in IDLE");
      assertEquals(0.0, ss.getShooter().getTargetVelocity(), 1e-4);

      // Latch INTAKING then enter Test mode: testInit() must reset to IDLE with 0 RPS
      ss.requestState(State.INTAKING);
      assertEquals(State.INTAKING, ss.getState());
      DriverStationSim.setTest(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      robot.testInit();
      robot.robotPeriodic();
      assertEquals(State.IDLE, ss.getState(), "testInit() must reset Superstructure to IDLE");
      assertEquals(0.0, ss.getIntake().getTargetVelocity(), 1e-4);
      assertEquals(0.0, ss.getShooter().getTargetVelocity(), 1e-4);

      // Verify .onFalse(...).ignoringDisable(true) fires while Disabled when trigger is released
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      XboxControllerSim opSim = new XboxControllerSim(1);
      opSim.setLeftBumperButton(true);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      assertEquals(State.INTAKING, ss.getState());

      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      // Re-latch manually to prove the .onFalse().ignoringDisable(true) binding itself fires
      ss.requestState(State.INTAKING);
      opSim.setLeftBumperButton(false);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      assertEquals(
          State.IDLE,
          ss.getState(),
          "Releasing LB while Disabled must trigger .ignoringDisable(true) IDLE reset");
    } finally {
      CommandScheduler.getInstance().cancelAll();
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      if (prevLogDir == null) {
        System.clearProperty("flux.logDir");
      } else {
        System.setProperty("flux.logDir", prevLogDir);
      }
      deleteTree(tmpDir);
    }
  }

  @Test
  public void testPitSystemCheckPassesHealthyAndFailsOnDisconnectedMechanism() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);

    PitSystemCheckCmd prodPit = container.createPitSystemCheckCommand();
    assertTrue(
        prodPit.getRequirements().contains(container.getSuperstructure()),
        "PitSystemCheckCmd must require Superstructure so active commands cannot fight it");
    double nominalSec = prodPit.getTotalLoops() * 0.020;
    assertTrue(
        nominalSec >= 15.0 && nominalSec <= 20.0,
        "Nominal PitSystemCheckCmd duration must be 15-20s (got " + nominalSec + " s)");

    // Run fast-step healthy check verifying every actuator passes with safe directions
    PitSystemCheckCmd healthyCheck =
        new PitSystemCheckCmd(
            container.getDrive(),
            List.of(
                new PositionCheckTarget(container.getTilter(), -2.0, 0.0),
                new PositionCheckTarget(container.getHood(), 2.0, 0.0)),
            List.of(
                new VelocityCheckTarget(container.getIntake(), 20.0),
                new VelocityCheckTarget(container.getIndexer(), 20.0),
                new VelocityCheckTarget(container.getFeeder(), -20.0),
                new VelocityCheckTarget(container.getShooter(), 25.0)),
            10,
            4,
            container::resetSuperstructureToIdle,
            container.getSuperstructure());
    healthyCheck.initialize();
    double minTilterPos = 0.0;
    double maxHoodPos = 0.0;
    double minFeederVel = 0.0;
    while (!healthyCheck.isFinished()) {
      healthyCheck.execute();
      container.getDrive().periodic();
      container.getTilter().periodic();
      container.getHood().periodic();
      container.getIntake().periodic();
      container.getIndexer().periodic();
      container.getFeeder().periodic();
      container.getShooter().periodic();
      minTilterPos = Math.min(minTilterPos, container.getTilter().getPosition());
      maxHoodPos = Math.max(maxHoodPos, container.getHood().getPosition());
      minFeederVel = Math.min(minFeederVel, container.getFeeder().getVelocity());
    }
    healthyCheck.end(false);
    assertTrue(
        healthyCheck.hasPassed(), "Healthy sim pit check must pass: " + healthyCheck.getSummary());
    assertTrue(healthyCheck.getFailedActuators().isEmpty());
    assertTrue(minTilterPos < -0.5, "Tilter must step in negative safe direction (-2.0 -> 0.0)");
    assertTrue(maxHoodPos > 0.5, "Hood must step in positive safe direction (2.0 -> 0.0)");
    assertTrue(minFeederVel < -5.0, "Feeder must step in negative safe direction (-20.0 RPS)");

    // Disconnected mechanism must cause PitSystemCheckCmd to fail
    MechanismSpec shooterSpec = fig.getMechanismSpec("FUEL", "Shooter");
    MechanismIO disconnectedShooterIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = false;
            inputs.velocityRPS = 0.0;
            inputs.statorCurrentA = 0.0;
          }
        };
    VelocityMechanism disconnectedShooter =
        new VelocityMechanism(disconnectedShooterIo, shooterSpec);
    PitSystemCheckCmd failingCheck =
        new PitSystemCheckCmd(
            container.getDrive(),
            List.of(
                new PositionCheckTarget(container.getTilter(), -2.0, 0.0),
                new PositionCheckTarget(container.getHood(), 2.0, 0.0)),
            List.of(
                new VelocityCheckTarget(container.getIntake(), 20.0),
                new VelocityCheckTarget(container.getIndexer(), 20.0),
                new VelocityCheckTarget(container.getFeeder(), -20.0),
                new VelocityCheckTarget(disconnectedShooter, 25.0)),
            10,
            4,
            container::resetSuperstructureToIdle,
            container.getSuperstructure());
    failingCheck.initialize();
    while (!failingCheck.isFinished()) {
      failingCheck.execute();
      container.getDrive().periodic();
      container.getTilter().periodic();
      container.getHood().periodic();
      container.getIntake().periodic();
      container.getIndexer().periodic();
      container.getFeeder().periodic();
      disconnectedShooter.periodic();
    }
    failingCheck.end(false);
    assertFalse(
        failingCheck.hasPassed(),
        "PitSystemCheckCmd must FAIL when any mechanism is disconnected");
    assertTrue(
        failingCheck.getFailedActuators().stream().anyMatch(s -> s.contains("Shooter")),
        "Failed actuator report must identify Shooter: " + failingCheck.getFailedActuators());
  }

  @Test
  public void testRobotIdentityPrecedenceAndRoboRioSafeFallbackTable() throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-blk07-test");
    try {
      Map<String, RobotIdentity> serialMap = Map.of("03231111", RobotIdentity.CORAL);
      Path rioTag = tmpDir.resolve("rio-id.txt");
      Path scTag = tmpDir.resolve("sc-id.txt");
      Path deployTag = tmpDir.resolve("deploy-id.txt");
      Path devTree = tmpDir.resolve("devtree-serial");

      // Case 1: Hardware serial map wins over conflicting persistent tags and legacy comments
      Files.writeString(rioTag, "FUEL\n");
      Files.writeString(scTag, "PRACTICE_2027\n");
      Files.writeString(deployTag, "COMP_2027\n");
      Files.writeString(devTree, "CHASSIS_3\0");
      assertEquals(
          RobotIdentity.CORAL,
          RobotIdentity.detect("03231111", "FUEL", devTree, rioTag, scTag, deployTag, serialMap));
      assertEquals("HAL_SERIAL_MAP", RobotIdentity.getLastResolutionSource());

      // Case 2: /home/lvuser/robot-identity.txt wins over legacy Comments ("CORAL")
      assertEquals(
          RobotIdentity.FUEL,
          RobotIdentity.detect(
              "03239999", "CORAL", devTree, rioTag, scTag, deployTag, serialMap));
      assertEquals("ROBORIO_PERSISTENT_TAG", RobotIdentity.getLastResolutionSource());
      assertFalse(RobotIdentity.usedFallback());

      // Case 3: SystemCore persistent tag wins over deploy tag, devicetree, and legacy comments
      Files.deleteIfExists(rioTag);
      assertEquals(
          RobotIdentity.PRACTICE_2027,
          RobotIdentity.detect(
              "03239999", "CORAL", devTree, rioTag, scTag, deployTag, serialMap));
      assertEquals("SYSTEMCORE_PERSISTENT_TAG", RobotIdentity.getLastResolutionSource());

      // Case 4: Deploy tag wins over devicetree and legacy comments
      Files.deleteIfExists(scTag);
      assertEquals(
          RobotIdentity.COMP_2027,
          RobotIdentity.detect(
              "03239999", "CORAL", devTree, rioTag, scTag, deployTag, serialMap));
      assertEquals("DEPLOY_TAG", RobotIdentity.getLastResolutionSource());

      // Case 5: Devicetree serial wins over legacy comments
      Files.deleteIfExists(deployTag);
      assertEquals(
          RobotIdentity.CHASSIS_3,
          RobotIdentity.detect(
              "03239999", "CORAL", devTree, rioTag, scTag, deployTag, serialMap));
      assertEquals("DEVICETREE_SERIAL", RobotIdentity.getLastResolutionSource());

      // Case 6: Substring noise and ambiguous comments do NOT hijack identity; fallback is FUEL
      Files.deleteIfExists(devTree);
      assertEquals(
          RobotIdentity.FUEL,
          RobotIdentity.detect(
              "0323BEEF", "REFUELING CORALINE", devTree, rioTag, scTag, deployTag, serialMap));
      assertTrue(RobotIdentity.usedFallback());
      assertEquals("DEFAULT_FUEL", RobotIdentity.getLastResolutionSource());
      assertTrue(RobotIdentity.getLastWarningMessage().contains("0323BEEF"));

      assertEquals(
          RobotIdentity.FUEL,
          RobotIdentity.detect(
              "0323DEAD", "legacy CORAL now FUEL", devTree, rioTag, scTag, deployTag, serialMap));
      assertTrue(RobotIdentity.usedFallback(), "Ambiguous multi-robot comment must not guess");
      assertEquals("DEFAULT_FUEL", RobotIdentity.getLastResolutionSource());
    } finally {
      deleteTree(tmpDir);
    }
  }

  @Test
  public void testHubAlignPoseMirrorsForRedAlliance() {
    CoordinateSystem savedCoord = FieldPositions.getCoordinateSystem();
    try {
      FieldPositions.setCoordinateSystem(CoordinateSystem.WALL_BLUE_2026);
      FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);

      Pose2d bluePose = RobotContainer.getHubAlignTargetPose(false);
      assertEquals(2.5, bluePose.getX(), 1e-6);
      assertEquals(4.1, bluePose.getY(), 1e-6);
      assertEquals(0.0, bluePose.getRotation().getRadians(), 1e-6);

      Pose2d redPose = RobotContainer.getHubAlignTargetPose(true);
      assertEquals(FieldPositions.FIELD_LENGTH_METERS - 2.5, redPose.getX(), 1e-6);
      assertEquals(FieldPositions.FIELD_WIDTH_METERS - 4.1, redPose.getY(), 1e-6);
      assertEquals(Math.PI, Math.abs(redPose.getRotation().getRadians()), 1e-6);

      // Verify DriveToPose on Red alliance drives toward the Red Hub spot, not across to Blue
      FieldPositions.setAllianceOverride(true);
      RobotContainer container =
          new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
      Drive drive = container.getDrive();
      drive.setPose(new Pose2d(13.2, 4.0, Rotation2d.kPi));
      DriveToPose redAlign =
          new DriveToPose(
              drive,
              () -> RobotContainer.getHubAlignTargetPose(FieldPositions.isRedAlliance()),
              () -> true);
      redAlign.initialize();
      for (int i = 0; i < 60 && !redAlign.isFinished(); i++) {
        redAlign.execute();
        drive.periodic();
      }
      redAlign.end(false);
      assertTrue(
          drive.getPose().getX() > 13.5,
          "Red-alliance Hub align must move toward Red pose (14.041, 3.969), got X="
              + drive.getPose().getX());
    } finally {
      FieldPositions.setAllianceOverride(null);
      FieldPositions.setCoordinateSystem(savedCoord);
    }
  }

  @Test
  public void testGhostSuperstructureUsesRealBatteryVoltage() {
    // Physical robot (forceSimIo=false, isReal=true) must use real battery voltage even on CORAL
    assertEquals(
        7.6,
        RobotContainer.selectEffectiveBatteryVoltage(false, true, 7.6, 12.6),
        1e-6,
        "Physical CORAL/CHASSIS_3 robot must feed real RobotController voltage to FaultSupervisor");
    // Desktop sim must continue using VirtualBatterySim voltage
    assertEquals(
        12.4,
        RobotContainer.selectEffectiveBatteryVoltage(true, true, 7.6, 12.4),
        1e-6);
    assertEquals(
        12.4,
        RobotContainer.selectEffectiveBatteryVoltage(false, false, 7.6, 12.4),
        1e-6);

    RobotContainer coral =
        new RobotContainer(RobotIdentity.CORAL, Paths.get("src/main/deploy/configs"), true);
    assertTrue(coral.isGhostSuperstructureActive());
    double realSagV = RobotContainer.selectEffectiveBatteryVoltage(false, true, 7.6, 12.6);
    double throttledStatorA = coral.getFaultSupervisor().updateBatteryProtection(realSagV, 18.0);
    assertTrue(
        throttledStatorA < FaultSupervisor.NOMINAL_DRIVE_STATOR_LIMIT_AMPS,
        "Real 7.6 V sag on physical CORAL must engage brownout stator throttling (got "
            + throttledStatorA
            + " A)");
  }

  @Test
  public void testShootingStopsIntake() {
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    Superstructure ss = container.getSuperstructure();

    ss.requestState(State.INTAKING);
    assertEquals(Superstructure.DEFAULT_INTAKE_IN_RPS, ss.getIntake().getTargetVelocity(), 1e-4);
    assertEquals(Superstructure.DEFAULT_TILTER_DEPLOY_ROT, ss.getTilter().getTargetPosition(), 1e-4);

    ss.shoot(Superstructure.DEFAULT_MID_SHOOTER_RPS, Superstructure.DEFAULT_MID_HOOD_ROT);
    assertEquals(State.SHOOTING, ss.getState());
    assertEquals(
        0.0, ss.getIntake().getTargetVelocity(), 1e-4, "SHOOTING must stop intake from INTAKING");
    assertEquals(
        Superstructure.DEFAULT_TILTER_STOW_ROT,
        ss.getTilter().getTargetPosition(),
        1e-4,
        "SHOOTING must stow tilter");

    ss.requestState(State.OUTTAKING);
    assertEquals(Superstructure.DEFAULT_INTAKE_OUT_RPS, ss.getIntake().getTargetVelocity(), 1e-4);
    ss.shoot(Superstructure.DEFAULT_MID_SHOOTER_RPS, Superstructure.DEFAULT_MID_HOOD_ROT);
    assertEquals(
        0.0, ss.getIntake().getTargetVelocity(), 1e-4, "SHOOTING must stop intake from OUTTAKING");
  }

  @Test
  public void testUnjamTimersTickOncePerLoopWhileShooting() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    FaultSupervisor sup = new FaultSupervisor(3);
    double[] lastIndexerVolts = {0.0};

    MechanismIO stalledIndexerIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.statorCurrentA = 48.0;
            inputs.velocityRPS = 0.0;
          }

          @Override
          public void setVoltage(double volts) {
            lastIndexerVolts[0] = volts;
          }
        };
    MechanismIO readyShooterIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.velocityRPS = Superstructure.DEFAULT_MID_SHOOTER_RPS;
          }
        };
    MechanismIO readyHoodIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.positionRotations = Superstructure.DEFAULT_MID_HOOD_ROT;
          }
        };

    VelocityMechanism intake =
        new VelocityMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Intake")),
            fig.getMechanismSpec("FUEL", "Intake"));
    PositionMechanism tilter =
        new PositionMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Tilter")),
            fig.getMechanismSpec("FUEL", "Tilter"));
    VelocityMechanism indexer =
        new VelocityMechanism(stalledIndexerIo, fig.getMechanismSpec("FUEL", "Indexer"));
    VelocityMechanism feeder =
        new VelocityMechanism(
            new MechanismIOSim(fig.getMechanismSpec("FUEL", "Feeder")),
            fig.getMechanismSpec("FUEL", "Feeder"));
    VelocityMechanism shooter =
        new VelocityMechanism(readyShooterIo, fig.getMechanismSpec("FUEL", "Shooter"));
    PositionMechanism hood =
        new PositionMechanism(readyHoodIo, fig.getMechanismSpec("FUEL", "Hood"), 0.15);

    Superstructure ss = new Superstructure(intake, tilter, indexer, feeder, shooter, hood, sup);
    ShotSolution sol =
        new ShotSolution(
            Rotation2d.kZero,
            2.8,
            Superstructure.DEFAULT_MID_SHOOTER_RPS,
            Superstructure.DEFAULT_MID_HOOD_ROT,
            0.35,
            0.0);

    // Pre-seed a feeder reverse countdown to verify indexerUnjamming does NOT decrement it
    for (int i = 0; i < FaultSupervisor.JAM_DETECT_LOOPS; i++) {
      sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_FEEDER, -8.0, 48.0, 0.0);
    }
    int feederCountdownBeforeIndexerJam =
        sup.getChannelReverseCountdown(FaultSupervisor.JAM_CHANNEL_FEEDER);
    assertEquals(FaultSupervisor.JAM_REVERSE_PULSE_LOOPS, feederCountdownBeforeIndexerJam);
    // Clear feeder countdown for the first phase, then re-seed right before indexer jam triggers
    sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_FEEDER, 0.0, 0.0, 0.0);

    // Call BOTH applyShotSolution AND periodic on every loop (exact teleop/auto shooting pattern)
    for (int loop = 1; loop < FaultSupervisor.JAM_DETECT_LOOPS; loop++) {
      shooter.periodic();
      hood.periodic();
      indexer.periodic();
      feeder.periodic();
      ss.applyShotSolution(sol, true, 0.0, true);
      ss.periodic();
      assertEquals(
          0,
          sup.dejamEventsTriggered(FaultSupervisor.JAM_CHANNEL_INDEXER),
          "Indexer jam must not trigger early on loop " + loop + " (double-tick bug)");
    }

    // Seed an active feeder reverse countdown right before the indexer jam triggers on loop 13
    for (int i = 0; i < FaultSupervisor.JAM_DETECT_LOOPS; i++) {
      sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_FEEDER, -8.0, 48.0, 0.0);
    }
    int seededFeederCountdown = sup.getChannelReverseCountdown(FaultSupervisor.JAM_CHANNEL_FEEDER);

    // Loop JAM_DETECT_LOOPS (13): indexer jam must trigger exactly once
    shooter.periodic();
    hood.periodic();
    indexer.periodic();
    feeder.periodic();
    ss.applyShotSolution(sol, true, 0.0, true);
    ss.periodic();
    assertEquals(1, sup.dejamEventsTriggered(FaultSupervisor.JAM_CHANNEL_INDEXER));
    assertEquals(FaultSupervisor.JAM_REVERSE_VOLTAGE, lastIndexerVolts[0], 1e-6);
    assertEquals(
        seededFeederCountdown,
        sup.getChannelReverseCountdown(FaultSupervisor.JAM_CHANNEL_FEEDER),
        "Feeder unjam countdown must NOT decrement while feeder is forced stopped by indexerUnjamming");

    // Verify unjam pulse lasts the full JAM_REVERSE_PULSE_LOOPS even with applyShotSolution + periodic
    for (int pulseLoop = 0; pulseLoop < FaultSupervisor.JAM_REVERSE_PULSE_LOOPS - 2; pulseLoop++) {
      ss.applyShotSolution(sol, true, 0.0, true);
      ss.periodic();
      assertTrue(
          sup.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_INDEXER),
          "Indexer unjam timer must tick once per loop, not twice (expired at pulseLoop="
              + pulseLoop
              + ")");
      assertEquals(
          seededFeederCountdown,
          sup.getChannelReverseCountdown(FaultSupervisor.JAM_CHANNEL_FEEDER),
          "Feeder countdown must stay paused throughout indexerUnjamming");
    }
  }

  @Test
  public void testDriverResetFieldHeadingPreservesTranslationForBlueAndRed() {
    CoordinateSystem savedCoord = FieldPositions.getCoordinateSystem();
    try {
      FieldPositions.setCoordinateSystem(CoordinateSystem.WALL_BLUE_2026);
      FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);
      RobotContainer container =
          new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
      Drive drive = container.getDrive();

      FieldPositions.setAllianceOverride(false);
      drive.setPose(new Pose2d(3.4, 5.2, Rotation2d.fromDegrees(73.0)));
      container.resetFieldHeading();
      Pose2d afterBlue = drive.getPose();
      assertEquals(3.4, afterBlue.getX(), 1e-6);
      assertEquals(5.2, afterBlue.getY(), 1e-6);
      assertEquals(0.0, afterBlue.getRotation().getRadians(), 1e-6);

      FieldPositions.setAllianceOverride(true);
      drive.setPose(new Pose2d(12.1, 2.8, Rotation2d.fromDegrees(-42.0)));
      container.resetFieldHeading();
      Pose2d afterRed = drive.getPose();
      assertEquals(12.1, afterRed.getX(), 1e-6);
      assertEquals(2.8, afterRed.getY(), 1e-6);
      assertEquals(Math.PI, Math.abs(afterRed.getRotation().getRadians()), 1e-6);
    } finally {
      FieldPositions.setAllianceOverride(null);
      FieldPositions.setCoordinateSystem(savedCoord);
    }
  }

  @Test
  public void testShotCalibrationNeverFeeds() {
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    container.getDrive().setPose(new Pose2d(2.5, 4.035, Rotation2d.kZero));
    ShotCalibrationCmd shotCal = container.createShotCalibrationCommand();
    Superstructure ss = container.getSuperstructure();

    shotCal.initialize();
    while (!shotCal.isFinished()) {
      shotCal.execute();
      container.getShooter().periodic();
      container.getHood().periodic();
      container.getIndexer().periodic();
      container.getFeeder().periodic();
      ss.periodic();
      assertFalse(
          ss.getState() == State.SHOOTING,
          "ShotCalibrationCmd must stay in PREPPING_SHOT and never enter SHOOTING");
      assertEquals(
          0.0, ss.getFeeder().getTargetVelocity(), 1e-4, "ShotCalibrationCmd must never feed");
      assertEquals(
          0.0,
          ss.getIndexer().getTargetVelocity(),
          1e-4,
          "ShotCalibrationCmd must never run indexer");
    }
    shotCal.end(false);
    assertEquals(State.IDLE, ss.getState());
    assertEquals(5, shotCal.getRecordedCsvRows().size());
  }

  @Test
  public void testInternalFlashRetentionCapAndWriteProbe() throws Exception {
    assertEquals(120L * 1024 * 1024, LogFileManager.MAX_INTERNAL_STORAGE_BYTES);
    assertEquals(500L * 1024 * 1024, LogFileManager.MAX_EXTERNAL_STORAGE_BYTES);

    Path tmpDir = Files.createTempDirectory("flux-riolib13-test");
    Path roDir = tmpDir.resolve("ro-usb");
    try {
      Path rwUsb = tmpDir.resolve("rw-usb");
      Path internalLogs = tmpDir.resolve("internal-logs");
      Files.createDirectories(rwUsb);
      Files.createDirectories(roDir);
      Files.createDirectories(internalLogs);

      assertTrue(LogFileManager.probeDirectoryWritable(rwUsb));
      try (var stream = Files.list(rwUsb)) {
        assertTrue(
            stream.noneMatch(p -> p.getFileName().toString().startsWith(".flux-rw-probe")),
            "Write probe must clean up its temporary probe file");
      }

      // Mount listed with 'ro' option must fail isMountedWritableDirectory
      Path roMounts = tmpDir.resolve("ro-mounts");
      Files.writeString(roMounts, "/dev/sda1 " + rwUsb.toRealPath() + " vfat ro,relatime 0 0\n");
      assertFalse(
          LogFileManager.isMountedWritableDirectory(rwUsb, tmpDir, roMounts),
          "Read-only mount option in /proc/mounts must fail mount check");

      // Directory with read-only permissions must fail probe even if /proc/mounts says 'rw'
      Path fakeRwMountsForRoDir = tmpDir.resolve("rw-mounts-ro-dir");
      Files.writeString(
          fakeRwMountsForRoDir, "/dev/sda1 " + roDir.toRealPath() + " vfat rw 0 0\n");
      Files.setPosixFilePermissions(roDir, PosixFilePermissions.fromString("r-xr-xr-x"));
      assertFalse(
          LogFileManager.probeDirectoryWritable(roDir),
          "Read-only directory must fail real temp-file write probe");
      assertFalse(
          LogFileManager.isMountedWritableDirectory(roDir, tmpDir, fakeRwMountsForRoDir),
          "Read-only remounted FAT32 directory must be rejected by write probe");

      // resolveLogPath must fall back to internal logs when USB is read-only
      String fallbackPath =
          LogFileManager.resolveLogPath(
              roDir, tmpDir, fakeRwMountsForRoDir, internalLogs.toFile());
      assertEquals(internalLogs.toAbsolutePath().toString(), fallbackPath);

      // Writable mounted USB resolves to USB logs/
      Path rwMounts = tmpDir.resolve("rw-mounts");
      Files.writeString(rwMounts, "/dev/sda1 " + rwUsb.toRealPath() + " vfat rw 0 0\n");
      String usbPath =
          LogFileManager.resolveLogPath(rwUsb, tmpDir, rwMounts, internalLogs.toFile());
      assertEquals(rwUsb.resolve("logs").toAbsolutePath().toString(), usbPath);
    } finally {
      try {
        Files.setPosixFilePermissions(roDir, PosixFilePermissions.fromString("rwxr-xr-x"));
      } catch (Exception ignored) {
      }
      deleteTree(tmpDir);
    }
  }

  @Test
  public void testJamReverseHaltsOnZeroCommand() {
    FaultSupervisor sup = new FaultSupervisor(3);
    double out = 8.0;
    for (int i = 0; i < FaultSupervisor.JAM_DETECT_LOOPS; i++) {
      out = sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_INTAKE, 8.0, 48.0, 0.0);
    }
    assertEquals(FaultSupervisor.JAM_REVERSE_VOLTAGE, out, 1e-6);
    assertTrue(sup.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_INTAKE));

    // Releasing trigger (0.0 V command) must immediately halt reverse unjam and clear countdown
    double zeroOut =
        sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_INTAKE, 0.0, 0.0, 0.0);
    assertEquals(0.0, zeroOut, 1e-6, "Zero command must immediately return 0.0 V");
    assertFalse(
        sup.isChannelUnjamming(FaultSupervisor.JAM_CHANNEL_INTAKE),
        "Zero command must clear active jamReverseCountdowns");
    assertEquals(0, sup.getChannelReverseCountdown(FaultSupervisor.JAM_CHANNEL_INTAKE));

    // Next fresh press must NOT replay a stale reverse pulse
    double freshPressOut =
        sup.filterMechanismCommandVoltage(FaultSupervisor.JAM_CHANNEL_INTAKE, 8.0, 12.0, 35.0);
    assertEquals(
        8.0, freshPressOut, 1e-6, "Next press after release must command requested positive volts");
  }

  @Test
  public void testRobotHeartbeatPacedMonotonicallyAcrossBackwardRtcJump() throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-trayrisk01-test");
    Path hbFile = tmpDir.resolve("flux-robot-state");
    try (Robot robot = new Robot(hbFile)) {
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();

      long initialWallMs = 1_795_000_000_000L;
      robot.writeRobotStateHeartbeat(true, 10.0, initialWallMs);
      String first = Files.readString(hbFile);
      assertTrue(first.contains("EPOCH_MS=" + initialWallMs), "Initial heartbeat writes wall ms");

      // Within 0.25 s FPGA interval: throttled even if wall clock changed
      robot.writeRobotStateHeartbeat(false, 10.10, initialWallMs + 100L);
      assertEquals(first, Files.readString(hbFile), "Heartbeat throttles inside 0.25 s FPGA window");

      // Simulate a 10-minute backward RTC step once >= 0.25 s of monotonic FPGA time elapsed
      long jumpedBackWallMs = initialWallMs - 600_000L;
      robot.writeRobotStateHeartbeat(false, 10.30, jumpedBackWallMs);
      String updated = Files.readString(hbFile);
      assertTrue(
          updated.contains("EPOCH_MS=" + jumpedBackWallMs),
          "Backward RTC jump must NOT freeze /tmp/flux-robot-state heartbeat updates: " + updated);
    } finally {
      deleteTree(tmpDir);
    }
  }

  @Test
  public void testPitSystemCheckDualMotorShooterPerMotorLimitAndInvertedDriveFail() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    MechanismSpec shooterSpec = fig.getMechanismSpec("FUEL", "Shooter");
    assertTrue(shooterSpec.followerMotorId() >= 0, "FUEL Shooter must have a follower motor");

    // Healthy dual-Kraken Shooter: 55 A leader + 55 A follower = 110 A total (<= 60 A per motor)
    MechanismIO healthyDualShooterIo =
        new MechanismIO() {
          private double targetRps = 0.0;

          @Override
          public void setVelocity(double velocityRPS) {
            targetRps = velocityRPS;
          }

          @Override
          public void stop() {
            targetRps = 0.0;
          }

          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.velocityRPS = targetRps;
            inputs.leaderStatorCurrentA = targetRps != 0.0 ? 55.0 : 0.0;
            inputs.followerStatorCurrentA = targetRps != 0.0 ? 55.0 : 0.0;
            inputs.statorCurrentA = inputs.leaderStatorCurrentA + inputs.followerStatorCurrentA;
          }
        };
    VelocityMechanism healthyDualShooter =
        new VelocityMechanism(healthyDualShooterIo, shooterSpec);
    PitSystemCheckCmd dualPassCheck =
        new PitSystemCheckCmd(
            container.getDrive(),
            List.of(),
            List.of(new VelocityCheckTarget(healthyDualShooter, 25.0)),
            10,
            4,
            () -> {});
    dualPassCheck.initialize();
    while (!dualPassCheck.isFinished()) {
      dualPassCheck.execute();
      container.getDrive().periodic();
      healthyDualShooter.periodic();
    }
    dualPassCheck.end(false);
    assertTrue(
        dualPassCheck.hasPassed(),
        "Dual-Kraken Shooter at 55 A/motor (110 A total) must PASS PitSystemCheckCmd: "
            + dualPassCheck.getSummary());

    // Overcurrent dual-Kraken Shooter: 63 A leader + 40 A follower (> 60 A on leader must FAIL)
    MechanismIO overcurrentDualShooterIo =
        new MechanismIO() {
          private double targetRps = 0.0;

          @Override
          public void setVelocity(double velocityRPS) {
            targetRps = velocityRPS;
          }

          @Override
          public void stop() {
            targetRps = 0.0;
          }

          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.velocityRPS = targetRps;
            inputs.leaderStatorCurrentA = targetRps != 0.0 ? 63.0 : 0.0;
            inputs.followerStatorCurrentA = targetRps != 0.0 ? 40.0 : 0.0;
            inputs.statorCurrentA = inputs.leaderStatorCurrentA + inputs.followerStatorCurrentA;
          }
        };
    VelocityMechanism overcurrentShooter =
        new VelocityMechanism(overcurrentDualShooterIo, shooterSpec);
    PitSystemCheckCmd dualFailCheck =
        new PitSystemCheckCmd(
            container.getDrive(),
            List.of(),
            List.of(new VelocityCheckTarget(overcurrentShooter, 25.0)),
            10,
            4,
            () -> {});
    dualFailCheck.initialize();
    while (!dualFailCheck.isFinished()) {
      dualFailCheck.execute();
      container.getDrive().periodic();
      overcurrentShooter.periodic();
    }
    dualFailCheck.end(false);
    assertFalse(
        dualFailCheck.hasPassed(),
        "Dual-Kraken Shooter with leader at 63 A (> 60 A) must FAIL PitSystemCheckCmd");
    assertTrue(
        dualFailCheck.getFailedActuators().stream()
            .anyMatch(s -> s.contains("Shooter") && s.contains("overcurrent")),
        "Failure report must cite Shooter overcurrent: " + dualFailCheck.getFailedActuators());

    // Inverted swerve corner (spins negative when commanded positive) must FAIL PitSystemCheckCmd
    SwerveConfig cfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    ModuleIO invertedFlIo =
        new ModuleIO() {
          private double cmdVelRadPerSec = 0.0;
          private Rotation2d turnAngle = Rotation2d.kZero;

          @Override
          public void setDriveVelocity(double velocityRadPerSec) {
            cmdVelRadPerSec = velocityRadPerSec;
          }

          @Override
          public void setTurnPosition(Rotation2d angle) {
            turnAngle = angle;
          }

          @Override
          public void stop() {
            cmdVelRadPerSec = 0.0;
          }

          @Override
          public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.turnPosition = turnAngle;
            inputs.driveVelocityRadPerSec = -cmdVelRadPerSec; // Inverted drive polarity
            inputs.driveStatorCurrentAmps = Math.abs(cmdVelRadPerSec) > 0.1 ? 12.0 : 0.0;
            inputs.turnStatorCurrentAmps = 2.0;
            inputs.odometryTimestamps = new double[] {0.02};
            inputs.odometryDrivePositionsRad = new double[] {0.0};
            inputs.odometryTurnPositions = new Rotation2d[] {turnAngle};
          }
        };
    Drive invertedDrive =
        new Drive(
            cfg,
            new GyroIO() {},
            invertedFlIo,
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg));
    PitSystemCheckCmd invertedDriveCheck =
        new PitSystemCheckCmd(invertedDrive, List.of(), List.of(), 10, 4, () -> {});
    invertedDriveCheck.initialize();
    while (!invertedDriveCheck.isFinished()) {
      invertedDriveCheck.execute();
      invertedDrive.periodic();
    }
    invertedDriveCheck.end(false);
    assertFalse(
        invertedDriveCheck.hasPassed(),
        "Inverted swerve drive motor must FAIL PitSystemCheckCmd (never pass via Math.abs)");
    assertTrue(
        invertedDriveCheck.getFailedActuators().stream()
            .anyMatch(s -> s.contains("DriveFL") && s.contains("inverted-direction")),
        "Failure report must flag DriveFL inverted-direction: "
            + invertedDriveCheck.getFailedActuators());
  }

  @Test
  public void testDriveSetPoseNeverSnapsBackUnderQueued250HzYawSamplesAndOverridesPreMatch() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig cfg = SwerveConfig.fromFluxFig(fig, "FUEL");

    // Simulate a Pigeon 2 whose 250 Hz queue continues streaming pre-reset 73 deg yaw samples
    Rotation2d[] hardwareYaw = {Rotation2d.fromDegrees(73.0)};
    double[] sampleTs = {1.000};
    GyroIO queuedPigeonIo =
        new GyroIO() {
          @Override
          public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.yawPosition = hardwareYaw[0];
            double t = sampleTs[0];
            sampleTs[0] += 0.020;
            inputs.odometryYawTimestamps =
                new double[] {t, t + 0.004, t + 0.008, t + 0.012, t + 0.016};
            inputs.odometryYawPositions =
                new Rotation2d[] {
                  hardwareYaw[0], hardwareYaw[0], hardwareYaw[0], hardwareYaw[0], hardwareYaw[0]
                };
          }
        };

    Drive drive =
        new Drive(
            cfg,
            queuedPigeonIo,
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg));

    // Establish initial 73 deg raw gyro state on the pose estimator
    drive.periodic();
    assertEquals(73.0, drive.getPose().getRotation().getDegrees(), 0.5);

    // Driver presses Y (reset heading to 0 deg): queued 73 deg samples must NOT snap heading back
    drive.setPose(new Pose2d(3.4, 5.2, Rotation2d.kZero));
    assertEquals(0.0, drive.getPose().getRotation().getDegrees(), 1e-4);

    for (int tick = 0; tick < 5; tick++) {
      drive.periodic();
      assertEquals(
          0.0,
          drive.getPose().getRotation().getDegrees(),
          0.5,
          "Queued 250 Hz raw yaw samples must never snap heading back after Drive.setPose()");
    }

    // Physical +5 deg rotation (73 -> 78 deg raw) must advance estimated heading to +5 deg
    hardwareYaw[0] = Rotation2d.fromDegrees(78.0);
    drive.periodic();
    assertEquals(5.0, drive.getPose().getRotation().getDegrees(), 0.5);

    // Explicit Disabled Y heading reset must prevent PreMatchCalibrator from overwriting it
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    FieldPositions.setAllianceOverride(false);
    try {
      container.getDrive().setPose(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0)));
      container.resetFieldHeading();
      assertTrue(container.getPreMatchCalibrator().isManualHeadingOverrideActive());
      assertEquals(0.0, container.getDrive().getPose().getRotation().getDegrees(), 1e-4);

      for (int i = 0; i < PreMatchCalibrator.REQUIRED_SAMPLES + 10; i++) {
        container
            .getPreMatchCalibrator()
            .addObservation(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(120.0)));
      }
      assertEquals(
          0.0,
          container.getDrive().getPose().getRotation().getDegrees(),
          1e-4,
          "PreMatchCalibrator must not overwrite an explicit manual Y heading reset");
    } finally {
      FieldPositions.setAllianceOverride(null);
    }
  }

  @Test
  public void testVisionPhotonVisionAmbiguityRejectionTargetsUsedAvgDistStdDevFloorAndSingleTagYaw() {
    List<TargetCorner> corners =
        List.of(
            new TargetCorner(0.0, 0.0),
            new TargetCorner(10.0, 0.0),
            new TargetCorner(10.0, 10.0),
            new TargetCorner(0.0, 10.0));

    // 1) VisionIOPhotonVision.buildPoseObservation rejects -1.0 single-tag ambiguity sentinel
    PhotonTrackedTarget invalidAmbTag =
        new PhotonTrackedTarget(
            0.0,
            0.0,
            1.0,
            0.0,
            7,
            -1,
            0.0f,
            new Transform3d(new Translation3d(2.0, 0.0, 0.0), Rotation3d.kZero),
            new Transform3d(),
            -1.0,
            corners,
            corners);
    EstimatedRobotPose invalidSingleEst =
        new EstimatedRobotPose(
            new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero),
            1.5,
            List.of(invalidAmbTag),
            PoseStrategy.LOWEST_AMBIGUITY);
    assertTrue(
        VisionIOPhotonVision.buildPoseObservation(invalidSingleEst).isEmpty(),
        "Single-tag PhotonVision observation with -1.0 ambiguity must be rejected");

    // 2) VisionIOPhotonVision.buildPoseObservation averages distance strictly over est.targetsUsed
    PhotonTrackedTarget tagA =
        new PhotonTrackedTarget(
            0.0,
            0.0,
            1.0,
            0.0,
            7,
            -1,
            0.0f,
            new Transform3d(new Translation3d(2.0, 0.0, 0.0), Rotation3d.kZero),
            new Transform3d(),
            0.08,
            corners,
            corners);
    PhotonTrackedTarget tagB =
        new PhotonTrackedTarget(
            0.0,
            0.0,
            1.0,
            0.0,
            8,
            -1,
            0.0f,
            new Transform3d(new Translation3d(4.0, 0.0, 0.0), Rotation3d.kZero),
            new Transform3d(),
            0.12,
            corners,
            corners);
    EstimatedRobotPose multiTagEst =
        new EstimatedRobotPose(
            new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero),
            2.0,
            List.of(tagA, tagB),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    Optional<PoseObservation> multiObsOpt = VisionIOPhotonVision.buildPoseObservation(multiTagEst);
    assertTrue(multiObsOpt.isPresent());
    assertEquals(2, multiObsOpt.get().tagCount());
    assertEquals(3.0, multiObsOpt.get().avgTagDistMeters(), 1e-6);
    assertEquals(0.0, multiObsOpt.get().ambiguity(), 1e-6);

    // 3) Vision.periodic() rejects -1.0 ambiguity, floors 0.0 dist std-dev, and sets single-tag
    //    PHOTONVISION angularStdDev = +Infinity while keeping multi-tag angularStdDev finite
    CameraSpec camSpec = new CameraSpec("FUEL", "OV9281-2", 0, 0.2, 0.0, 0.5, 0.0, -10.0, 0.0, 1.0);
    double nowSec = Timer.getFPGATimestamp();
    List<PoseObservation> pendingObs = new ArrayList<>();
    VisionIO stubIo =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.framesDrainedThisCycle = pendingObs.size();
            inputs.poseObservations = pendingObs.toArray(new PoseObservation[0]);
            inputs.tagIds = new int[] {7};
          }
        };
    List<edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1>>
        acceptedStdDevs = new ArrayList<>();
    Vision vision =
        new Vision(
            (pose, ts, stdDevs) -> acceptedStdDevs.add(stdDevs),
            new FaultSupervisor(1),
            new CameraSpec[] {camSpec},
            stubIo);

    // Negative ambiguity must be rejected in Vision even if injected directly
    pendingObs.add(
        new PoseObservation(
            nowSec,
            new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero),
            -1.0,
            1,
            2.0,
            ObservationType.PHOTONVISION));
    vision.periodic();
    assertEquals(0, acceptedStdDevs.size(), "Vision must reject ambiguity < 0.0");

    // Single-tag PHOTONVISION at 0.0 m distance: linearStdDev floored, angularStdDev == +Infinity
    pendingObs.clear();
    pendingObs.add(
        new PoseObservation(
            nowSec,
            new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero),
            0.05,
            1,
            0.0,
            ObservationType.PHOTONVISION));
    vision.periodic();
    assertEquals(1, acceptedStdDevs.size());
    assertTrue(
        acceptedStdDevs.get(0).get(0, 0) >= Vision.MIN_LINEAR_STD_DEV_METERS,
        "Zero-distance vision observation must floor linearStdDev (got "
            + acceptedStdDevs.get(0).get(0, 0)
            + ")");
    assertTrue(
        Double.isInfinite(acceptedStdDevs.get(0).get(2, 0)),
        "Single-tag PHOTONVISION observation must set angularStdDev = +Infinity");

    // Multi-tag PHOTONVISION keeps finite angularStdDev >= MIN_ANGULAR_STD_DEV_RAD
    pendingObs.clear();
    pendingObs.add(
        new PoseObservation(
            nowSec,
            new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero),
            0.0,
            2,
            2.5,
            ObservationType.PHOTONVISION));
    vision.periodic();
    assertEquals(2, acceptedStdDevs.size());
    assertTrue(
        Double.isFinite(acceptedStdDevs.get(1).get(2, 0))
            && acceptedStdDevs.get(1).get(2, 0) >= Vision.MIN_ANGULAR_STD_DEV_RAD,
        "Multi-tag PHOTONVISION must keep finite floored angularStdDev");
  }

  private static void deleteTree(Path root) {
    if (root == null || !Files.exists(root)) {
      return;
    }
    try (var walk = Files.walk(root)) {
      walk.sorted(java.util.Comparator.reverseOrder())
          .forEach(
              p -> {
                try {
                  Files.deleteIfExists(p);
                } catch (Exception ignored) {
                }
              });
    } catch (Exception ignored) {
    }
  }
}
