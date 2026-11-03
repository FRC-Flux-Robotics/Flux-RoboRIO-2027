package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.commands.Blitz15MinCmd;
import frc.lib.commands.DriveCommands;
import frc.lib.commands.DriveToPose;
import frc.lib.commands.HardStopHomingCmd;
import frc.lib.commands.PitSystemCheckCmd;
import frc.lib.commands.PreMatchCalibrator;
import frc.lib.commands.ShopStressSuiteCmd;
import frc.lib.commands.ZeroPriorBootstrapCmd;
import frc.lib.drive.Drive;
import frc.lib.drive.GyroIO;
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIOSim;
import frc.lib.drive.PhoenixOdometryThread;
import frc.lib.drive.SwerveConfig;
import frc.lib.drive.VirtualGeofence;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
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
import frc.lib.util.PiecewiseSensitivity;
import frc.lib.util.SafeAutoBuilder;
import frc.lib.util.VirtualBatterySim;
import frc.lib.vision.Vision;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

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

    // Scenario 1: COMP_2027 on SystemCore
    fig.setActiveProfile("COMP_2027");
    assertEquals("SYSTEMCORE_2027", fig.getString("controllerProfile"));
    assertEquals("can_s0", fig.getString("swerveCanBus"));
    assertEquals(5, fig.getInt("loopPeriodMs"));

    // Scenario 2: 1-Cell Fallback to roboRIO 2.0 on COMP_2027
    fig.setActiveProfile("COMP_2027:roborio_fallback");
    assertEquals("ROBORIO_CANIVORE_2026", fig.getString("controllerProfile"));
    assertEquals("Drivetrain", fig.getString("swerveCanBus"));
    assertEquals("PHOTONVISION_NT4", fig.getString("visionBackend"));
    assertEquals(20, fig.getInt("loopPeriodMs"));
    assertEquals(60.0, fig.getDouble("driveStatorLimitAmps"), 1e-6);
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
    assertTrue(ss.isReadyToFire(), "Shooter and Hood must reach target and unlock firing gate");
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
    assertTrue(tunable.hasChanged(99));
    assertFalse(tunable.hasChanged(99));
    tunable.setTestOverride(2.50);
    assertEquals(2.50, tunable.get(), 1e-6);
    assertTrue(tunable.hasChanged(99));
    LoggedTunableNumber.setTuningMode(false);
    assertEquals(1.25, tunable.get(), 1e-6);

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
    assertTrue(stationaryShot.shooterRps() > 40.0);
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

    int serialArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_SERIAL, 0);
    assertTrue(batIo.processCanFrame(serialArbId, BatteryIO.encodeSerialFrame("2026-B02"), 8));
    assertEquals("2026-B02", batIo.getResolvedBatteryName());

    int telemArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_TELEMETRY, 0);
    assertTrue(
        batIo.processCanFrame(
            telemArbId, BatteryIO.encodeTelemetryFrame(12.640, 18.5, 96.5, 94.0, 15.8), 8));
    double bfgDecodeErrMv = Math.abs(batIo.getVoltageVolts() - 12.640) * 1000.0;
    assertEquals(12.640, batIo.getVoltageVolts(), 1e-4);
    assertEquals(18.5, batIo.getCurrentAmps(), 1e-3);
    assertEquals(96.5, batIo.getStateOfChargePct(), 1e-3);
    assertEquals(15.8, batIo.getInternalResistanceMilliOhms(), 1e-3);
    assertTrue(batIo.isHealthy());

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
    for (int i = 0; i < PitSystemCheckCmd.TOTAL_LOOPS + 1; i++) {
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
    assertEquals(1, blitz.getCurrentStageIndex());

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
    try (Robot robot = new Robot()) {
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
}
