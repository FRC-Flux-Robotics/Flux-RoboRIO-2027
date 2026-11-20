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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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

      // Unknown real hex serial must warn loudly on fallback instead of staying silent
      assertEquals(
          RobotIdentity.COMP_2027,
          RobotIdentity.detect("0323CAFE", "", null, null, null, null, serialMap));
      assertTrue(RobotIdentity.usedFallback(), "Unknown hex serial must flag fallback");
      assertTrue(
          RobotIdentity.getLastWarningMessage().contains("0323CAFE"),
          "Fallback warning must name the unrecognized hex serial");

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
}
