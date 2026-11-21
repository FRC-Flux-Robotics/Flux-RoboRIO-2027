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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.lib.util.Nt4FluxFigTuningBackend;
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
    fuelContainer.resetMechanismEncoders();
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
    hood.resetPosition();
    tilter.resetPosition();

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
        new HardStopHomingCmd(fuelContainer.getHood(), -2.0, 0.0, 50.0, 2, 20);
    homingCmd.initialize();
    for (int i = 0; i < 15 && !homingCmd.isFinished(); i++) {
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

    fuelContainer.resetMechanismEncoders();
    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    Path checkpointFile = Path.of("build/test-blitz-progress.json");
    Files.deleteIfExists(checkpointFile);
    PitSystemCheckCmd pitCheck = fuelContainer.createPitSystemCheckCommand();
    HardStopHomingCmd blitzHoming =
        new HardStopHomingCmd(fuelContainer.getHood(), -2.0, 0.0, 50.0, 2, 20);
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
    fuelContainer.resetMechanismEncoders();
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
      opSim.setPOV(-1);
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
      CommandScheduler.getInstance().getDefaultButtonLoop().clear();
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
    container.resetMechanismEncoders();

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
    container.resetMechanismEncoders();
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
    tilter.resetPosition();
    hood.resetPosition();

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

  @Test
  public void testManualShortShotFallbackEncoderZeroAndRotationCeiling() {
    Path configDir = Paths.get("src/main/deploy/configs");
    FluxFig fuelCfg = FluxFig.load(configDir, RobotIdentity.FUEL.name());
    FluxFig coralCfg = FluxFig.load(configDir, RobotIdentity.CORAL.name());
    assertEquals(
        4.71,
        fuelCfg.getDouble("maxAngularSpeedRadPerSec"),
        1e-6,
        "FUEL maxAngularSpeedRadPerSec must stay capped at safe 0.75 rot/s (4.71 rad/s)");
    assertEquals(
        4.71,
        coralCfg.getDouble("maxAngularSpeedRadPerSec"),
        1e-6,
        "CORAL maxAngularSpeedRadPerSec must stay capped at safe 0.75 rot/s (4.71 rad/s)");

    RobotContainer container = new RobotContainer(RobotIdentity.FUEL, configDir, true);
    container.resetMechanismEncoders();
    container.getDrive().setPose(Pose2d.kZero);

    // Manual short shot must feed once shooter + hood reach tolerance even when Pose=(0,0) is out of Hub range.
    Command manualShot = container.createManualShortShotCommand();
    manualShot.initialize();
    for (int i = 0; i < 120; i++) {
      manualShot.execute();
      container.getIntake().periodic();
      container.getTilter().periodic();
      container.getIndexer().periodic();
      container.getFeeder().periodic();
      container.getShooter().periodic();
      container.getHood().periodic();
      container.getSuperstructure().periodic();
    }
    assertEquals(Superstructure.State.SHOOTING, container.getSuperstructure().getState());
    assertTrue(
        container.getSuperstructure().isReadyToFire(),
        "Manual short shot must reach ReadyToFire without AprilTag localization");
    assertEquals(0.0, container.getIntake().getTargetVelocity(), 1e-6);
    assertEquals(
        Superstructure.DEFAULT_FEEDER_RPS, container.getFeeder().getTargetVelocity(), 1e-6);
    manualShot.end(false);
    container.resetSuperstructureToIdle();
    assertEquals(Superstructure.State.IDLE, container.getSuperstructure().getState());

    // Zeroing relative encoders at stow resets both Tilter and Hood cleanly to 0.0.
    container.getTilter().setPosition(-5.0);
    container.getHood().setPosition(3.0);
    for (int i = 0; i < 30; i++) {
      container.getTilter().periodic();
      container.getHood().periodic();
    }
    assertTrue(Math.abs(container.getTilter().getPosition()) > 0.5);
    assertTrue(Math.abs(container.getHood().getPosition()) > 0.5);
    container.resetMechanismEncoders();
    container.getTilter().periodic();
    container.getHood().periodic();
    assertEquals(0.0, container.getTilter().getPosition(), 1e-4);
    assertEquals(0.0, container.getHood().getPosition(), 1e-4);
  }

  @Test
  public void testPositionMechanismStaysNeutralUntilZeroConfirmed() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec tilterSpec = fig.getMechanismSpec("FUEL", "Tilter");
    double[] posRot = {-2.0};
    int[] positionCmdCount = {0};
    int[] stopCount = {0};
    double[] lastPositionCmd = {Double.NaN};
    MechanismIO spyTilterIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.positionRotations = posRot[0];
          }

          @Override
          public void setPosition(double positionRotations) {
            positionCmdCount[0]++;
            lastPositionCmd[0] = positionRotations;
          }

          @Override
          public void stop() {
            stopCount[0]++;
          }

          @Override
          public boolean resetPosition() {
            posRot[0] = 0.0;
            return true;
          }
        };

    PositionMechanism tilter = new PositionMechanism(spyTilterIo, tilterSpec);
    tilter.periodic();
    assertFalse(tilter.isZeroConfirmed(), "ZeroConfirmed must start false on boot");
    assertEquals(-2.0, tilter.getPosition(), 1e-6);

    // Enabling and commanding stow (0.0) or deploy (-17.0) must keep motor neutral until re-zeroed
    tilter.setPosition(0.0);
    tilter.setPosition(-17.0);
    assertEquals(
        0,
        positionCmdCount[0],
        "Un-zeroed PositionMechanism must send zero position commands to motor");
    assertTrue(stopCount[0] >= 2, "Un-zeroed setPosition() must keep motor neutral via stop()");
    assertFalse(tilter.atTarget(), "Un-zeroed PositionMechanism must never report atTarget()");

    // After resetPosition(), zero is confirmed and position commands execute normally
    assertTrue(tilter.resetPosition());
    tilter.periodic();
    assertTrue(tilter.isZeroConfirmed(), "ZeroConfirmed must become true after resetPosition()");
    assertEquals(0.0, tilter.getPosition(), 1e-6);

    tilter.setPosition(-5.0);
    assertEquals(1, positionCmdCount[0], "Zeroed PositionMechanism must forward setPosition()");
    assertEquals(-5.0, lastPositionCmd[0], 1e-6);
  }

  @Test
  public void testTeleopOperatorBackReZeroAndDpadManualJog() throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-cmp-c03-test");
    String prevLogDir = System.getProperty("flux.logDir");
    System.setProperty("flux.logDir", tmpDir.resolve("logs").toString());
    try (Robot robot = new Robot(tmpDir.resolve("flux-robot-state"))) {
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();

      robot.robotInit();
      RobotContainer container = robot.getRobotContainer();
      assertFalse(container.getTilter().isZeroConfirmed());
      assertFalse(container.getHood().isZeroConfirmed());

      // Enter Teleop (NOT Test mode)
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      robot.teleopInit();
      robot.robotPeriodic();

      XboxControllerSim opSim = new XboxControllerSim(1);
      opSim.setPOV(-1);
      opSim.setBackButton(false);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();

      // Operator Back re-zero must work in Teleop
      opSim.setBackButton(true);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      assertTrue(
          container.getTilter().isZeroConfirmed(),
          "Operator Back must re-zero Tilter in Teleop (not restricted to Test mode)");
      assertTrue(
          container.getHood().isZeroConfirmed(),
          "Operator Back must re-zero Hood in Teleop (not restricted to Test mode)");

      opSim.setBackButton(false);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();

      // Operator D-pad Up jogs Hood positive in Teleop and stops on release
      opSim.setPOV(0);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      for (int i = 0; i < 8; i++) {
        robot.robotPeriodic();
      }
      assertTrue(
          container.getHood().getPosition() > 0.05,
          "Operator POV Up must jog Hood positively in Teleop (pos="
              + container.getHood().getPosition()
              + ")");

      opSim.setPOV(-1);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      robot.robotPeriodic();
      assertEquals(
          0.0,
          container.getHood().getAppliedVoltage(),
          1e-4,
          "Releasing Operator POV must immediately stop Hood jog voltage");

      // Operator D-pad Left jogs Tilter negative (deploy direction) in Teleop and stops on release
      opSim.setPOV(270);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      for (int i = 0; i < 8; i++) {
        robot.robotPeriodic();
      }
      assertTrue(
          container.getTilter().getPosition() < -0.05,
          "Operator POV Left must jog Tilter negatively in Teleop (pos="
              + container.getTilter().getPosition()
              + ")");

      opSim.setPOV(-1);
      opSim.notifyNewData();
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      robot.robotPeriodic();
      assertEquals(
          0.0,
          container.getTilter().getAppliedVoltage(),
          1e-4,
          "Releasing Operator POV must immediately stop Tilter jog voltage");
    } finally {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().getDefaultButtonLoop().clear();
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
  public void testVisionEnableToggleStopsConsumerWhileUpdatingInputs() {
    CameraSpec camSpec = new CameraSpec("FUEL", "OV9281-2", 0, 0.2, 0.0, 0.5, 0.0, -10.0, 0.0, 1.0);
    int[] updateCalls = {0};
    int[] consumerCalls = {0};
    VisionIO stubIo =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            updateCalls[0]++;
            inputs.connected = true;
            inputs.framesDrainedThisCycle = 1;
            inputs.tagIds = new int[] {7, 8};
            inputs.poseObservations =
                new PoseObservation[] {
                  new PoseObservation(
                      Timer.getFPGATimestamp(),
                      new Pose3d(3.0, 4.0, 0.0, Rotation3d.kZero),
                      0.0,
                      2,
                      2.0,
                      ObservationType.MEGATAG_1)
                };
          }
        };

    Vision vision =
        new Vision(
            (pose, ts, stdDevs) -> consumerCalls[0]++,
            new FaultSupervisor(1),
            new CameraSpec[] {camSpec},
            stubIo);
    assertTrue(vision.isVisionEnabled(), "Vision must default to enabled (true)");

    vision.periodic();
    assertEquals(1, updateCalls[0]);
    assertEquals(1, consumerCalls[0]);

    // Switch vision off: camera inputs must still update, consumer gets ZERO further calls
    vision.setVisionEnabled(false);
    assertFalse(vision.isVisionEnabled());
    for (int i = 0; i < 5; i++) {
      vision.periodic();
    }
    assertEquals(6, updateCalls[0], "Camera VisionIO.updateInputs must keep updating when disabled");
    assertEquals(6, vision.getTotalFramesDrained(), "Camera frames must keep logging when disabled");
    assertEquals(
        1,
        consumerCalls[0],
        "Vision consumer must receive ZERO further calls after Vision/Enabled is turned off");
    assertTrue(
        vision.getLatestYawTrustedPose().isEmpty(),
        "Disabled vision must not expose poses to PreMatchCalibrator");

    // Re-enable for cleanup
    vision.setVisionEnabled(true);
  }

  @Test
  public void testSafeDefaultAutoDoesNothingAndPreservesNonOriginPose() {
    Path configDir = Paths.get("src/main/deploy/configs");
    RobotContainer container = new RobotContainer(RobotIdentity.FUEL, configDir, true);
    Drive drive = container.getDrive();

    Pose2d startPose = new Pose2d(5.25, 3.10, Rotation2d.fromDegrees(42.0));
    drive.setPose(startPose);

    Command defaultAuto = container.getAutonomousCommand();
    assertEquals(RobotContainer.SAFE_DEFAULT_AUTO_NAME, defaultAuto.getName());

    defaultAuto.initialize();
    for (int i = 0; i < 20; i++) {
      if (!defaultAuto.isFinished()) {
        defaultAuto.execute();
      }
      drive.periodic();
    }
    defaultAuto.end(false);

    Pose2d afterPose = drive.getPose();
    assertEquals(startPose.getX(), afterPose.getX(), 1e-4, "Safe default auto must not reset X");
    assertEquals(startPose.getY(), afterPose.getY(), 1e-4, "Safe default auto must not reset Y");
    assertEquals(
        startPose.getRotation().getDegrees(),
        afterPose.getRotation().getDegrees(),
        1e-3,
        "Safe default auto must not reset heading");
    assertEquals(
        0.0,
        Math.hypot(
            drive.getChassisSpeeds().vxMetersPerSecond,
            drive.getChassisSpeeds().vyMetersPerSecond),
        1e-4,
        "Safe default auto must command zero linear drive output");
    assertEquals(
        0.0,
        drive.getChassisSpeeds().omegaRadiansPerSecond,
        1e-4,
        "Safe default auto must command zero angular drive output");

    // Empty renderAuto selector still resolves to the student starter routine
    Command renderDefault = container.resolveAutonomousCommand("");
    assertEquals(StudentAutoRoutines.DEFAULT_STUDENT_AUTO_NAME, renderDefault.getName());
  }

  @Test
  public void testRealRobotWeakBatteryRintStillDerates() {
    Path configDir = Paths.get("src/main/deploy/configs");
    RobotContainer container = new RobotContainer(RobotIdentity.FUEL, configDir, true);
    BatteryIO batIo = container.getBatteryIo();

    // Absent gauge: falls back to nominal batteries.csv R_int (~15.875 mOhm)
    assertFalse(batIo.isConnected());
    double absentRint =
        RobotContainer.selectEffectiveRintMilliOhms(
            batIo.isConnected(),
            batIo.getInternalResistanceMilliOhms(),
            batIo.getNominalRintMilliOhms());
    assertTrue(
        absentRint >= 15.0 && absentRint <= 17.0,
        "Absent BFG must use nominal batteries.csv R_int (got " + absentRint + " mOhm)");

    // Connected weak battery (> 22 mOhm, < 75% SoC -> isHealthy() == false): MUST still derate!
    batIo.simulateCanReading("FLUX-BAT-2026-B01", 12.1, 55.0, 70.0, 30.0);
    assertTrue(batIo.isConnected());
    assertFalse(batIo.isHealthy(), "Weak battery (30 mOhm, 55% SoC) must report unhealthy");

    double weakRint =
        RobotContainer.selectEffectiveRintMilliOhms(
            batIo.isConnected(),
            batIo.getInternalResistanceMilliOhms(),
            batIo.getNominalRintMilliOhms());
    assertEquals(
        30.0,
        weakRint,
        1e-6,
        "Connected weak battery must feed its real 30 mOhm R_int even when isHealthy() is false");

    double statorLimitA = container.getFaultSupervisor().updateBatteryProtection(12.1, weakRint);
    container.getDrive().applyBatteryProtection(container.getFaultSupervisor());
    assertTrue(
        statorLimitA < FaultSupervisor.NOMINAL_DRIVE_STATOR_LIMIT_AMPS,
        "Weak 30 mOhm battery must derate drive stator limit below 60 A (got " + statorLimitA + ")");
    assertTrue(
        container.getDrive().getBrownoutDemandScale() < 1.0,
        "Weak 30 mOhm battery must derate Drive/BrownoutDemandScale below 1.0 (got "
            + container.getDrive().getBrownoutDemandScale()
            + ")");
  }

  @Test
  public void testBatteryVoltageStepHysteresisChangesScaleAtMostOncePerSecond() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig cfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    Drive drive =
        new Drive(
            cfg,
            new GyroIO() {},
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg));
    FaultSupervisor sup = new FaultSupervisor(3);

    // Step voltage across 9.6 V every 0.20 s (10 loops) over 3.0 seconds (150 loops @ 20 ms)
    int scaleChanges = 0;
    double prevScale = drive.getBrownoutDemandScale();
    for (int loop = 0; loop < 150; loop++) {
      double vBat = ((loop / 10) % 2 == 0) ? 9.1 : 10.6;
      sup.updateBatteryProtection(vBat, 16.0, 0.020);
      drive.applyBatteryProtection(sup);
      double scale = drive.getBrownoutDemandScale();
      if (Math.abs(scale - prevScale) > 1e-6) {
        scaleChanges++;
        prevScale = scale;
      }
    }
    assertTrue(
        scaleChanges >= 1 && scaleChanges <= 3,
        "Stepping battery voltage across 9.6 V over 3.0 s must change scale at most once/sec (got "
            + scaleChanges
            + " changes)");

    // Shallow jitter between 9.4 V and 9.8 V stays inside hysteresis band (9.6..10.2 V) with no surge
    sup.reset();
    int hysteresisChanges = 0;
    double lastScale = 1.0;
    for (int loop = 0; loop < 150; loop++) {
      double vBat = ((loop / 5) % 2 == 0) ? 9.3 : 9.8;
      sup.updateBatteryProtection(vBat, 16.0, 0.020);
      drive.applyBatteryProtection(sup);
      double scale = drive.getBrownoutDemandScale();
      if (Math.abs(scale - lastScale) > 1e-6) {
        hysteresisChanges++;
        lastScale = scale;
      }
    }
    assertTrue(
        hysteresisChanges <= 3,
        "Jitter across 9.6 V inside hysteresis band must stay damped <= 1/sec (got "
            + hysteresisChanges
            + ")");
  }

  @Test
  public void testManualShortShotUses2026ShortValues() {
    assertEquals(33.33, Superstructure.DEFAULT_SHORT_SHOOTER_RPS, 1e-6);
    assertEquals(0.0, Superstructure.DEFAULT_SHORT_HOOD_ROT, 1e-6);
    assertEquals(12.5, Superstructure.HOOD_MAX_ROT, 1e-6);

    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    container.resetMechanismEncoders();
    Command manualShort = container.createManualShortShotCommand();
    manualShort.initialize();
    manualShort.execute();
    assertEquals(
        33.33,
        container.getShooter().getTargetVelocity(),
        1e-4,
        "Manual short shot must target 33.33 RPS");
    assertEquals(
        0.0,
        container.getHood().getTargetPosition(),
        1e-4,
        "Manual short shot must target 0.0 hood rotations");
    manualShort.end(false);
  }

  @Test
  public void testThreeCameraConsensusQuarantinesOnlyBentCameraAndRecovers() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    CameraSpec[] cams = fig.getCamerasForRobot("FUEL").toArray(new CameraSpec[0]);
    assertEquals(3, cams.length);

    FaultSupervisor sup = new FaultSupervisor(3);
    Pose2d truePose = new Pose2d(4.0, 4.0, Rotation2d.kZero);
    VisionIOSim[] ios = new VisionIOSim[3];
    for (int i = 0; i < 3; i++) {
      ios[i] = new VisionIOSim(cams[i], () -> truePose);
      ios[i].setFramesPerCycle(1);
    }
    Vision vision = new Vision((pose, ts, std) -> {}, sup, cams, ios);

    // Offset camera #1 by +20 cm: ONLY camera #1 may be quarantined; healthy cameras #0 & #2 stay clear
    ios[1].setMountErrorPerturbation(
        new Transform3d(new Translation3d(0.20, 0.0, 0.0), Rotation3d.kZero));
    for (int loop = 0; loop < FaultSupervisor.CAMERA_OUTLIER_CONSECUTIVE_FRAMES + 3; loop++) {
      vision.periodic();
    }
    assertTrue(sup.isCameraQuarantined(1), "Bent camera #1 (+20 cm) must be quarantined");
    assertFalse(
        sup.isCameraQuarantined(0),
        "Healthy camera #0 must NOT be quarantined by single bent camera #1");
    assertFalse(
        sup.isCameraQuarantined(2),
        "Healthy camera #2 must NOT be quarantined by single bent camera #1");

    // Realign camera #1: after consecutive agreeing frames it must un-quarantine cleanly
    ios[1].setMountErrorPerturbation(new Transform3d());
    for (int loop = 0; loop < FaultSupervisor.CAMERA_RECOVERY_CONSECUTIVE_FRAMES + 2; loop++) {
      vision.periodic();
    }
    assertFalse(
        sup.isCameraQuarantined(1),
        "Realigned camera #1 must recover from quarantine after consecutive good frames");
    assertFalse(sup.isCameraQuarantined(0));
    assertFalse(sup.isCameraQuarantined(2));

    // Rejected high-ambiguity frame must never poison latestCameraPose or trigger false quarantine
    VisionIO[] rejectedTestIos = new VisionIO[3];
    for (int i = 0; i < 3; i++) {
      final int camIdx = i;
      rejectedTestIos[i] =
          new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs inputs) {
              inputs.connected = true;
              inputs.framesDrainedThisCycle = 1;
              inputs.tagIds = new int[] {7};
              double ts = Timer.getFPGATimestamp();
              PoseObservation good =
                  new PoseObservation(
                      ts,
                      new Pose3d(4.0, 4.0, 0.0, Rotation3d.kZero),
                      0.05,
                      2,
                      2.0,
                      ObservationType.PHOTONVISION);
              if (camIdx == 1) {
                PoseObservation badAmbiguity =
                    new PoseObservation(
                        ts,
                        new Pose3d(4.50, 4.0, 0.0, Rotation3d.kZero),
                        0.85,
                        1,
                        3.5,
                        ObservationType.PHOTONVISION);
                inputs.poseObservations = new PoseObservation[] {good, badAmbiguity};
              } else {
                inputs.poseObservations = new PoseObservation[] {good};
              }
            }
          };
    }
    FaultSupervisor supFiltered = new FaultSupervisor(3);
    Vision filteredVision = new Vision((pose, ts, std) -> {}, supFiltered, cams, rejectedTestIos);
    for (int loop = 0; loop < 10; loop++) {
      filteredVision.periodic();
    }
    assertFalse(
        supFiltered.isCameraQuarantined(1),
        "Rejected high-ambiguity observation must never overwrite latestCameraPose");
  }

  @Test
  public void testPreMatchCalibratorIgnoresSingleTagPhotonVisionAndBimodalCancellation() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig cfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    Drive drive =
        new Drive(
            cfg,
            new GyroIO() {},
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg));
    drive.setPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));

    CameraSpec camSpec = fig.getCamerasForRobot("FUEL").get(0);
    VisionIO singleTagPvIo =
        new VisionIO() {
          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.framesDrainedThisCycle = 1;
            inputs.tagIds = new int[] {7};
            inputs.poseObservations =
                new PoseObservation[] {
                  new PoseObservation(
                      Timer.getFPGATimestamp(),
                      new Pose3d(3.0, 4.0, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(45.0))),
                      0.05,
                      1,
                      2.0,
                      ObservationType.PHOTONVISION)
                };
          }
        };
    Vision vision =
        new Vision(
            drive::addVisionMeasurement,
            new FaultSupervisor(1),
            new CameraSpec[] {camSpec},
            singleTagPvIo);
    PreMatchCalibrator calibrator = new PreMatchCalibrator(drive, vision);

    // Feed 60 Disabled loops of single-tag PHOTONVISION with 45 deg yaw error: heading must NOT shift
    for (int i = 0; i < PreMatchCalibrator.REQUIRED_SAMPLES + 10; i++) {
      vision.periodic();
      calibrator.updateDisabled();
    }
    assertFalse(
        calibrator.isGyroSeeded(),
        "Single-tag PHOTONVISION observations must never seed PreMatchCalibrator gyro");
    assertEquals(
        0.0,
        drive.getPose().getRotation().getDegrees(),
        1e-3,
        "Drive heading must not shift from single-tag PHOTONVISION poses while Disabled");

    // Feed 50 alternating 0 deg / 180 deg flips: circular-mean cancellation must be rejected
    PreMatchCalibrator bimodalCal = new PreMatchCalibrator(drive, vision);
    for (int i = 0; i < PreMatchCalibrator.REQUIRED_SAMPLES; i++) {
      Rotation2d flip = (i % 2 == 0) ? Rotation2d.kZero : Rotation2d.kPi;
      bimodalCal.addObservation(new Pose2d(3.0, 4.0, flip));
    }
    assertFalse(
        bimodalCal.isGyroSeeded(),
        "Symmetric 180-deg bimodal yaw flips must be rejected instead of cancelling to 0 deg");
  }

  @Test
  public void testAngularVelocityClampedAcrossDriveAndCommands() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig cfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    double maxOmega = cfg.maxAngularSpeedRadPerSec();
    assertEquals(4.71, maxOmega, 1e-6);

    Drive drive =
        new Drive(
            cfg,
            new GyroIO() {},
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg),
            new ModuleIOSim(cfg));
    drive.setPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));

    // Direct Drive.runVelocity(omega = 9.0) must clamp commanded omega <= 4.71 rad/s
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 9.0));
    assertTrue(
        Math.abs(drive.getLastCommandedSpeeds().omegaRadiansPerSecond) <= maxOmega + 1e-6,
        "Drive.runVelocity must clamp omega <= 4.71 rad/s (got "
            + drive.getLastCommandedSpeeds().omegaRadiansPerSecond
            + ")");
    for (int i = 0; i < 15; i++) {
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 9.0));
      drive.periodic();
      assertTrue(
          Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) <= maxOmega + 0.15,
          "Simulated chassis omega must respect 4.71 rad/s ceiling");
    }

    // DriveCommands.joystickDriveAtAngle with 180 deg error + full 4.5 m/s translation + feedforward
    drive.setPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));
    PiecewiseSensitivity sens =
        new PiecewiseSensitivity("Test/AimCurve", 0.09, 0.60, 0.10, 0.30, 0.80);
    Command aimCmd =
        DriveCommands.joystickDriveAtAngle(
            drive,
            sens,
            () -> 1.0,
            () -> 0.0,
            () -> Rotation2d.kPi,
            () -> 5.0,
            () -> true);
    aimCmd.initialize();
    for (int i = 0; i < 20; i++) {
      aimCmd.execute();
      assertTrue(
          Math.abs(drive.getLastCommandedSpeeds().omegaRadiansPerSecond) <= maxOmega + 1e-6,
          "joystickDriveAtAngle must clamp total omega <= 4.71 rad/s (got "
              + drive.getLastCommandedSpeeds().omegaRadiansPerSecond
              + ")");
      drive.periodic();
    }
    aimCmd.end(false);
  }

  @Test
  public void testHardStopHomingGraceWindowAndFailedResetPosition() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec hoodSpec = fig.getMechanismSpec("FUEL", "Hood");

    // Case A: Tick-1..4 startup inrush spike at rest followed by normal motion must NOT false-zero
    double[] currentA = {24.0};
    double[] velRps = {0.0};
    boolean[] resetSucceeds = {true};
    MechanismIO scriptedIo =
        new MechanismIO() {
          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.statorCurrentA = currentA[0];
            inputs.velocityRPS = velRps[0];
          }

          @Override
          public boolean resetPosition() {
            return resetSucceeds[0];
          }
        };

    PositionMechanism hoodA = new PositionMechanism(scriptedIo, hoodSpec);
    HardStopHomingCmd graceCmd = new HardStopHomingCmd(hoodA);
    graceCmd.initialize();
    // Loops 1..6: heavy startup inrush at zero velocity (within 8-loop breakaway grace window)
    for (int i = 0; i < 6; i++) {
      hoodA.periodic();
      graceCmd.execute();
      assertFalse(
          graceCmd.isFinished(),
          "Tick-" + (i + 1) + " startup inrush at rest must not false-zero mid-travel");
    }
    // Loops 7..15: mechanism breaks away and moves freely (low current, moving)
    currentA[0] = 6.0;
    velRps[0] = -3.0;
    for (int i = 0; i < 9; i++) {
      hoodA.periodic();
      graceCmd.execute();
      assertFalse(graceCmd.isFinished(), "Free-moving mechanism must not finish homing");
    }
    graceCmd.end(true);
    assertFalse(graceCmd.isHomed());
    assertFalse(hoodA.isZeroConfirmed());

    // Case B: Hard stop reached after grace window, but resetPosition() fails -> Homed/ZeroConfirmed stay false
    PositionMechanism hoodB = new PositionMechanism(scriptedIo, hoodSpec);
    resetSucceeds[0] = false;
    currentA[0] = 24.0;
    velRps[0] = 0.0;
    HardStopHomingCmd failedResetCmd = new HardStopHomingCmd(hoodB);
    failedResetCmd.initialize();
    int totalRequiredLoops =
        HardStopHomingCmd.DEFAULT_BREAKAWAY_GRACE_LOOPS
            + HardStopHomingCmd.DEFAULT_REQUIRED_STALL_LOOPS;
    for (int i = 0; i < totalRequiredLoops + 2 && !failedResetCmd.isFinished(); i++) {
      hoodB.periodic();
      failedResetCmd.execute();
    }
    assertTrue(failedResetCmd.isFinished(), "Homing should finish once stall is confirmed");
    failedResetCmd.end(false);
    assertFalse(
        failedResetCmd.isHomed(),
        "HardStopHomingCmd must NOT mark Homed=true when resetPosition() returns false");
    assertFalse(
        hoodB.isZeroConfirmed(),
        "PositionMechanism must NOT mark ZeroConfirmed=true when resetPosition() returns false");
  }

  @Test
  public void testPitSystemCheckFailsInvertedPositionAndReboundingVelocity() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    MechanismSpec tilterSpec = fig.getMechanismSpec("FUEL", "Tilter");
    MechanismSpec hoodSpec = fig.getMechanismSpec("FUEL", "Hood");
    MechanismSpec intakeSpec = fig.getMechanismSpec("FUEL", "Intake");

    // Backwards-moving Tilter & Hood (move opposite to commanded delta) must FAIL PitSystemCheckCmd
    MechanismIO invertedPosIo =
        new MechanismIO() {
          private double pos = 0.0;
          private double vel = 0.0;

          @Override
          public void setPosition(double targetPos) {
            double dir = Math.signum(targetPos - pos);
            vel = -dir * 4.0;
            pos += vel * 0.020;
          }

          @Override
          public void stop() {
            vel = 0.0;
          }

          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.positionRotations = pos;
            inputs.velocityRPS = vel;
            inputs.leaderStatorCurrentA = 8.0;
            inputs.statorCurrentA = 8.0;
          }
        };
    PositionMechanism invertedHood = new PositionMechanism(invertedPosIo, hoodSpec);
    invertedHood.resetPosition();
    MechanismIO invertedTilterIo =
        new MechanismIO() {
          private double pos = 0.0;
          private double vel = 0.0;

          @Override
          public void setPosition(double targetPos) {
            double dir = Math.signum(targetPos - pos);
            vel = -dir * 4.0;
            pos += vel * 0.020;
          }

          @Override
          public void stop() {
            vel = 0.0;
          }

          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.positionRotations = pos;
            inputs.velocityRPS = vel;
            inputs.leaderStatorCurrentA = 8.0;
            inputs.statorCurrentA = 8.0;
          }
        };
    PositionMechanism invertedTilter = new PositionMechanism(invertedTilterIo, tilterSpec);
    invertedTilter.resetPosition();

    // Rebounding inverted velocity mechanism: spins backwards (-15 RPS) then rebounds forwards (+15 RPS)
    MechanismIO reboundingVelIo =
        new MechanismIO() {
          private int activeTick = 0;
          private double currentVel = 0.0;

          @Override
          public void setVelocity(double velocityRPS) {
            activeTick++;
            currentVel = (activeTick <= 5) ? -15.0 : 15.0;
          }

          @Override
          public void stop() {
            currentVel = 0.0;
          }

          @Override
          public void updateInputs(MechanismIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.velocityRPS = currentVel;
            inputs.leaderStatorCurrentA = 10.0;
            inputs.statorCurrentA = 10.0;
          }
        };
    VelocityMechanism reboundingIntake = new VelocityMechanism(reboundingVelIo, intakeSpec);

    PitSystemCheckCmd check =
        new PitSystemCheckCmd(
            container.getDrive(),
            List.of(
                new PositionCheckTarget(invertedTilter, -2.0, 0.0),
                new PositionCheckTarget(invertedHood, 2.0, 0.0)),
            List.of(new VelocityCheckTarget(reboundingIntake, 20.0)),
            10,
            4,
            () -> {});
    check.initialize();
    while (!check.isFinished()) {
      check.execute();
      container.getDrive().periodic();
      invertedTilter.periodic();
      invertedHood.periodic();
      reboundingIntake.periodic();
    }
    check.end(false);

    assertFalse(
        check.hasPassed(),
        "PitSystemCheckCmd must FAIL inverted Tilter/Hood and rebounding inverted velocity mechanism");
    assertTrue(
        check.getFailedActuators().stream()
            .anyMatch(s -> s.contains("Tilter") && s.contains("inverted-direction")),
        "Failed actuators must flag Tilter inverted-direction: " + check.getFailedActuators());
    assertTrue(
        check.getFailedActuators().stream()
            .anyMatch(s -> s.contains("Hood") && s.contains("inverted-direction")),
        "Failed actuators must flag Hood inverted-direction: " + check.getFailedActuators());
    assertTrue(
        check.getFailedActuators().stream()
            .anyMatch(s -> s.contains("Intake") && s.contains("inverted-direction")),
        "Failed actuators must flag rebounding Intake inverted-direction: "
            + check.getFailedActuators());

    // Unzeroed PositionMechanism must fail PitSystemCheckCmd with explicit not-zeroed reason.
    PositionMechanism unzeroedTilter = new PositionMechanism(new MechanismIOSim(tilterSpec), tilterSpec);
    PositionMechanism unzeroedHood = new PositionMechanism(new MechanismIOSim(hoodSpec), hoodSpec);
    PitSystemCheckCmd unzeroedCheck =
        new PitSystemCheckCmd(
            container.getDrive(),
            List.of(
                new PositionCheckTarget(unzeroedTilter, -2.0, 0.0),
                new PositionCheckTarget(unzeroedHood, 2.0, 0.0)),
            List.of(),
            6,
            2,
            () -> {});
    unzeroedCheck.initialize();
    while (!unzeroedCheck.isFinished()) {
      unzeroedCheck.execute();
      container.getDrive().periodic();
      unzeroedTilter.periodic();
      unzeroedHood.periodic();
    }
    unzeroedCheck.end(false);
    assertFalse(
        unzeroedCheck.hasPassed(),
        "PitSystemCheckCmd must fail clearly when Tilter/Hood have not been zeroed since boot");
    assertTrue(
        unzeroedCheck.getFailedActuators().stream()
            .anyMatch(s -> s.contains("Tilter") && s.contains("not-zeroed")),
        "Expected explicit not-zeroed reason for Tilter: " + unzeroedCheck.getFailedActuators());
    assertTrue(
        unzeroedCheck.getFailedActuators().stream()
            .anyMatch(s -> s.contains("Hood") && s.contains("not-zeroed")),
        "Expected explicit not-zeroed reason for Hood: " + unzeroedCheck.getFailedActuators());
  }

  @Test
  public void testM129BareDrivetrainZeroCamerasAutoRateOdometryFourCornerPitCheckAndDefaultSplit()
      throws Exception {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();

    RobotContainer chassis3 =
        new RobotContainer(RobotIdentity.CHASSIS_3, Paths.get("src/main/deploy/configs"), true);
    try {
      assertTrue(chassis3.isGhostSuperstructureActive());
      assertEquals(0, chassis3.getCameraCount(), "CHASSIS_3 has 0 rows in cameras.csv");
      assertEquals(0, chassis3.getVision().getCameraCount(), "Vision must instantiate 0 cameras");

      // Ghost PitSystemCheckCmd on bare chassis runs strictly the 4 swerve corners (~6.8 s)
      // and passes without Port 1 Operator controller or mechanism encoder zeroing.
      PitSystemCheckCmd pitCheck = chassis3.createPitSystemCheckCommand();
      assertEquals(
          4 * (PitSystemCheckCmd.DEFAULT_STEP_LOOPS + PitSystemCheckCmd.DEFAULT_SETTLE_LOOPS),
          pitCheck.getTotalLoops(),
          "Bare-chassis PitSystemCheckCmd must run exactly 4 swerve corner steps");
      pitCheck.initialize();
      while (!pitCheck.isFinished()) {
        pitCheck.execute();
        chassis3.getDrive().periodic();
      }
      pitCheck.end(false);
      assertTrue(
          pitCheck.hasPassed(),
          "Bare-chassis 4-corner PitSystemCheckCmd must pass without Operator Back zeroing: "
              + pitCheck.getSummary());

      // Guarded Driver A (0 cameras) and Driver RT (Ghost chassis) must not fire blind moves or shooting.
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setFmsAttached(false);
      XboxControllerSim driverSim = new XboxControllerSim(0);
      driverSim.setLeftY(0.0);
      driverSim.setLeftX(0.0);
      driverSim.setRightX(0.0);
      driverSim.setAButton(true);
      driverSim.setRightTriggerAxis(1.0);
      driverSim.notifyNewData();
      DriverStationSim.notifyNewData();

      chassis3.getDrive().setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
      for (int i = 0; i < 10; i++) {
        CommandScheduler.getInstance().run();
        chassis3.periodicUpdate();
      }
      assertEquals(
          0.0,
          chassis3.getDrive().getPose().getTranslation().getNorm(),
          1e-4,
          "Driver A must remain inactive on 0-camera CHASSIS_3");
      assertEquals(
          State.IDLE,
          chassis3.getSuperstructure().getState(),
          "Driver RT must remain inactive on bare-drivetrain CHASSIS_3");

      driverSim.setAButton(false);
      driverSim.setRightTriggerAxis(0.0);
      driverSim.notifyNewData();
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().getDefaultButtonLoop().clear();
      chassis3.close();
    }

    // Auto-rate selection: 100 Hz on CAN 2.0 ("rio" / "can_s*") vs 250 Hz on CANivore FD.
    assertEquals(100.0, PhoenixOdometryThread.resolveFrequencyHz("rio", 0.0), 1e-6);
    assertEquals(100.0, PhoenixOdometryThread.resolveFrequencyHz("can_s0", 0.0), 1e-6);
    assertEquals(100.0, PhoenixOdometryThread.resolveFrequencyHz("can_s2", 0.0), 1e-6);
    assertEquals(250.0, PhoenixOdometryThread.resolveFrequencyHz("Drivetrain", 0.0), 1e-6);
    assertEquals(250.0, PhoenixOdometryThread.resolveFrequencyHz("CANdace", 0.0), 1e-6);
    assertEquals(125.0, PhoenixOdometryThread.resolveFrequencyHz("rio", 125.0), 1e-6);

    // Editing DEFAULT behavioral limits updates FUEL, CORAL, and CHASSIS_3 identically while
    // keeping hardware columns robot-specific and rio CAN bus at 100 Hz.
    Path tmpDir = Files.createTempDirectory("flux-m129-default-split-");
    try {
      try (var stream = Files.list(Paths.get("src/main/deploy/configs"))) {
        for (Path src : stream.filter(Files::isRegularFile).toList()) {
          Files.copy(src, tmpDir.resolve(src.getFileName()), StandardCopyOption.REPLACE_EXISTING);
        }
      }
      Path paramsPath = tmpDir.resolve("parameters.csv");
      String paramsCsv = Files.readString(paramsPath);
      paramsCsv =
          paramsCsv
              .replace("maxLinearSpeedMps,4.5,", "maxLinearSpeedMps,3.25,")
              .replace(
                  "swerveCanBus,Drivetrain,can_s0,Drivetrain,can_s0,Drivetrain,,CANdace,Drivetrain",
                  "swerveCanBus,Drivetrain,can_s0,Drivetrain,can_s0,Drivetrain,,CANdace,rio");
      Files.writeString(paramsPath, paramsCsv);

      FluxFig fig = new FluxFig(tmpDir);
      assertEquals(0, fig.validateStrict().size(), "Modified parameters.csv must validate strictly");
      for (String robot : List.of("FUEL", "CORAL", "CHASSIS_3")) {
        fig.setActiveProfile(robot);
        assertEquals(3.25, fig.getDouble("maxLinearSpeedMps"), 1e-6, "DEFAULT must govern " + robot);
        assertEquals(4.71, fig.getDouble("maxAngularSpeedRadPerSec"), 1e-6);
        assertEquals(60.0, fig.getDouble("driveStatorLimitAmps"), 1e-6);
        assertEquals(40.0, fig.getDouble("driveSupplyLimitAmps"), 1e-6);
        assertEquals(7.5, fig.getDouble("brownoutShedThresholdVolts"), 1e-6);
      }
      fig.setActiveProfile("FUEL:snappy");
      assertEquals(4.8, fig.getDouble("maxLinearSpeedMps"), 1e-6, "Overlay must still override DEFAULT");

      SwerveConfig fuelCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
      SwerveConfig chassis3RioCfg = SwerveConfig.fromFluxFig(fig, "CHASSIS_3");
      assertEquals("Drivetrain", fuelCfg.canBus());
      assertEquals(250.0, fuelCfg.odometryFrequencyHz(), 1e-6);
      assertEquals("rio", chassis3RioCfg.canBus());
      assertEquals(100.0, chassis3RioCfg.odometryFrequencyHz(), 1e-6);
    } finally {
      deleteTree(tmpDir);
    }
  }

  @Test
  public void testM130LiveNt4TuningUpdatesDrivetrainWritesTmpSnapshotAndRevertsOnFms()
      throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-m130-nt4-tuning-");
    NetworkTableInstance ntInst = NetworkTableInstance.create();
    Nt4FluxFigTuningBackend ntBackend = new Nt4FluxFigTuningBackend(ntInst);
    RobotContainer container = null;
    try {
      Path cfgDir = tmpDir.resolve("configs");
      Files.createDirectories(cfgDir);
      try (var stream = Files.list(Paths.get("src/main/deploy/configs"))) {
        for (Path src : stream.filter(Files::isRegularFile).toList()) {
          Files.copy(src, cfgDir.resolve(src.getFileName()), StandardCopyOption.REPLACE_EXISTING);
        }
      }
      String originalParamsCsv = Files.readString(cfgDir.resolve("parameters.csv"));
      String originalMechsCsv = Files.readString(cfgDir.resolve("mechanisms.csv"));
      Files.writeString(
          cfgDir.resolve(FluxFig.ACTIVE_PROFILE_FILE),
          FluxFig.formatActiveProfileDirective(
              "CHASSIS_3", FluxFig.Mode.TUNING, "v3", "EXPERIMENTAL", "live-tuning"));

      Path tmpSnapshot = tmpDir.resolve("fluxfig-tuned-snapshot.txt");
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.notifyNewData();

      container =
          new RobotContainer(RobotIdentity.CHASSIS_3, cfgDir, null, true, ntBackend, tmpSnapshot);
      assertEquals("EXPERIMENTAL", container.getBootConfigChannel());
      assertEquals("live-tuning", container.getBootConfigPreset());
      assertEquals(RobotContainer.OperatingMode.TRAINING, container.getOperatingMode());
      assertTrue(container.isLiveTuningBackendEnabled());
      assertEquals(5, ntBackend.getPublishedTopicCount());
      assertEquals(4.5, container.getDrive().getMaxLinearSpeedMps(), 1e-6);
      assertEquals(4.71, container.getDrive().getMaxAngularSpeedRadPerSec(), 1e-6);
      assertEquals(0.80, container.getJoystickSensitivity().getMaxScale(), 1e-6);
      assertEquals(1.00, container.getRotationSensitivity().getMaxScale(), 1e-6);
      assertEquals(0.90, container.getDrive().getRuntimeDriveKp(), 1e-6);

      RobotContainer finalContainer = container;
      assertThrows(
          IllegalArgumentException.class,
          () -> finalContainer.getFluxFig().registerTunableParameters("swerveCanBus"));
      assertThrows(
          IllegalArgumentException.class,
          () -> finalContainer.getFluxFig().registerTunableParameters("driveStatorLimitAmps"));

      try (DoublePublisher linPub =
              ntInst.getDoubleTopic("/FluxFig/Tuning/parameters/maxLinearSpeedMps").publish();
          DoublePublisher angPub =
              ntInst.getDoubleTopic("/FluxFig/Tuning/parameters/maxAngularSpeedRadPerSec").publish();
          DoublePublisher joyPub =
              ntInst.getDoubleTopic("/FluxFig/Tuning/parameters/joystickSensitivity").publish();
          DoublePublisher rotPub =
              ntInst.getDoubleTopic("/FluxFig/Tuning/parameters/rotationSensitivity").publish();
          DoublePublisher kpPub =
              ntInst.getDoubleTopic("/FluxFig/Tuning/mechanisms/SwerveDriveFL/kP").publish()) {
        linPub.set(2.2);
        angPub.set(3.0);
        joyPub.set(0.65);
        rotPub.set(0.85);
        kpPub.set(1.35);
        ntInst.flushLocal();

        container.periodicUpdate();
      }

      assertEquals(2.2, container.getDrive().getMaxLinearSpeedMps(), 1e-6);
      assertEquals(3.0, container.getDrive().getMaxAngularSpeedRadPerSec(), 1e-6);
      assertEquals(0.65, container.getJoystickSensitivity().getMaxScale(), 1e-6);
      assertEquals(0.85, container.getRotationSensitivity().getMaxScale(), 1e-6);
      assertEquals(1.35, container.getDrive().getRuntimeDriveKp(), 1e-6);
      assertTrue(Files.exists(tmpSnapshot), "Live tuning must write /tmp tuned snapshot file");
      String snap = Files.readString(tmpSnapshot);
      assertTrue(snap.contains("CHASSIS_3.maxLinearSpeedMps: 4.5 -> 2.2"), snap);
      assertTrue(snap.contains("mechanisms[CHASSIS_3:SwerveDriveFL].kP: 0.90 -> 1.35"), snap);
      FluxFig parsedSnap = FluxFig.fromEffectiveSnapshot(snap);
      assertEquals(2.2, parsedSnap.getDouble("maxLinearSpeedMps"), 1e-6);
      assertEquals(1.35, parsedSnap.getMechanismSpec("CHASSIS_3", "SwerveDriveFL").kP(), 1e-6);
      assertEquals(
          originalParamsCsv,
          Files.readString(cfgDir.resolve("parameters.csv")),
          "Live tuning must never modify deploy/configs/parameters.csv");
      assertEquals(
          originalMechsCsv,
          Files.readString(cfgDir.resolve("mechanisms.csv")),
          "Live tuning must never modify deploy/configs/mechanisms.csv");

      // FMS lockout immediately switches back to PRODUCTION and reverts all live overrides.
      DriverStationSim.setFmsAttached(true);
      DriverStationSim.notifyNewData();
      container.periodicUpdate();

      assertEquals(RobotContainer.OperatingMode.COMPETITION, container.getOperatingMode());
      assertEquals(FluxFig.Mode.PRODUCTION, container.getFluxFig().getMode());
      assertFalse(container.getFluxFig().isModified());
      assertEquals(4.5, container.getDrive().getMaxLinearSpeedMps(), 1e-6);
      assertEquals(4.71, container.getDrive().getMaxAngularSpeedRadPerSec(), 1e-6);
      assertEquals(0.80, container.getJoystickSensitivity().getMaxScale(), 1e-6);
      assertEquals(1.00, container.getRotationSensitivity().getMaxScale(), 1e-6);
      assertEquals(0.90, container.getDrive().getRuntimeDriveKp(), 1e-6);
    } finally {
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.notifyNewData();
      if (container != null) {
        container.close();
      }
      ntBackend.close();
      ntInst.close();
      deleteTree(tmpDir);
    }
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
