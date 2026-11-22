package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.drive.Drive;
import frc.lib.drive.GyroIO;
import frc.lib.drive.GyroIO.GyroIOInputs;
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIO.ModuleIOInputs;
import frc.lib.drive.SwerveConfig;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.lib.safety.BatteryIO;
import frc.lib.safety.BatteryIO.BatteryIOInputs;
import frc.lib.safety.BatteryMonitor;
import frc.lib.safety.FaultSupervisor;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIO.VisionIOInputs;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

/**
 * Tier 5 Verification Gate: Deterministic AdvantageKit {@link WPILOGReader} {@code Mode.REPLAY}
 * Verification against a real recorded {@code Flux-RoboRIO-2027} {@code .wpilog} (or real CT
 * {@code CHASSIS_3} log via {@code -Dflux.replayFixture=<path>}).
 *
 * <p>Replays the logged {@code Drive/Gyro}, {@code Drive/Module0..3}, and {@code Battery} inputs
 * through production {@link Drive}, {@link BatteryMonitor}, and {@link FaultSupervisor} with zero
 * hardware or sim plants, asserting {@code < 1 cm} pose fidelity across every logged loop.
 */
class Tier5NytrReplayGateTest {

  static final Path DEFAULT_REPLAY_FIXTURE =
      Paths.get("src/test/resources/replay/roborio-2027-sim-drive.wpilog");

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0), "WPILib HAL must initialize for Tier 5 replay");
  }

  static Path resolveReplayFixture() {
    String override = System.getProperty("flux.replayFixture");
    if (override != null && !override.isBlank()) {
      return Paths.get(override.trim());
    }
    return DEFAULT_REPLAY_FIXTURE;
  }

  @Test
  void testDeterministicWpilogReaderReplayMatchesRecordedDrivePoseWithin1Cm() throws Exception {
    Path fixturePath = resolveReplayFixture();
    assertTrue(
        Files.isRegularFile(fixturePath),
        "Real recorded AdvantageKit .wpilog replay fixture must exist: "
            + fixturePath.toAbsolutePath());

    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    FluxWpiLogCodec.ParsedLog rawLog = FluxWpiLogCodec.read(fixturePath);
    String loggedProfile = "FUEL";
    List<FluxWpiLogCodec.StringSample> idMeta =
        rawLog.getStringSeries("/RealMetadata/RobotIdentity");
    if (idMeta.isEmpty()) {
      idMeta = rawLog.getStringSeries("/Metadata/RobotIdentity");
    }
    if (!idMeta.isEmpty() && !idMeta.get(0).value().isBlank()) {
      loggedProfile = idMeta.get(0).value().trim();
    }
    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fig, loggedProfile);

    // Pure replay IO holders populated directly from WPILOGReader LogTable frames.
    GyroIOInputs[] currentGyro = {new GyroIOInputs()};
    ModuleIOInputs[] currentMods = {
      new ModuleIOInputs(), new ModuleIOInputs(), new ModuleIOInputs(), new ModuleIOInputs()
    };
    BatteryIOInputs[] currentBat = {new BatteryIOInputs()};

    GyroIO replayGyroIo =
        new GyroIO() {
          @Override
          public void updateInputs(GyroIOInputs inputs) {
            copyGyroInputs(currentGyro[0], inputs);
          }
        };
    ModuleIO[] replayModuleIos = new ModuleIO[4];
    for (int i = 0; i < 4; i++) {
      final int idx = i;
      replayModuleIos[i] =
          new ModuleIO() {
            @Override
            public void updateInputs(ModuleIOInputs inputs) {
              copyModuleInputs(currentMods[idx], inputs);
            }
          };
    }
    BatteryIO replayBatteryIo =
        new BatteryIO() {
          @Override
          public void updateInputs(BatteryIOInputs inputs) {
            copyBatteryInputs(currentBat[0], inputs);
          }
        };

    CameraSpec[] camConfigs = fig.getCamerasForRobot(loggedProfile).toArray(new CameraSpec[0]);
    VisionIOInputs[] currentCams = new VisionIOInputs[camConfigs.length];
    VisionIO[] replayVisionIos = new VisionIO[camConfigs.length];
    for (int c = 0; c < camConfigs.length; c++) {
      final int camIdx = c;
      currentCams[c] = new VisionIOInputs();
      replayVisionIos[c] =
          new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs inputs) {
              copyVisionInputs(currentCams[camIdx], inputs);
            }
          };
    }

    Drive replayDrive = new Drive(swerveConfig, replayGyroIo, replayModuleIos);
    BatteryMonitor replayBattery = new BatteryMonitor(replayBatteryIo, fig);
    FaultSupervisor replaySupervisor = new FaultSupervisor(Math.max(1, camConfigs.length));
    Vision replayVision =
        new Vision(replayDrive::addVisionMeasurement, replaySupervisor, camConfigs, replayVisionIos);

    WPILOGReader reader = new WPILOGReader(fixturePath.toAbsolutePath().toString());
    LogTable table = new LogTable(0);
    reader.start();
    int framesReplayed = 0;
    int comparedPoses = 0;
    double maxTranslationErrM = 0.0;
    double maxHeadingErrDeg = 0.0;
    double maxTravelMeters = 0.0;
    double minReplayVBat = 13.0;
    int replayDropouts = 0;
    boolean seededInitialPose = false;

    try {
      while (reader.updateTable(table)) {
        framesReplayed++;
        double frameTimeSec = table.getTimestamp() / 1_000_000.0;
        currentGyro[0].fromLog(table.getSubtable("Drive/Gyro"));
        for (int i = 0; i < 4; i++) {
          currentMods[i].fromLog(table.getSubtable("Drive/Module" + i));
        }
        for (int c = 0; c < camConfigs.length; c++) {
          currentCams[c].fromLog(table.getSubtable("Vision/Camera" + c));
        }
        currentBat[0].fromLog(table.getSubtable("Battery"));

        Pose2d recordedPose =
            (table.get("RealOutputs/Drive/Pose") != null)
                ? table.get("RealOutputs/Drive/Pose", Pose2d.kZero)
                : null;
        if (!seededInitialPose && recordedPose != null) {
          replayDrive.setPose(recordedPose);
          seededInitialPose = true;
        }

        // Exact CommandScheduler order: Drive.periodic() logs Drive/Pose, then Vision.periodic()
        // fuses camera frames for subsequent cycles, then RobotContainer.periodicUpdate() runs.
        replayDrive.periodic();
        if (recordedPose != null) {
          Pose2d replayedPose = replayDrive.getPose();
          double transErr = replayedPose.getTranslation().getDistance(recordedPose.getTranslation());
          double headingErr =
              Math.abs(replayedPose.getRotation().minus(recordedPose.getRotation()).getDegrees());
          maxTranslationErrM = Math.max(maxTranslationErrM, transErr);
          maxHeadingErrDeg = Math.max(maxHeadingErrDeg, headingErr);
          maxTravelMeters = Math.max(maxTravelMeters, recordedPose.getTranslation().getNorm());
          comparedPoses++;
        }

        replayVision.periodicAtTimestamp(frameTimeSec);
        replayVision.recordMotionSample(
            replayDrive.getChassisSpeeds().omegaRadiansPerSecond,
            replayDrive.getMaxTiltRad(),
            frameTimeSec);
        replayBattery.periodic();
        double vBat = currentBat[0].voltageVolts;
        if (Double.isFinite(vBat) && vBat > 0.0) {
          minReplayVBat = Math.min(minReplayVBat, vBat);
          if (vBat < 6.80) {
            replayDropouts++;
          }
          replaySupervisor.updateBatteryProtection(
              vBat,
              replayBattery.getInternalResistanceMilliOhms(),
              replayDrive.getLoopPeriodSeconds());
          replayDrive.applyBatteryProtection(replaySupervisor);
        }
      }
    } finally {
      reader.end();
    }

    PrecisionReporter.recordLowerIsBetter(
        5,
        "tier5_replay_max_pose_err_m",
        "Mode.REPLAY Max Pose Error",
        maxTranslationErrM,
        0.001,
        0.010,
        "m");
    PrecisionReporter.recordLowerIsBetter(
        5,
        "tier5_nytr_replay_dropouts",
        "Real .wpilog Replay Enable Dropouts",
        replayDropouts,
        0.0,
        0.0,
        "dropouts");
    PrecisionReporter.recordHigherIsBetter(
        5,
        "tier5_nytr_replay_min_vbat",
        "Real .wpilog Replay Min Battery V",
        minReplayVBat,
        10.0,
        9.00,
        "V");

    assertTrue(framesReplayed >= 50, "Expected >= 50 replayed .wpilog frames, got " + framesReplayed);
    assertTrue(comparedPoses >= 50, "Expected >= 50 Drive/Pose comparisons, got " + comparedPoses);
    assertTrue(
        maxTravelMeters > 0.50,
        "Recorded replay log must contain real driving motion (> 0.50 m, actual="
            + maxTravelMeters
            + " m)");
    assertFalse(
        replayDrive.getOdometrySampleCount() == 0,
        "Replay must integrate real odometry samples from .wpilog");
    assertTrue(
        maxTranslationErrM < 0.010,
        "Mode.REPLAY Drive/Pose must match recorded pose within < 1 cm (actual="
            + String.format("%.4f", maxTranslationErrM)
            + " m)");
    assertTrue(
        maxHeadingErrDeg < 0.5,
        "Mode.REPLAY Drive heading must match recorded pose within < 0.5 deg (actual="
            + String.format("%.4f", maxHeadingErrDeg)
            + " deg)");
    assertEquals(0, replayDropouts, "Recorded run replay must have 0 brownout dropouts");
  }

  private static void copyGyroInputs(GyroIOInputs src, GyroIOInputs dst) {
    dst.connected = src.connected;
    dst.yawPosition = src.yawPosition;
    dst.yawVelocityRadPerSec = src.yawVelocityRadPerSec;
    dst.rollRad = src.rollRad;
    dst.pitchRad = src.pitchRad;
    dst.supplyVoltageV = src.supplyVoltageV;
    dst.usingOnboardFallback = src.usingOnboardFallback;
    dst.odometryYawTimestamps = src.odometryYawTimestamps.clone();
    dst.odometryYawPositions = src.odometryYawPositions.clone();
  }

  private static void copyModuleInputs(ModuleIOInputs src, ModuleIOInputs dst) {
    dst.driveConnected = src.driveConnected;
    dst.drivePositionRad = src.drivePositionRad;
    dst.driveVelocityRadPerSec = src.driveVelocityRadPerSec;
    dst.driveAppliedVolts = src.driveAppliedVolts;
    dst.driveStatorCurrentAmps = src.driveStatorCurrentAmps;
    dst.driveSupplyCurrentAmps = src.driveSupplyCurrentAmps;
    dst.turnConnected = src.turnConnected;
    dst.usingFusedCancoder = src.usingFusedCancoder;
    dst.turnAbsolutePosition = src.turnAbsolutePosition;
    dst.turnPosition = src.turnPosition;
    dst.turnVelocityRadPerSec = src.turnVelocityRadPerSec;
    dst.turnAppliedVolts = src.turnAppliedVolts;
    dst.turnStatorCurrentAmps = src.turnStatorCurrentAmps;
    dst.turnSupplyCurrentAmps = src.turnSupplyCurrentAmps;
    dst.odometryTimestamps = src.odometryTimestamps.clone();
    dst.odometryDrivePositionsRad = src.odometryDrivePositionsRad.clone();
    dst.odometryTurnPositions = src.odometryTurnPositions.clone();
  }

  private static void copyBatteryInputs(BatteryIOInputs src, BatteryIOInputs dst) {
    dst.connected = src.connected;
    dst.rawSerialOrCode = src.rawSerialOrCode;
    dst.voltageVolts = src.voltageVolts;
    dst.currentAmps = src.currentAmps;
    dst.stateOfChargePct = src.stateOfChargePct;
    dst.stateOfHealthPct = src.stateOfHealthPct;
    dst.internalResistanceMilliOhms = src.internalResistanceMilliOhms;
    dst.consumedChargeMah = src.consumedChargeMah;
    dst.minVoltageVolts = src.minVoltageVolts;
    dst.peakCurrentAmps = src.peakCurrentAmps;
    dst.temperatureCelsius = src.temperatureCelsius;
    dst.dodWattHours = src.dodWattHours;
    dst.batteryAgeSeconds = src.batteryAgeSeconds;
    dst.cycleCount = src.cycleCount;
    dst.chargeState = src.chargeState;
    dst.framesReceived = src.framesReceived;
  }

  private static void copyVisionInputs(VisionIOInputs src, VisionIOInputs dst) {
    dst.connected = src.connected;
    dst.poseObservations = src.poseObservations.clone();
    dst.tagObservations = src.tagObservations.clone();
    dst.tagIds = src.tagIds.clone();
    dst.framesDrainedThisCycle = src.framesDrainedThisCycle;
  }
}
