package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.lib.safety.BatteryIOSim;
import frc.lib.safety.FaultSupervisor;
import frc.lib.vision.VisionIOSim;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.Superstructure.State;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Tier 4 Verification Gate: Loop-Counted 150-Second Full Match Simulation stepping real {@code
 * ModuleIOSim}, {@code GyroIOSim}, {@code MechanismIOSim}, {@code VisionIOSim}, and {@code
 * VirtualBatterySim} draws (zero {@code SimHooks.stepTiming} global HAL clock mutation) with
 * 5-Scenario {@link FaultSupervisor} Chaos Fault Injection and Binary {@code .wpilog} Recording.
 */
class Tier4ChaosMatchSimTest {

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void testFull150SecChaosMatchFastForwardZeroBrownouts() throws Exception {
    // Isolate from earlier JUnit suites that may leave DriverStationSim in Autonomous Enabled,
    // which would otherwise trigger Vision's 2.0 s wall-clock AUTO_START_VISION_IGNORE_SEC window.
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    container.getDrive().setPose(new Pose2d(10.47, 4.035, Rotation2d.kZero));
    double dtSec = container.getDrive().getLoopPeriodSeconds();
    FaultSupervisor supervisor = container.getFaultSupervisor();
    Path wpilogPath = Path.of("build/chaos-match-sim.wpilog");

    // Pre-match disabled calibration phase: 60 loops of stationary MegaTag1 observations
    for (int d = 0; d < 60; d++) {
      container.getVision().periodic();
      container.disabledPeriodic();
    }
    assertTrue(
        container.getPreMatchCalibrator().isGyroSeeded(),
        "PreMatchCalibrator must seed gyro during 60 disabled loops before match start");
    container.resetMechanismEncoders();

    // Configure degraded 28 mOhm competition battery on the actual VirtualBatterySim + BatteryIOSim
    double weakBatteryRintOhms = 0.028;
    double openCircuitVoltage = 12.4;
    container.getVirtualBatterySim().setBatteryParameters(openCircuitVoltage, weakBatteryRintOhms);
    if (container.getBatteryIo() instanceof BatteryIOSim simBat) {
      simBat.simulateReading("FLUX-BAT-2026-B01", openCircuitVoltage, 85.0, 82.0, 28.0);
    }

    int quarantineDetectedLoop = -1;
    int bentCamInjectLoop = 1100; // t = 22.0s
    int totalLoops = (int) Math.round(150.0 / dtSec);
    String diagAtInject = "";

    try (FluxWpiLogCodec.Writer log =
        new FluxWpiLogCodec.Writer(wpilogPath, "FLUX-2027-Tier4-ChaosMatch")) {
      int vBatEntry = log.startEntry("/Robot/BatteryVoltageV", "double", "V", 0);
      int statorLimitEntry =
          log.startEntry("/FaultSupervisor/ActiveDriveStatorLimitA", "double", "A", 0);
      int cam1QuarantinedEntry =
          log.startEntry("/FaultSupervisor/Cam1Quarantined", "boolean", "", 0);
      int dejamCountEntry = log.startEntry("/FaultSupervisor/DejamEvents", "double", "count", 0);

      for (int loop = 0; loop < totalLoops; loop++) {
        long tsUs = Math.round(loop * dtSec * 1_000_000.0);
        double timeSec = loop * dtSec;

        if (loop == bentCamInjectLoop) {
          ((VisionIOSim) container.getVision().getCameraIo(1))
              .setMountErrorPerturbation(
                  new Transform3d(new Translation3d(0.12, 0.0, 0.0), new Rotation3d()));
        }

        boolean jamActive = (loop >= 2400 && loop <= 2420) || (loop >= 4800 && loop <= 4820);
        double indexerCurrentA = jamActive ? 45.0 : 12.0;
        double indexerVelRps = jamActive ? 0.10 : 32.0;
        supervisor.filterMechanismCommandVoltage(8.0, indexerCurrentA, indexerVelRps);

        int visibleTags = (loop >= 3500 && loop <= 3650) ? 0 : 2;
        for (int c = 0; c < 3; c++) {
          ((VisionIOSim) container.getVision().getCameraIo(c))
              .setVisibleTagSimulation(visibleTags, 2.6, 0.08);
        }

        double speedScale = supervisor.reportSteerEncoderFault(2, loop >= 5750);
        double demandFactor = 0.65 + 0.35 * Math.abs(Math.sin(timeSec * 1.7)) * speedScale;

        // Step actual RobotContainer subsystems + VirtualBatterySim along a smooth mid-field
        // circle centered at (8.27, 4.035) with bounded centering nudge so the 150s chaos match
        // stays strictly inside field bounds without saturating the degraded 28 mOhm battery.
        double startupRamp = Math.min(1.0, (loop + 1) / 25.0);
        Pose2d currentPose = container.getDrive().getPose();
        double centerNudgeX = MathUtil.clamp((8.27 - currentPose.getX()) * 0.25, -0.35, 0.35);
        double centerNudgeY = MathUtil.clamp((4.035 - currentPose.getY()) * 0.25, -0.35, 0.35);
        double vxField = (-1.4 * Math.sin(timeSec * 0.7) + centerNudgeX) * demandFactor * startupRamp;
        double vyField = (1.4 * Math.cos(timeSec * 0.7) + centerNudgeY) * demandFactor * startupRamp;
        container
            .getDrive()
            .runFieldRelative(new ChassisSpeeds(vxField, vyField, 0.70 * startupRamp), true, 0.0);
        container
            .getSuperstructure()
            .requestState((loop % 200 < 100) ? State.INTAKING : State.IDLE);
        container.getDrive().periodic();
        container.getVision().periodic();
        if (quarantineDetectedLoop < 0 && supervisor.isCameraQuarantined(1)) {
          quarantineDetectedLoop = loop;
        }
        if (loop == bentCamInjectLoop + 4) {
          diagAtInject =
              String.format(
                  "pose=%s omega=%.3f accepted=%d rejected=%d motionRej=%d visEn=%s q=%s",
                  container.getDrive().getPose(),
                  container.getDrive().getChassisSpeeds().omegaRadiansPerSecond,
                  container.getVision().getAcceptedObservationCount(),
                  container.getVision().getRejectedObservationCount(),
                  container.getVision().getRejectedByMotionGateCount(),
                  container.getVision().isVisionEnabled(),
                  supervisor.isCameraQuarantined(1));
        }
        container.getIntake().periodic();
        container.getTilter().periodic();
        container.getIndexer().periodic();
        container.getFeeder().periodic();
        container.getShooter().periodic();
        container.getHood().periodic();
        container.getSuperstructure().periodic();
        container.periodicUpdate();

        if (loop % 10 == 0) {
          log.appendDouble(vBatEntry, tsUs, container.getVirtualBatterySim().terminalVoltageV());
          log.appendDouble(statorLimitEntry, tsUs, supervisor.activeDriveStatorLimitAmps());
          log.appendBoolean(cam1QuarantinedEntry, tsUs, supervisor.isCameraQuarantined(1));
          log.appendDouble(dejamCountEntry, tsUs, supervisor.dejamEventsTriggered());
        }
      }
    }

    double minBatteryVoltageV = container.getVirtualBatterySim().minVoltageV();
    int brownoutCount = container.getVirtualBatterySim().brownoutCount();
    double peakSupplyAmps = container.getVirtualBatterySim().peakSupplyCurrentAmps();
    double quarantineLatencyMs = (quarantineDetectedLoop - bentCamInjectLoop) * (dtSec * 1000.0);
    FluxWpiLogCodec.ParsedLog parsed = FluxWpiLogCodec.read(wpilogPath);

    PrecisionReporter.recordHigherIsBetter(
        4,
        "tier4_chaos_min_vbat",
        "150s Chaos Match Min Battery V",
        minBatteryVoltageV,
        9.50,
        7.50,
        "V");
    PrecisionReporter.recordLowerIsBetter(
        4,
        "tier4_cam_quarantine_ms",
        "Bent Cam (+12cm) Quarantine Latency",
        quarantineLatencyMs,
        80.0,
        200.0,
        "ms");
    PrecisionReporter.recordExactTarget(
        4,
        "tier4_indexer_dejam_pulses",
        "Indexer Stall Jam Auto-Clears",
        supervisor.dejamEventsTriggered(),
        2.0,
        0.1,
        "pulses");

    assertTrue(
        peakSupplyAmps > 10.0,
        "Tier 4 must exercise real ModuleIOSim + MechanismIOSim supply current draws (peak="
            + peakSupplyAmps
            + " A)");
    assertTrue(
        container.getDrive().isGyroConnected()
            && Math.abs(container.getDrive().getUnwrappedGyroYawRad()) > 5.0,
        "Real GyroIOSim must integrate continuous chassis rotation across the 150s match");
    assertEquals(0, brownoutCount, "Zero battery brownouts (<6.8V) allowed during 150s chaos match");
    assertTrue(
        minBatteryVoltageV >= 7.50,
        "VirtualBatterySim voltage must stay >= 7.50V on 28mOhm battery (min="
            + minBatteryVoltageV
            + " V)");
    assertTrue(supervisor.isCameraQuarantined(1), "Bent camera #1 (+12cm) must be quarantined");
    assertEquals(
        80.0,
        quarantineLatencyMs,
        25.0,
        "Quarantine must trigger in 5 frames (~80-100ms) [" + diagAtInject + "]");
    assertEquals(2, supervisor.dejamEventsTriggered(), "Both injected indexer jams must auto-clear");
    assertTrue(Files.size(wpilogPath) > 10_000, "Generated .wpilog must contain full match trace");
    assertTrue(parsed.durationSeconds() >= 149.0, "Parsed .wpilog duration must span ~150s match");
  }
}
