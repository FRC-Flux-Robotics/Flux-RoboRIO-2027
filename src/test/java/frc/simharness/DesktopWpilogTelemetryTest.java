package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.robot.Robot;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Dedicated isolated-JVM verification for M1.3 ({@code ./gradlew verifyWpilog}): executes a real
 * 30-second {@link Robot#startCompetition()} desktop simulation, shuts down cleanly so AdvantageKit
 * flushes {@code WPILOGWriter}, and parses the resulting binary {@code .wpilog} to prove non-empty
 * {@code OdometryTimestamps}, {@code ConfigSHA256}, {@code PowerDistribution}, and motor
 * voltage/velocity/position channels with zero {@code .hoot} files.
 */
public class DesktopWpilogTelemetryTest {

  @BeforeAll
  public static void initHal() {
    assertTrue(HAL.initialize(500, 0), "WPILib HAL must initialize");
  }

  @Test
  public void testThirtySecondDesktopRunWritesCompleteReplayableWpilog() throws Exception {
    int durationSec = Integer.parseInt(System.getProperty("flux.wpilogDurationSec", "30"));
    Path outDir = Paths.get("build/m13-wpilog-verify").toAbsolutePath();
    if (Files.exists(outDir)) {
      try (Stream<Path> walk = Files.walk(outDir)) {
        walk.sorted(Comparator.reverseOrder())
            .forEach(
                p -> {
                  try {
                    Files.deleteIfExists(p);
                  } catch (IOException ignored) {
                  }
                });
      }
    }
    Files.createDirectories(outDir);
    Path stateFile = outDir.resolve("flux-robot-state");

    String prevLogDir = System.getProperty("flux.logDir");
    System.setProperty("flux.logDir", outDir.toString());

    String expectedConfigSha;
    try (Robot robot = new Robot(stateFile)) {
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();

      Throwable[] threadFailure = new Throwable[1];
      Thread robotThread =
          new Thread(
              () -> {
                try {
                  robot.startCompetition();
                } catch (Throwable t) {
                  threadFailure[0] = t;
                }
              },
              "M13-DesktopRobotLoop");
      robotThread.start();

      long waitDeadlineMs = System.currentTimeMillis() + 10_000L;
      while (robot.getRobotContainer() == null && System.currentTimeMillis() < waitDeadlineMs) {
        Thread.sleep(25);
      }
      assertNotNull(robot.getRobotContainer(), "RobotContainer must initialize inside startCompetition");
      expectedConfigSha = robot.getRobotContainer().getBootConfigSha256();

      // Enable Teleop and drive via XboxControllerSim so all Drive/Superstructure/Logger calls run
      // strictly on the robot loop thread (zero cross-thread StructBuffer races).
      XboxControllerSim driverSim = new XboxControllerSim(0);
      driverSim.setLeftY(-0.35);
      driverSim.setRightX(-0.15);
      driverSim.setRightTriggerAxis(1.0);
      driverSim.notifyNewData();
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      long endAtMs = System.currentTimeMillis() + durationSec * 1000L;
      boolean sawEnabledHeartbeat = false;
      while (System.currentTimeMillis() < endAtMs) {
        driverSim.notifyNewData();
        if (!sawEnabledHeartbeat && Files.exists(stateFile)) {
          String txt = Files.readString(stateFile, StandardCharsets.UTF_8);
          if (txt.contains("STATE=ENABLED") && txt.contains("ENABLED=true")) {
            sawEnabledHeartbeat = true;
          }
        }
        Thread.sleep(100);
      }
      assertTrue(sawEnabledHeartbeat, "Atomic robot state file must report STATE=ENABLED during run");

      driverSim.setLeftY(0.0);
      driverSim.setRightX(0.0);
      driverSim.setRightTriggerAxis(0.0);
      driverSim.notifyNewData();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      Thread.sleep(150);

      robot.endCompetition();
      robotThread.join(10_000L);
      assertFalse(robotThread.isAlive(), "Robot loop thread must terminate cleanly on endCompetition()");
      if (threadFailure[0] != null) {
        throw new AssertionError("Robot loop threw an exception", threadFailure[0]);
      }
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      if (prevLogDir == null) {
        System.clearProperty("flux.logDir");
      } else {
        System.setProperty("flux.logDir", prevLogDir);
      }
    }

    assertFalse(Files.exists(stateFile), "Clean Robot.close() must delete state heartbeat file");

    List<Path> wpilogs;
    List<Path> hoots;
    try (Stream<Path> walk = Files.walk(outDir)) {
      List<Path> all = walk.filter(Files::isRegularFile).toList();
      wpilogs = all.stream().filter(p -> p.getFileName().toString().endsWith(".wpilog")).toList();
      hoots = all.stream().filter(p -> p.getFileName().toString().endsWith(".hoot")).toList();
    }
    assertEquals(0, hoots.size(), "SignalLogger .hoot auto-logging must stay OFF for M1");
    assertFalse(wpilogs.isEmpty(), "Expected AdvantageKit WPILOGWriter to produce a .wpilog file");

    Path logFile = wpilogs.get(0);
    assertTrue(Files.size(logFile) > 50_000L, "30s .wpilog must contain substantial telemetry data");

    FluxWpiLogCodec.ParsedLog parsed = FluxWpiLogCodec.read(logFile);
    assertTrue(
        parsed.durationSeconds() >= Math.max(1.0, durationSec - 2.0),
        "Logged duration must cover ~" + durationSec + "s (actual=" + parsed.durationSeconds() + ")");

    WpilogScan scan = scanWpilogEntries(logFile);
    String loggedConfigSha = scan.stringValues.get("/RealMetadata/ConfigSHA256");
    if (loggedConfigSha == null) {
      loggedConfigSha = scan.stringValues.get("/Metadata/ConfigSHA256");
    }
    assertEquals(
        expectedConfigSha,
        loggedConfigSha,
        "ConfigSHA256 metadata must be recorded before Logger.start() and match FluxFig boot SHA");

    assertTrue(
        scan.hasNonEmptyDoubleArrayEndingWith("Drive/Module0/OdometryTimestamps"),
        "Drive/Module0/OdometryTimestamps must be logged with non-empty samples");
    assertTrue(
        scan.hasEntryEndingWith("PowerDistribution/Voltage")
            && scan.hasEntryEndingWith("PowerDistribution/TotalCurrent"),
        "PowerDistribution voltage and total current channels must be logged");
    assertTrue(
        scan.hasEntryEndingWith("Drive/Module0/DriveAppliedVolts")
            && scan.hasEntryEndingWith("Drive/Module0/DriveVelocityRadPerSec")
            && scan.hasEntryEndingWith("Drive/Module0/DrivePositionRad"),
        "Drive module voltage, velocity, and position channels must be logged");
    assertTrue(
        scan.hasEntryEndingWith("Shooter/AppliedVoltage")
            && scan.hasEntryEndingWith("Shooter/VelocityRPS")
            && scan.hasEntryEndingWith("Hood/PositionRotations"),
        "Mechanism voltage, velocity, and position channels must be logged");
    assertTrue(
        scan.hasEntryEndingWith("Vision/Camera0/Connected"),
        "Vision camera inputs must be logged");
    assertTrue(
        scan.hasEntryEndingWith("Performance/TotalLoopMs"),
        "Performance/TotalLoopMs must be logged");

    List<Double> totalLoopSamples = scan.doubleSeriesEndingWith("Performance/TotalLoopMs");
    assertTrue(
        totalLoopSamples.size() > 20,
        "Expected > 20 Performance/TotalLoopMs samples (got " + totalLoopSamples.size() + ")");
    int warmupTicks = Math.min(5, totalLoopSamples.size() - 1);
    double tick1LoopMs = totalLoopSamples.get(0);
    double maxPostBootLoopMs = 0.0;
    int steadyStateOverruns = 0;
    for (int i = warmupTicks; i < totalLoopSamples.size(); i++) {
      double ms = totalLoopSamples.get(i);
      maxPostBootLoopMs = Math.max(maxPostBootLoopMs, ms);
      if (ms >= 20.0) {
        steadyStateOverruns++;
      }
    }
    assertTrue(
        tick1LoopMs < 500.0,
        "Tick-1 loop time must stay well below multi-second startup stall (actual="
            + tick1LoopMs
            + " ms)");
    assertTrue(
        maxPostBootLoopMs < 20.0,
        "Post-boot Performance/TotalLoopMs must stay < 20.0 ms with zero steady-state overruns"
            + " (maxPostBootLoopMs="
            + maxPostBootLoopMs
            + " ms, overruns="
            + steadyStateOverruns
            + ")");

    System.out.printf(
        "[verifyWpilog] PASS: %s (%d bytes, %.2fs, %d topics, ConfigSHA256=%s, tick1=%.2fms,"
            + " maxPostBoot=%.2fms, overruns=%d)%n",
        logFile.getFileName(),
        Files.size(logFile),
        parsed.durationSeconds(),
        scan.entryNames.size(),
        loggedConfigSha,
        tick1LoopMs,
        maxPostBootLoopMs,
        steadyStateOverruns);
  }

  private record WpilogScan(
      Map<Integer, String> entryNames,
      Map<Integer, String> entryTypes,
      Map<String, Integer> sampleCounts,
      Map<String, String> stringValues,
      Map<String, Integer> maxDoubleArrayLen,
      Map<String, List<Double>> doubleSeries) {
    boolean hasEntryEndingWith(String suffix) {
      for (Map.Entry<String, Integer> e : sampleCounts.entrySet()) {
        if (e.getKey().endsWith(suffix) && e.getValue() > 0) {
          return true;
        }
      }
      return false;
    }

    boolean hasNonEmptyDoubleArrayEndingWith(String suffix) {
      for (Map.Entry<String, Integer> e : maxDoubleArrayLen.entrySet()) {
        if (e.getKey().endsWith(suffix) && e.getValue() > 0) {
          return true;
        }
      }
      return false;
    }

    List<Double> doubleSeriesEndingWith(String suffix) {
      for (Map.Entry<String, List<Double>> e : doubleSeries.entrySet()) {
        if (e.getKey().endsWith(suffix)) {
          return e.getValue();
        }
      }
      return List.of();
    }
  }

  private static WpilogScan scanWpilogEntries(Path file) throws IOException {
    byte[] data = Files.readAllBytes(file);
    ByteBuffer buf = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
    assertTrue(data.length >= 12, "WPILOG header truncated");
    String magic = new String(data, 0, 6, StandardCharsets.US_ASCII);
    assertEquals("WPILOG", magic);
    int extraLen = buf.getInt(8);
    int pos = 12 + extraLen;

    Map<Integer, String> entryNames = new LinkedHashMap<>();
    Map<Integer, String> entryTypes = new LinkedHashMap<>();
    Map<String, Integer> sampleCounts = new LinkedHashMap<>();
    Map<String, String> stringValues = new LinkedHashMap<>();
    Map<String, Integer> maxDoubleArrayLen = new LinkedHashMap<>();
    Map<String, List<Double>> doubleSeries = new LinkedHashMap<>();

    while (pos + 4 <= data.length) {
      int hdr = data[pos++] & 0xFF;
      int idLen = (hdr & 0x03) + 1;
      int szLen = ((hdr >> 2) & 0x03) + 1;
      int tsLen = ((hdr >> 4) & 0x07) + 1;
      if (pos + idLen + szLen + tsLen > data.length) {
        break;
      }
      int eid = (int) readUnsignedLe(data, pos, idLen);
      pos += idLen;
      int sz = (int) readUnsignedLe(data, pos, szLen);
      pos += szLen;
      pos += tsLen;
      if (sz < 0 || pos + sz > data.length) {
        break;
      }
      if (eid == 0 && sz >= 1) {
        int ctrl = data[pos] & 0xFF;
        if (ctrl == 0 && sz >= 9) {
          int newEid = buf.getInt(pos + 1);
          int nameLen = buf.getInt(pos + 5);
          int nameStart = pos + 9;
          int typeLenPos = nameStart + nameLen;
          if (typeLenPos + 4 <= pos + sz) {
            String name = new String(data, nameStart, nameLen, StandardCharsets.UTF_8);
            int typeLen = buf.getInt(typeLenPos);
            String type = new String(data, typeLenPos + 4, typeLen, StandardCharsets.UTF_8);
            entryNames.put(newEid, name);
            entryTypes.put(newEid, type);
          }
        }
      } else {
        String name = entryNames.get(eid);
        String type = entryTypes.get(eid);
        if (name != null) {
          sampleCounts.merge(name, 1, Integer::sum);
          if ("string".equals(type)) {
            stringValues.put(name, new String(data, pos, sz, StandardCharsets.UTF_8));
          } else if ("double".equals(type) && sz == 8) {
            doubleSeries.computeIfAbsent(name, k -> new ArrayList<>()).add(buf.getDouble(pos));
          } else if ("double[]".equals(type)) {
            int arrLen = sz / 8;
            maxDoubleArrayLen.merge(name, arrLen, Math::max);
          }
        }
      }
      pos += sz;
    }
    return new WpilogScan(
        entryNames, entryTypes, sampleCounts, stringValues, maxDoubleArrayLen, doubleSeries);
  }

  private static long readUnsignedLe(byte[] data, int offset, int len) {
    long value = 0L;
    for (int i = 0; i < len; i++) {
      value |= ((long) (data[offset + i] & 0xFF)) << (8 * i);
    }
    return value;
  }
}
