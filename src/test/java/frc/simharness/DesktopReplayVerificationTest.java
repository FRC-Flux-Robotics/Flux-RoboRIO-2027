package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.robot.Robot;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

/**
 * Isolated-JVM end-to-end verification of {@link Robot.Mode#REPLAY} ({@code ./gradlew
 * verifyReplay}): spawns a child JVM running {@link Robot#startCompetition()} with {@code
 * WPILOGReader} + {@code setUseTiming(false)} (isolating AdvantageKit's EOF {@code System.exit(0)}
 * from JUnit), then parses the flushed {@code replayed_sim.wpilog} and verifies timestamp-aligned
 * {@code Drive/Pose} reproduces the original recorded run within {@code < 1 cm}.
 */
public class DesktopReplayVerificationTest {

  /** Child-JVM entry point invoked by {@link #testFullRobotModeReplayReproducesLoggedPoseWithin1Cm()}. */
  public static void main(String[] args) throws Exception {
    if (args.length < 3) {
      System.err.println("Usage: DesktopReplayVerificationTest <stateFile> <inputLog> <outputLog>");
      System.exit(2);
    }
    if (!HAL.initialize(500, 0)) {
      System.err.println("WPILib HAL failed to initialize");
      System.exit(3);
    }
    Path stateFile = Paths.get(args[0]).toAbsolutePath();
    Path inputLog = Paths.get(args[1]).toAbsolutePath();
    Path outputLog = Paths.get(args[2]).toAbsolutePath();
    try (Robot robot = new Robot(stateFile, inputLog, outputLog)) {
      if (robot.getMode() != Robot.Mode.REPLAY) {
        System.err.println("Expected Mode.REPLAY, got " + robot.getMode());
        System.exit(4);
      }
      robot.startCompetition();
    }
  }

  @Test
  public void testFullRobotModeReplayReproducesLoggedPoseWithin1Cm() throws Exception {
    Path inputLog = Tier5NytrReplayGateTest.resolveReplayFixture().toAbsolutePath();
    assertTrue(Files.isRegularFile(inputLog), "Replay fixture missing: " + inputLog);

    Path outDir = Paths.get("build/m22-replay-verify").toAbsolutePath();
    Files.createDirectories(outDir);
    Path outLog = outDir.resolve("replayed_sim.wpilog");
    Files.deleteIfExists(outLog);
    Path stateFile = outDir.resolve("replay-state");
    Files.deleteIfExists(stateFile);

    String javaBin =
        ProcessHandle.current()
            .info()
            .command()
            .orElse(Paths.get(System.getProperty("java.home"), "bin", "java").toString());
    List<String> cmd = new ArrayList<>();
    cmd.add(javaBin);
    String libPath = System.getProperty("java.library.path");
    if (libPath != null && !libPath.isBlank()) {
      cmd.add("-Djava.library.path=" + libPath);
    }
    cmd.add("-cp");
    cmd.add(System.getProperty("java.class.path"));
    cmd.add(DesktopReplayVerificationTest.class.getName());
    cmd.add(stateFile.toString());
    cmd.add(inputLog.toString());
    cmd.add(outLog.toString());

    ProcessBuilder pb = new ProcessBuilder(cmd);
    pb.redirectErrorStream(true);
    Process proc = pb.start();
    StringBuilder childOut = new StringBuilder();
    try (BufferedReader br =
        new BufferedReader(new InputStreamReader(proc.getInputStream(), StandardCharsets.UTF_8))) {
      String line;
      while ((line = br.readLine()) != null) {
        childOut.append(line).append('\n');
      }
    }
    boolean exited = proc.waitFor(60, TimeUnit.SECONDS);
    if (!exited) {
      proc.destroyForcibly();
    }
    assertTrue(exited, "Mode.REPLAY child JVM timed out:\n" + childOut);
    assertEquals(0, proc.exitValue(), "Mode.REPLAY child JVM failed:\n" + childOut);

    assertFalse(
        Files.exists(stateFile), "Mode.REPLAY must not write /tmp/flux-robot-state heartbeat");
    assertTrue(
        Files.isRegularFile(outLog) && Files.size(outLog) > 10_000L,
        "Mode.REPLAY must flush a non-empty output .wpilog: " + outLog);

    FluxWpiLogCodec.ParsedLog orig = FluxWpiLogCodec.read(inputLog);
    FluxWpiLogCodec.ParsedLog replayed = FluxWpiLogCodec.read(outLog);

    List<FluxWpiLogCodec.DoubleSample> origX = orig.getDoubleSeries("/RealOutputs/Drive/Pose.x");
    List<FluxWpiLogCodec.DoubleSample> origY = orig.getDoubleSeries("/RealOutputs/Drive/Pose.y");
    List<FluxWpiLogCodec.DoubleSample> repX =
        replayed.getDoubleSeries("/ReplayOutputs/Drive/Pose.x");
    List<FluxWpiLogCodec.DoubleSample> repY =
        replayed.getDoubleSeries("/ReplayOutputs/Drive/Pose.y");

    assertTrue(origX.size() >= 50, "Original log must have >= 50 Drive/Pose samples");
    assertTrue(repX.size() >= 50, "Replayed log must have >= 50 Drive/Pose samples");

    int matched = 0;
    double maxErrMeters = 0.0;
    int origIdx = 0;
    int origCount = Math.min(origX.size(), origY.size());
    int repCount = Math.min(repX.size(), repY.size());
    for (int i = 0; i < repCount; i++) {
      long repTs = repX.get(i).timestampUs();
      while (origIdx + 1 < origCount && origX.get(origIdx + 1).timestampUs() <= repTs) {
        origIdx++;
      }
      if (Math.abs(origX.get(origIdx).timestampUs() - repTs) <= 2_000L) {
        double dx = repX.get(i).value() - origX.get(origIdx).value();
        double dy = repY.get(i).value() - origY.get(origIdx).value();
        maxErrMeters = Math.max(maxErrMeters, Math.hypot(dx, dy));
        matched++;
      }
    }

    assertTrue(
        matched >= 50,
        "Expected >= 50 timestamp-aligned Drive/Pose comparisons (matched=" + matched + ")");
    assertTrue(
        maxErrMeters < 0.010,
        "Full Robot Mode.REPLAY output Drive/Pose must match input within < 1 cm (maxErr="
            + String.format("%.5f", maxErrMeters)
            + " m across "
            + matched
            + " matched samples)");
    System.out.printf(
        "[verifyReplay] PASS: %d timestamp-aligned pose samples compared, maxErr=%.6f m (< 0.010 m)%n",
        matched, maxErrMeters);
  }
}
