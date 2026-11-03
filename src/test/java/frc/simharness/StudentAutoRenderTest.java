package frc.simharness;

import edu.wpi.first.hal.HAL;
import frc.lib.util.SafeAutoBuilder;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.autos.StudentAutoRoutines;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Locale;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Dedicated simulation entry point for {@code ./gradlew renderAuto [-Pauto="<name>"]}.
 *
 * <p>Decoupled from Tier 2 hardcoded geometry assertions so students can build and visualize any
 * 2026 autonomous trajectory on {@code FUEL} without failing JUnit assertions before {@code
 * FluxProcessor} renders {@code student-auto.html} and {@code student-auto.mp4}.
 */
public class StudentAutoRenderTest {

  @BeforeAll
  public static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  public void renderSelectedStudentAuto() throws Exception {
    Path configDir = Paths.get("src/main/deploy/configs");
    RobotContainer container = new RobotContainer(RobotIdentity.FUEL, configDir, true);
    String selector = System.getProperty("flux.auto", "").trim();

    if ("list".equalsIgnoreCase(selector)) {
      System.out.println("Available 2026 FUEL Autonomous Routines (-Pauto=\"<name>\"):");
      for (String key :
          StudentAutoRoutines.createNamedAutos(
                  container.getDrive(),
                  container.getSuperstructure(),
                  container.getShotCalculator())
              .keySet()) {
        System.out.println("  • " + key);
      }
    }

    var selectedAuto =
        "list".equalsIgnoreCase(selector)
            ? container.getAutonomousCommand()
            : container.resolveAutonomousCommand(selector);
    Path outDir = Paths.get("build/auto-sim");
    var frames =
        StudentAutoSimExporter.simulateAndExportJson(
            container, selectedAuto, selectedAuto.getName(), outDir);

    String jsonText =
        Files.readString(outDir.resolve("student-auto-frames.json"), StandardCharsets.UTF_8);
    int shotsScored = countOccurrences(jsonText, "\"scored\":true");
    int shotsTotal =
        shotsScored + countOccurrences(jsonText, "\"scored\":false");
    int ballsUnpicked = countOccurrences(jsonText, "\"tPick\":999.000");
    int ballsTotal = countOccurrences(jsonText, "\"tPick\":");
    int ballsIntaked = Math.max(0, ballsTotal - ballsUnpicked);

    var first = frames.get(0);
    var last = frames.get(frames.size() - 1);
    double pathLengthM = 0.0;
    double minVolt = 99.0;
    for (int i = 0; i < frames.size(); i++) {
      minVolt = Math.min(minVolt, frames.get(i).batteryVoltage());
      if (i > 0) {
        pathLengthM +=
            Math.hypot(
                frames.get(i).xMeters() - frames.get(i - 1).xMeters(),
                frames.get(i).yMeters() - frames.get(i - 1).yMeters());
      }
    }
    System.out.printf(
        Locale.US,
        "[renderAuto] Simulated '%s' (%d frames, %.2f m driven, %d floor balls intaked, %d/%d Hub shots scored, minV=%.2f V): start=(%.2f, %.2f) -> end=(%.2f, %.2f)%n",
        selectedAuto.getName(),
        frames.size(),
        pathLengthM,
        ballsIntaked,
        shotsScored,
        shotsTotal,
        minVolt,
        first.xMeters(),
        first.yMeters(),
        last.xMeters(),
        last.yMeters());
    String warn = SafeAutoBuilder.getLastCancelReason();
    if (!warn.isBlank()) {
      System.out.println("[renderAuto] SafeAuto Warning: " + warn);
    } else if (pathLengthM < 0.02
        && shotsTotal == 0
        && selectedAuto.getName().toUpperCase(Locale.US).startsWith("STUDENT")) {
      System.out.println(
          "[renderAuto] STUDENT NOTE: Your default STUDENT auto is currently empty — open src/main/java/frc/robot/autos/StudentAutoRoutines.java (createStudentDefaultAuto), add commands, and re-run ./gradlew renderAuto!");
    }
    warnIfWallClockTimeoutUsed();
  }

  private static void warnIfWallClockTimeoutUsed() throws Exception {
    Path srcFile = Paths.get("src/main/java/frc/robot/autos/StudentAutoRoutines.java");
    if (!Files.exists(srcFile)) {
      return;
    }
    for (String line : Files.readAllLines(srcFile, StandardCharsets.UTF_8)) {
      String trimmed = line.trim();
      if (trimmed.startsWith("//") || trimmed.startsWith("*")) {
        continue;
      }
      if (trimmed.contains("Commands.waitSeconds(")
          || trimmed.contains("new WaitCommand(")
          || trimmed.contains(".withTimeout(")) {
        System.out.println(
            "[renderAuto] Tip: Replace Commands.waitSeconds(...) / .withTimeout(...) with waitLoopSeconds(...) / withLoopTimeout(...) so fast-forward ./gradlew renderAuto simulation advances!");
        break;
      }
    }
  }

  private static int countOccurrences(String text, String sub) {
    int count = 0;
    int idx = 0;
    while ((idx = text.indexOf(sub, idx)) >= 0) {
      count++;
      idx += sub.length();
    }
    return count;
  }
}
