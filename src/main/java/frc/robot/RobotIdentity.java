package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

/**
 * Fleet robot identities for FRC Team 10413 (FLUX Robotics), resolving via NI roboRIO / SystemCore
 * hardware hex serial map, HAL serial/comments, Linux device-tree serial, roboRIO/SystemCore
 * persistent identity tag files, and deploy tag before falling back loudly to {@link #COMP_2027}.
 */
public enum RobotIdentity {
  COMP_2027,
  PRACTICE_2027,
  FUEL,
  CORAL,
  CHASSIS_3;

  public static final Path DEFAULT_DEVICETREE_SERIAL =
      Paths.get("/sys/firmware/devicetree/base/serial-number");
  public static final Path DEFAULT_ROBORIO_PERSISTENT_TAG =
      Paths.get("/home/lvuser/robot-identity.txt");
  public static final Path DEFAULT_SYSTEMCORE_PERSISTENT_TAG =
      Paths.get("/home/systemcore/robot-identity.txt");
  private static final String DEPLOY_TAG_FILENAME = "robot-identity.txt";

  /**
   * Known physical controller hex serial numbers mapped to {@link RobotIdentity} (normalized
   * uppercase hex without leading {@code 0x}). Populate with verified NI roboRIO 8-hex and
   * SystemCore 16-hex serials during hardware inventory; {@code /home/lvuser/robot-identity.txt}
   * provides immediate on-robot identification until each board serial is recorded here.
   */
  private static final Map<String, RobotIdentity> HARDWARE_SERIAL_MAP = buildHardwareSerialMap();

  private static volatile String lastResolutionSource = "UNINITIALIZED";
  private static volatile boolean lastResolutionUsedFallback = false;
  private static volatile String lastWarningMessage = "";

  private static Map<String, RobotIdentity> buildHardwareSerialMap() {
    Map<String, RobotIdentity> map = new LinkedHashMap<>();
    // Recorded physical controller hex serials (add 8-hex roboRIO / 16-hex SystemCore IDs on M1.11)
    return Collections.unmodifiableMap(map);
  }

  public static Map<String, RobotIdentity> getHardwareSerialMap() {
    return HARDWARE_SERIAL_MAP;
  }

  public static RobotIdentity fromSerialNumber(String serial) {
    return fromSerialOrComments(serial, "");
  }

  public static RobotIdentity fromSerialOrComments(String serial, String comments) {
    Optional<RobotIdentity> fromMap = resolveFromSerialMap(serial, HARDWARE_SERIAL_MAP);
    if (fromMap.isPresent()) {
      return fromMap.get();
    }
    String combined =
        ((serial == null ? "" : serial) + " " + (comments == null ? "" : comments)).trim();
    return parseExplicitToken(combined).orElse(COMP_2027);
  }

  /**
   * Resolves the active {@link RobotIdentity} across hardware hex serial map, HAL serial/comments,
   * Linux device-tree serial, {@code /home/lvuser/robot-identity.txt}, {@code
   * /home/systemcore/robot-identity.txt}, and deploy tag before falling back loudly to {@link
   * #COMP_2027}.
   */
  public static RobotIdentity detect() {
    String halSerial = RobotController.getSerialNumber();
    String comments = readHalCommentsSafe();
    Path deployTag = Filesystem.getDeployDirectory().toPath().resolve(DEPLOY_TAG_FILENAME);
    return detect(
        halSerial,
        comments,
        DEFAULT_DEVICETREE_SERIAL,
        DEFAULT_ROBORIO_PERSISTENT_TAG,
        DEFAULT_SYSTEMCORE_PERSISTENT_TAG,
        deployTag,
        HARDWARE_SERIAL_MAP);
  }

  /** Deterministic overload preserved for unit/sim tests with a single persistent tag path. */
  public static RobotIdentity detect(
      String halSerial,
      String comments,
      Path devicetreeSerialPath,
      Path deployTagPath,
      Path persistentTagPath) {
    return detect(
        halSerial,
        comments,
        devicetreeSerialPath,
        persistentTagPath,
        null,
        deployTagPath,
        HARDWARE_SERIAL_MAP);
  }

  /** Full deterministic overload supporting roboRIO/SystemCore tag paths and hex serial maps. */
  public static RobotIdentity detect(
      String halSerial,
      String comments,
      Path devicetreeSerialPath,
      Path roborioPersistentTagPath,
      Path systemcorePersistentTagPath,
      Path deployTagPath,
      Map<String, RobotIdentity> serialMap) {
    Optional<RobotIdentity> fromHalSerialMap = resolveFromSerialMap(halSerial, serialMap);
    if (fromHalSerialMap.isPresent()) {
      recordMatch("HAL_SERIAL_MAP");
      return fromHalSerialMap.get();
    }

    String combined =
        ((halSerial == null ? "" : halSerial) + " " + (comments == null ? "" : comments)).trim();
    Optional<RobotIdentity> fromHal = parseExplicitToken(combined);
    if (fromHal.isPresent()) {
      recordMatch("HAL_SERIAL_OR_COMMENTS");
      return fromHal.get();
    }

    String devTreeRaw = readCleanText(devicetreeSerialPath);
    Optional<RobotIdentity> fromDevTreeMap = resolveFromSerialMap(devTreeRaw, serialMap);
    if (fromDevTreeMap.isPresent()) {
      recordMatch("DEVICETREE_SERIAL_MAP");
      return fromDevTreeMap.get();
    }
    Optional<RobotIdentity> fromDevTree = parseExplicitToken(devTreeRaw);
    if (fromDevTree.isPresent()) {
      recordMatch("DEVICETREE_SERIAL");
      return fromDevTree.get();
    }

    Optional<RobotIdentity> fromRioTag =
        parseExplicitToken(readCleanText(roborioPersistentTagPath));
    if (fromRioTag.isPresent()) {
      recordMatch("ROBORIO_PERSISTENT_TAG");
      return fromRioTag.get();
    }

    Optional<RobotIdentity> fromSystemCoreTag =
        parseExplicitToken(readCleanText(systemcorePersistentTagPath));
    if (fromSystemCoreTag.isPresent()) {
      recordMatch("SYSTEMCORE_PERSISTENT_TAG");
      return fromSystemCoreTag.get();
    }

    Optional<RobotIdentity> fromDeployTag = parseExplicitToken(readCleanText(deployTagPath));
    if (fromDeployTag.isPresent()) {
      recordMatch("DEPLOY_TAG");
      return fromDeployTag.get();
    }

    lastResolutionSource = "DEFAULT_COMP_2027";
    lastResolutionUsedFallback = true;
    String normalizedHal = normalizeSerial(halSerial);
    String serialDisplay = normalizedHal.isEmpty() ? "<blank>" : normalizedHal;
    lastWarningMessage =
        "RobotIdentity: unrecognized controller serial '"
            + serialDisplay
            + "' and no identity tag found; falling back to COMP_2027. Write 'FUEL' or 'CORAL' to"
            + " /home/lvuser/robot-identity.txt (or add serial to HARDWARE_SERIAL_MAP).";
    DriverStation.reportWarning(lastWarningMessage, false);
    return COMP_2027;
  }

  private static void recordMatch(String source) {
    lastResolutionSource = source;
    lastResolutionUsedFallback = false;
    lastWarningMessage = "";
  }

  public static String getLastResolutionSource() {
    return lastResolutionSource;
  }

  public static boolean usedFallback() {
    return lastResolutionUsedFallback;
  }

  public static String getLastWarningMessage() {
    return lastWarningMessage;
  }

  static Optional<RobotIdentity> resolveFromSerialMap(
      String rawSerial, Map<String, RobotIdentity> serialMap) {
    if (serialMap == null || serialMap.isEmpty()) {
      return Optional.empty();
    }
    String normalized = normalizeSerial(rawSerial);
    if (normalized.isEmpty()) {
      return Optional.empty();
    }
    RobotIdentity direct = serialMap.get(normalized);
    if (direct != null) {
      return Optional.of(direct);
    }
    for (Map.Entry<String, RobotIdentity> entry : serialMap.entrySet()) {
      if (normalized.equals(normalizeSerial(entry.getKey()))) {
        return Optional.of(entry.getValue());
      }
    }
    return Optional.empty();
  }

  static String normalizeSerial(String raw) {
    if (raw == null) {
      return "";
    }
    String cleaned = raw.replace("\0", "").trim().toUpperCase(Locale.ROOT);
    if (cleaned.startsWith("0X")) {
      cleaned = cleaned.substring(2);
    }
    return cleaned;
  }

  static Optional<RobotIdentity> parseExplicitToken(String raw) {
    if (raw == null || raw.isBlank()) {
      return Optional.empty();
    }
    String upper = raw.replace("\0", "").trim().toUpperCase(Locale.ROOT);
    if (upper.contains("CORAL")) {
      return Optional.of(CORAL);
    }
    if (upper.contains("FUEL")) {
      return Optional.of(FUEL);
    }
    if (upper.contains("CHASSIS")) {
      return Optional.of(CHASSIS_3);
    }
    if (upper.contains("PRACTICE")) {
      return Optional.of(PRACTICE_2027);
    }
    if (upper.contains("COMP_2027") || upper.equals("COMP")) {
      return Optional.of(COMP_2027);
    }
    return Optional.empty();
  }

  private static String readHalCommentsSafe() {
    try {
      // Remove getComments() after the WPILib 2027 GradleRIO cutover (deleted in allwpilib#9645).
      return RobotController.getComments();
    } catch (Throwable ignored) {
      return "";
    }
  }

  private static String readCleanText(Path path) {
    if (path == null) {
      return "";
    }
    try {
      if (!Files.exists(path)) {
        return "";
      }
      return Files.readString(path, StandardCharsets.UTF_8).replace("\0", "").trim();
    } catch (Exception ignored) {
      return "";
    }
  }
}
