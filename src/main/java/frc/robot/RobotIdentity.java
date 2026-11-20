package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Pattern;

/**
 * Fleet robot identities for FRC Team 10413 (FLUX Robotics), resolving via NI roboRIO / SystemCore
 * hardware hex serial map, persistent identity tag files ({@code /home/lvuser/robot-identity.txt}
 * and {@code /home/systemcore/robot-identity.txt}), deploy tag, Linux device-tree serial, and
 * legacy HAL serial/comments before falling back loudly to the roboRIO-safe {@link #FUEL} profile.
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
  private static final Pattern TOKEN_SPLIT_PATTERN = Pattern.compile("[^A-Z0-9_]+");

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
    return parseExplicitToken(combined).orElse(FUEL);
  }

  /**
   * Resolves the active {@link RobotIdentity} across hardware hex serial map, {@code
   * /home/lvuser/robot-identity.txt}, {@code /home/systemcore/robot-identity.txt}, deploy tag,
   * Linux device-tree serial, and legacy HAL serial/comments before falling back loudly to {@link
   * #FUEL}.
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

    String combined =
        ((halSerial == null ? "" : halSerial) + " " + (comments == null ? "" : comments)).trim();
    Optional<RobotIdentity> fromHal = parseExplicitToken(combined);
    if (fromHal.isPresent()) {
      recordMatch("HAL_SERIAL_OR_COMMENTS");
      return fromHal.get();
    }

    lastResolutionSource = "DEFAULT_FUEL";
    lastResolutionUsedFallback = true;
    String normalizedHal = normalizeSerial(halSerial);
    String serialDisplay = normalizedHal.isEmpty() ? "<blank>" : normalizedHal;
    lastWarningMessage =
        "RobotIdentity: unrecognized controller serial '"
            + serialDisplay
            + "' and no valid identity tag found; falling back to roboRIO-safe FUEL profile. Write"
            + " 'FUEL' or 'CORAL' to /home/lvuser/robot-identity.txt (or add serial to"
            + " HARDWARE_SERIAL_MAP).";
    DriverStation.reportError(lastWarningMessage, false);
    return FUEL;
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

  /**
   * Matches an exact or word-bounded robot identity token; rejects substring noise and ambiguous
   * multi-identity strings so legacy comments cannot shadow explicit identity files.
   */
  static Optional<RobotIdentity> parseExplicitToken(String raw) {
    if (raw == null || raw.isBlank()) {
      return Optional.empty();
    }
    String upper = raw.replace("\0", "").trim().toUpperCase(Locale.ROOT);
    Optional<RobotIdentity> exact = matchSingleToken(upper);
    if (exact.isPresent()) {
      return exact;
    }

    Set<RobotIdentity> matches = EnumSet.noneOf(RobotIdentity.class);
    for (String token : TOKEN_SPLIT_PATTERN.split(upper)) {
      if (token.isEmpty()) {
        continue;
      }
      matchSingleToken(token).ifPresent(matches::add);
    }
    if (matches.size() == 1) {
      return Optional.of(matches.iterator().next());
    }
    return Optional.empty();
  }

  private static Optional<RobotIdentity> matchSingleToken(String token) {
    return switch (token) {
      case "FUEL" -> Optional.of(FUEL);
      case "CORAL" -> Optional.of(CORAL);
      case "CHASSIS_3" -> Optional.of(CHASSIS_3);
      case "PRACTICE_2027", "PRACTICE" -> Optional.of(PRACTICE_2027);
      case "COMP_2027", "COMP" -> Optional.of(COMP_2027);
      default -> Optional.empty();
    };
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
