package frc.lib.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.FileStore;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * OS-agnostic AdvantageKit & CTRE log storage manager (`/U/logs` -> operating directory `logs`),
 * verifying that `/U` is a real mounted writable filesystem before writing, recursively cleaning
 * old `.wpilog` / `.hoot` files and empty directories, and atomically writing `run-meta.json` and
 * `/tmp/flux-robot-state`.
 */
public final class LogFileManager {
  private static final long MAX_STORAGE_BYTES = 500L * 1024 * 1024; // 500 MB
  private static final Path DEFAULT_USB_MOUNT = Path.of("/U");
  private static final Path DEFAULT_ROOT_PATH = Path.of("/");
  private static final Path PROC_MOUNTS = Path.of("/proc/mounts");

  private LogFileManager() {}

  /**
   * Resolves `/U/logs` only if `/U` is an actually mounted, writable filesystem (never writing to
   * an unmounted root-flash `/U` stub), else falls back to `<operatingDir>/logs`.
   */
  public static String getLogPath() {
    String overrideDir = System.getProperty("flux.logDir");
    if (overrideDir != null && !overrideDir.isBlank()) {
      return getLogPath(new File(overrideDir.trim()), MAX_STORAGE_BYTES);
    }
    if (isMountedWritableDirectory(DEFAULT_USB_MOUNT, DEFAULT_ROOT_PATH)) {
      File usbLogsDir = DEFAULT_USB_MOUNT.resolve("logs").toFile();
      if (ensureWritableDirectory(usbLogsDir)) {
        return getLogPath(usbLogsDir, MAX_STORAGE_BYTES);
      }
    }
    File localDir = new File(Filesystem.getOperatingDirectory(), "logs");
    return getLogPath(localDir, MAX_STORAGE_BYTES);
  }

  /**
   * Returns true only when {@code mountPoint} is an existing writable directory backed by a
   * separate mounted filesystem (either a distinct {@link FileStore} from {@code rootPath} or an
   * active entry in {@code /proc/mounts}).
   */
  public static boolean isMountedWritableDirectory(Path mountPoint, Path rootPath) {
    return isMountedWritableDirectory(mountPoint, rootPath, PROC_MOUNTS);
  }

  /** Deterministic overload allowing explicit mount table paths in unit tests. */
  public static boolean isMountedWritableDirectory(
      Path mountPoint, Path rootPath, Path procMountsPath) {
    if (mountPoint == null || !Files.isDirectory(mountPoint) || !Files.isWritable(mountPoint)) {
      return false;
    }
    try {
      Path realMount = mountPoint.toRealPath();
      if (!Files.isDirectory(realMount) || !Files.isWritable(realMount)) {
        return false;
      }
      if (isListedInProcMounts(realMount, procMountsPath)
          || isListedInProcMounts(mountPoint.toAbsolutePath().normalize(), procMountsPath)) {
        return true;
      }
      if (rootPath != null && Files.exists(rootPath)) {
        FileStore mountStore = Files.getFileStore(realMount);
        FileStore rootStore = Files.getFileStore(rootPath.toRealPath());
        return !mountStore.equals(rootStore);
      }
      return false;
    } catch (IOException ignored) {
      return false;
    }
  }

  static boolean isListedInProcMounts(Path targetPath, Path procMountsPath) {
    if (targetPath == null || procMountsPath == null || !Files.isRegularFile(procMountsPath)) {
      return false;
    }
    String expected = targetPath.toString();
    try {
      for (String line : Files.readAllLines(procMountsPath, StandardCharsets.UTF_8)) {
        String trimmed = line.trim();
        if (trimmed.isEmpty()) {
          continue;
        }
        String[] parts = trimmed.split("\\s+");
        if (parts.length >= 2 && expected.equals(parts[1])) {
          return true;
        }
      }
    } catch (IOException ignored) {
    }
    return false;
  }

  private static boolean ensureWritableDirectory(File dir) {
    if (dir == null) {
      return false;
    }
    if (!dir.exists() && !dir.mkdirs()) {
      return false;
    }
    return dir.isDirectory() && Files.isWritable(dir.toPath());
  }

  public static String getLogPath(File logDir, long maxBytes) {
    if (!logDir.exists()) {
      logDir.mkdirs();
    }
    cleanOldLogs(logDir, maxBytes);
    return logDir.getAbsolutePath();
  }

  /** Writes per-run `run-meta.json` summary inside the active log directory for FluxTray/FluxProcessor. */
  public static Path writeRunMetaJson(
      Path logDir,
      String robotId,
      String profile,
      String configSha256,
      String batteryName,
      int loopPeriodMs)
      throws IOException {
    return writeRunMetaJson(
        logDir,
        robotId,
        profile,
        configSha256,
        batteryName,
        loopPeriodMs,
        readDeployedGitSha(Filesystem.getDeployDirectory().toPath()),
        Instant.now().toString());
  }

  /**
   * Atomically writes per-run `run-meta.json` summary including deployed `gitSha` and stable
   * `bootTimestampIso`.
   */
  public static Path writeRunMetaJson(
      Path logDir,
      String robotId,
      String profile,
      String configSha256,
      String batteryName,
      int loopPeriodMs,
      String gitSha,
      String bootTimestampIso)
      throws IOException {
    Files.createDirectories(logDir);
    Path metaFile = logDir.resolve("run-meta.json");
    String effectiveGitSha =
        (gitSha != null && !gitSha.isBlank()) ? gitSha : "unknown";
    String effectiveBootIso =
        (bootTimestampIso != null && !bootTimestampIso.isBlank())
            ? bootTimestampIso
            : Instant.now().toString();
    String json =
        "{\n"
            + "  \"robotId\": \""
            + escapeJson(robotId)
            + "\",\n"
            + "  \"profile\": \""
            + escapeJson(profile)
            + "\",\n"
            + "  \"gitSha\": \""
            + escapeJson(effectiveGitSha)
            + "\",\n"
            + "  \"configSha256\": \""
            + escapeJson(configSha256)
            + "\",\n"
            + "  \"batteryName\": \""
            + escapeJson(batteryName)
            + "\",\n"
            + "  \"loopPeriodMs\": "
            + loopPeriodMs
            + ",\n"
            + "  \"bootTimestampIso\": \""
            + escapeJson(effectiveBootIso)
            + "\"\n"
            + "}\n";
    writeStringAtomically(metaFile, json);
    return metaFile;
  }

  /**
   * Atomically writes the robot-side lockout heartbeat file ({@code /tmp/flux-robot-state}) read by
   * {@code FluxTray} SSH pre-flight checks for defense-in-depth safety lockout.
   */
  public static Path writeRobotStateFile(
      Path stateFile,
      boolean enabled,
      boolean fmsAttached,
      long epochMs,
      String robotId,
      long pid)
      throws IOException {
    Path parent = stateFile.getParent();
    if (parent != null) {
      Files.createDirectories(parent);
    }
    String state = fmsAttached ? "FMS" : (enabled ? "ENABLED" : "DISABLED");
    String safeRobotId = (robotId != null && !robotId.isBlank()) ? robotId.trim() : "UNKNOWN";
    String payload =
        "STATE="
            + state
            + "\nENABLED="
            + enabled
            + "\nFMS="
            + fmsAttached
            + "\nEPOCH_MS="
            + epochMs
            + "\nROBOT_ID="
            + safeRobotId
            + "\nPID="
            + pid
            + "\n";
    writeStringAtomically(stateFile, payload);
    return stateFile;
  }

  private static void writeStringAtomically(Path target, String content) throws IOException {
    Path parent = target.getParent();
    Path tmpFile =
        (parent != null)
            ? parent.resolve("." + target.getFileName() + ".tmp")
            : Path.of("." + target.getFileName() + ".tmp");
    Files.writeString(tmpFile, content, StandardCharsets.UTF_8);
    try {
      Files.move(
          tmpFile, target, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
    } catch (AtomicMoveNotSupportedException e) {
      Files.move(tmpFile, target, StandardCopyOption.REPLACE_EXISTING);
    }
  }

  /** Reads `gitSha` from `<deployDir>/deploy-stamp.json` if present, else returns `"unknown"`. */
  public static String readDeployedGitSha(Path deployDir) {
    if (deployDir == null) {
      return "unknown";
    }
    Path stampFile = deployDir.resolve("deploy-stamp.json");
    if (!Files.isRegularFile(stampFile)) {
      return "unknown";
    }
    try {
      String raw = Files.readString(stampFile, StandardCharsets.UTF_8);
      int idx = raw.indexOf("\"gitSha\"");
      if (idx < 0) {
        return "unknown";
      }
      int colon = raw.indexOf(':', idx);
      int firstQuote = raw.indexOf('"', colon + 1);
      int secondQuote = raw.indexOf('"', firstQuote + 1);
      if (colon > 0 && firstQuote > colon && secondQuote > firstQuote) {
        String val = raw.substring(firstQuote + 1, secondQuote).trim();
        return val.isEmpty() ? "unknown" : val;
      }
    } catch (IOException ignored) {
    }
    return "unknown";
  }

  private static String escapeJson(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  /**
   * Recursively deletes oldest `.wpilog` and `.hoot` files until total size <= maxBytes, then
   * prunes empty subdirectories left behind.
   */
  public static void cleanOldLogs(File logDir, long maxBytes) {
    List<File> logFiles = new ArrayList<>();
    collectLogFiles(logDir, logFiles);
    if (logFiles.isEmpty()) {
      return;
    }

    logFiles.sort(Comparator.comparingLong(File::lastModified));
    long totalSize = 0;
    for (File f : logFiles) {
      totalSize += f.length();
    }

    int i = 0;
    while (totalSize > maxBytes && i < logFiles.size()) {
      File target = logFiles.get(i);
      long fileSize = target.length();
      if (target.delete()) {
        totalSize -= fileSize;
      }
      i++;
    }
    pruneEmptySubdirectories(logDir, true);
  }

  private static void collectLogFiles(File dir, List<File> out) {
    File[] children = dir.listFiles();
    if (children == null) {
      return;
    }
    for (File f : children) {
      if (f.isDirectory()) {
        collectLogFiles(f, out);
      } else if (f.getName().endsWith(".wpilog") || f.getName().endsWith(".hoot")) {
        out.add(f);
      }
    }
  }

  private static void pruneEmptySubdirectories(File dir, boolean isRoot) {
    File[] children = dir.listFiles();
    if (children == null) {
      return;
    }
    for (File f : children) {
      if (f.isDirectory()) {
        pruneEmptySubdirectories(f, false);
      }
    }
    if (!isRoot) {
      File[] remaining = dir.listFiles();
      if (remaining != null && remaining.length == 0) {
        dir.delete();
      }
    }
  }
}
