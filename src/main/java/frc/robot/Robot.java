package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.fluxfig.FluxFig;
import frc.lib.util.LogFileManager;
import frc.lib.util.LoggedTracer;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * LoggedRobot lifecycle for {@code Flux-RoboRIO-2027} ({@code FUEL} / {@code CORAL} / {@code
 * CHASSIS_3}): AdvantageKit {@link Mode#REAL}, {@link Mode#SIM}, and deterministic {@link
 * Mode#REPLAY} ({@link WPILOGReader} + {@code setUseTiming(false)}), 50 Hz control loop via
 * FluxFig, {@code ConfigSHA256} metadata recorded before {@link Logger#start()}, {@code 6.0 V}
 * roboRIO brownout floor, {@link LoggedPowerDistribution} logging, atomic {@code
 * /tmp/flux-robot-state} lockout heartbeat for {@code FluxTray}, batched CAN signal refresh,
 * main-thread RT priority {@code 1}, and isolated {@link #testInit()}.
 */
public class Robot extends LoggedRobot {

  /** AdvantageKit robot execution mode. */
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  /** Safe SCHED_FIFO priority (1 rather than 99) preventing Linux RT worker starvation (-1003). */
  public static final int MAIN_THREAD_RT_PRIORITY = 1;

  /** Lowered roboRIO 2.0 brownout threshold (6.0 V vs. 6.8 V default) for CTRE swerve sag margin. */
  public static final double ROBORIO_BROWNOUT_VOLTAGE_V = 6.0;

  /** Atomic robot-side lockout state file checked by {@code FluxTray} SSH pre-flight guards. */
  public static final Path DEFAULT_ROBOT_STATE_FILE = Path.of("/tmp/flux-robot-state");

  /** Monotonic 250 ms pacing interval for {@code /tmp/flux-robot-state} heartbeat writes. */
  public static final double ROBOT_STATE_HEARTBEAT_INTERVAL_SEC = 0.25;

  private static boolean loggerStarted = false;

  private final Path robotStateFile;
  private final Mode mode;
  private final Path replayLogPath;
  private final Path replayOutputPath;
  private RobotIdentity identity;
  private RobotContainer robotContainer;
  private Command autonomousCommand;
  private String logDir;
  private String deployedGitSha = "unknown";
  private String bootTimestampIso = "";
  private boolean lastHeartbeatEnabled = false;
  private boolean lastHeartbeatFms = false;
  private double lastHeartbeatFpgaSec = Double.NEGATIVE_INFINITY;
  private Boolean lastJoystickWarningSilenced = null;

  public Robot() {
    this(DEFAULT_ROBOT_STATE_FILE);
  }

  /** Deterministic constructor allowing an isolated state-file path in unit/sim tests. */
  public Robot(Path robotStateFile) {
    this(robotStateFile, resolveReplayLogPathFromProperties(), resolveReplayOutPathFromProperties());
  }

  /** Deterministic constructor supporting explicit AdvantageKit {@link Mode#REPLAY} log paths. */
  public Robot(Path robotStateFile, Path replayLogPath, Path replayOutputPath) {
    super(resolveBootLoopPeriodSeconds());
    this.robotStateFile = (robotStateFile != null) ? robotStateFile : DEFAULT_ROBOT_STATE_FILE;
    this.replayLogPath = RobotBase.isReal() ? null : replayLogPath;
    this.replayOutputPath = RobotBase.isReal() ? null : replayOutputPath;
    this.mode = resolveMode(RobotBase.isReal(), this.replayLogPath);
  }

  public static Mode resolveMode(boolean isRealRobot, Path replayLogPath) {
    if (isRealRobot) {
      return Mode.REAL;
    }
    return (replayLogPath != null) ? Mode.REPLAY : Mode.SIM;
  }

  private static Path resolveReplayLogPathFromProperties() {
    String prop = System.getProperty("flux.replayLog");
    if (prop == null || prop.isBlank()) {
      prop = System.getenv("FLUX_REPLAY_LOG");
    }
    return (prop != null && !prop.isBlank()) ? Path.of(prop.trim()) : null;
  }

  private static Path resolveReplayOutPathFromProperties() {
    String prop = System.getProperty("flux.replayOutLog");
    if (prop == null || prop.isBlank()) {
      prop = System.getenv("FLUX_REPLAY_OUT_LOG");
    }
    return (prop != null && !prop.isBlank()) ? Path.of(prop.trim()) : null;
  }

  private static boolean shouldWriteSimWpilog() {
    String prop = System.getProperty("flux.simWriteWpilog", "true");
    return !"false".equalsIgnoreCase(prop.trim());
  }

  private static Path deriveDefaultReplayOutputPath(Path replayInput) {
    String name = replayInput.getFileName().toString();
    int dot = name.lastIndexOf('.');
    String stem = (dot > 0) ? name.substring(0, dot) : name;
    String ext = (dot > 0) ? name.substring(dot) : ".wpilog";
    Path parent = replayInput.getParent();
    return (parent != null) ? parent.resolve(stem + "_sim" + ext) : Path.of(stem + "_sim" + ext);
  }

  private static double resolveBootLoopPeriodSeconds() {
    try {
      RobotIdentity bootId = RobotIdentity.detect();
      FluxFig fig =
          FluxFig.load(
              Filesystem.getDeployDirectory().toPath().resolve("configs"), bootId.name());
      return Math.max(0.002, fig.getInt("loopPeriodMs") / 1000.0);
    } catch (Exception e) {
      return 0.020;
    }
  }

  @Override
  public void robotInit() {
    // Keep CTRE .hoot auto-logging OFF (WPILOG-only to avoid flash/USB bloat).
    SignalLogger.enableAutoLogging(false);
    if (mode != Mode.REPLAY) {
      RobotController.setBrownoutVoltage(ROBORIO_BROWNOUT_VOLTAGE_V);
      LiveWindow.disableAllTelemetry();
      updateJoystickWarningSilence();
    }

    logDir = LogFileManager.getLogPath();
    bootTimestampIso = Instant.now().toString();
    Path deployDir = Filesystem.getDeployDirectory().toPath();
    Path configDir = deployDir.resolve("configs");
    deployedGitSha = LogFileManager.readDeployedGitSha(deployDir);
    identity = RobotIdentity.detect();
    FluxFig bootConfig = FluxFig.load(configDir, identity.name());
    FluxFig.ActiveProfileDirective bootDirective =
        RobotContainer.readActiveProfileDirective(configDir);
    String bootConfigChannel = RobotContainer.resolveBootConfigChannel(bootDirective, bootConfig);
    String bootConfigPreset = RobotContainer.resolveBootConfigPreset(bootDirective, bootConfig);

    // Record all immutable boot metadata BEFORE Logger.start() so WPILOG headers capture it.
    Logger.recordMetadata("RobotMode", mode.name());
    Logger.recordMetadata("LogDirectory", logDir);
    Logger.recordMetadata("GitSHA", deployedGitSha);
    Logger.recordMetadata("RobotIdentity", identity.name());
    Logger.recordMetadata("RobotIdentitySource", RobotIdentity.getLastResolutionSource());
    Logger.recordMetadata("ConfigSHA256", bootConfig.getConfigSha256());
    Logger.recordMetadata("FluxFigProfile", bootConfig.getActiveProfile());
    Logger.recordMetadata("FluxFigVersion", bootConfig.getEffectiveVersion());
    Logger.recordMetadata("ConfigChannel", bootConfigChannel);
    Logger.recordMetadata("ConfigPreset", bootConfigPreset);

    if (!loggerStarted) {
      switch (mode) {
        case REAL -> {
          Logger.addDataReceiver(new WPILOGWriter(logDir));
          Logger.addDataReceiver(new NT4Publisher());
        }
        case SIM -> {
          Logger.addDataReceiver(new NT4Publisher());
          if (shouldWriteSimWpilog()) {
            Logger.addDataReceiver(new WPILOGWriter(logDir));
          }
        }
        case REPLAY -> {
          setUseTiming(false);
          Logger.setReplaySource(new WPILOGReader(replayLogPath.toString()));
          if (!"false".equalsIgnoreCase(System.getProperty("flux.replayWriteOutput", "true"))) {
            Path outLog =
                (replayOutputPath != null)
                    ? replayOutputPath
                    : deriveDefaultReplayOutputPath(replayLogPath);
            Logger.addDataReceiver(new WPILOGWriter(outLog.toString()));
          }
        }
      }
      Logger.start();
      loggerStarted = true;
    }

    if (mode != Mode.REPLAY) {
      LoggedPowerDistribution.getInstance();
    }

    robotContainer = new RobotContainer(identity, configDir, bootConfig, mode);
    robotContainer.getDrive().setLoopPeriodSeconds(getPeriod());

    Logger.recordOutput("Phoenix/CanConfigFailures", PhoenixUtil.getFailedConfigCallCount());
    Logger.recordOutput("Phoenix/CanConfigSkipped", PhoenixUtil.getSkippedConfigCallCount());

    if (mode == Mode.REAL) {
      Threads.setCurrentThreadPriority(true, MAIN_THREAD_RT_PRIORITY);
      Logger.recordOutput("Robot/MainThreadPriority", Threads.getCurrentThreadPriority());
    }

    if (mode != Mode.REPLAY && !Logger.hasReplaySource()) {
      robotContainer.warmupBeforeCompetitionLoop();
      refreshRunMetaJson();
      writeRobotStateHeartbeat(true);
    }
  }

  private void updateJoystickWarningSilence() {
    boolean silence = !DriverStation.isFMSAttached();
    if (lastJoystickWarningSilenced == null || lastJoystickWarningSilenced.booleanValue() != silence) {
      DriverStation.silenceJoystickConnectionWarning(silence);
      lastJoystickWarningSilenced = silence;
    }
  }

  private void writeRobotStateHeartbeat(boolean force) {
    writeRobotStateHeartbeat(force, Timer.getFPGATimestamp(), System.currentTimeMillis());
  }

  /**
   * Writes atomic {@code /tmp/flux-robot-state} paced by monotonic FPGA time so backward RTC steps
   * never freeze the heartbeat, while still stamping wall-clock {@code EPOCH_MS}.
   */
  public void writeRobotStateHeartbeat(boolean force, double fpgaTimestampSec, long wallEpochMs) {
    boolean enabled = DriverStation.isEnabled();
    boolean fms = DriverStation.isFMSAttached();
    double elapsedFpgaSec = fpgaTimestampSec - lastHeartbeatFpgaSec;
    if (!force
        && enabled == lastHeartbeatEnabled
        && fms == lastHeartbeatFms
        && elapsedFpgaSec >= 0.0
        && elapsedFpgaSec < ROBOT_STATE_HEARTBEAT_INTERVAL_SEC) {
      return;
    }
    try {
      LogFileManager.writeRobotStateFile(
          robotStateFile,
          enabled,
          fms,
          wallEpochMs,
          identity != null ? identity.name() : "UNKNOWN",
          ProcessHandle.current().pid());
      lastHeartbeatEnabled = enabled;
      lastHeartbeatFms = fms;
      lastHeartbeatFpgaSec = fpgaTimestampSec;
    } catch (IOException ignored) {
    }
  }

  private void refreshRunMetaJson() {
    if (mode == Mode.REPLAY || logDir == null || identity == null || robotContainer == null) {
      return;
    }
    try {
      LogFileManager.writeRunMetaJson(
          Path.of(logDir),
          identity.name(),
          robotContainer.getFluxFig().getActiveProfile(),
          robotContainer.getBootConfigSha256(),
          robotContainer.getBatteryMonitor().getResolvedBatteryName(),
          robotContainer.getFluxFig().getInt("loopPeriodMs"),
          deployedGitSha,
          bootTimestampIso,
          robotContainer.getBootConfigChannel(),
          robotContainer.getBootConfigPreset());
    } catch (IOException ignored) {
    }
  }

  @Override
  public void robotPeriodic() {
    double loopStartSec = Timer.getFPGATimestamp();
    LoggedTracer.reset();
    if (mode != Mode.REPLAY) {
      updateJoystickWarningSilence();
      writeRobotStateHeartbeat(false);
    }
    if (mode != Mode.REPLAY && !Logger.hasReplaySource()) {
      PhoenixSignals.refreshAll();
    }
    LoggedTracer.trace("PhoenixSignals");
    CommandScheduler.getInstance().run();
    LoggedTracer.trace("CommandScheduler");
    if (robotContainer != null) {
      robotContainer.periodicUpdate();
      LoggedTracer.trace("RobotContainer");
    }
    Logger.recordOutput(
        "Performance/TotalLoopMs", (Timer.getFPGATimestamp() - loopStartSec) * 1000.0);
  }

  @Override
  public void disabledInit() {
    if (mode != Mode.REPLAY) {
      writeRobotStateHeartbeat(true);
    }
    if (robotContainer != null) {
      robotContainer.resetSuperstructureToIdle();
      robotContainer.disabledPeriodic();
      refreshRunMetaJson();
    }
  }

  @Override
  public void disabledPeriodic() {
    if (robotContainer != null) {
      robotContainer.disabledPeriodic();
    }
  }

  @Override
  public void autonomousInit() {
    if (mode != Mode.REPLAY) {
      writeRobotStateHeartbeat(true);
    }
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    if (robotContainer != null) {
      autonomousCommand = robotContainer.getAutonomousCommand();
      if (autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(autonomousCommand);
      }
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (mode != Mode.REPLAY) {
      writeRobotStateHeartbeat(true);
    }
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }
    if (robotContainer != null) {
      robotContainer.resetSuperstructureToIdle();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }
    if (robotContainer != null) {
      robotContainer.resetSuperstructureToIdle();
    }
    if (mode != Mode.REPLAY) {
      writeRobotStateHeartbeat(true);
    }
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void close() {
    if (robotContainer != null) {
      robotContainer.close();
    }
    if (mode != Mode.REPLAY) {
      try {
        Files.deleteIfExists(robotStateFile);
      } catch (IOException ignored) {
      }
    }
    super.close();
  }

  public Mode getMode() {
    return mode;
  }

  public RobotContainer getRobotContainer() {
    return robotContainer;
  }

  public Path getRobotStateFile() {
    return robotStateFile;
  }

  public String getLogDir() {
    return logDir;
  }
}
