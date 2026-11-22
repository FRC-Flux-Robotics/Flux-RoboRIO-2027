package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.lib.drive.GyroIOPigeon2;
import frc.lib.drive.GyroIOSim;
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIOSim;
import frc.lib.drive.ModuleIOTalonFX;
import frc.lib.drive.PhoenixOdometryThread;
import frc.lib.drive.SwerveConfig;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.ActiveProfileDirective;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIOSim;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.BatteryIO;
import frc.lib.safety.BatteryIOBfgCan;
import frc.lib.safety.BatteryIOSim;
import frc.lib.safety.BatteryMonitor;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.Nt4FluxFigTuningBackend;
import frc.lib.util.PhoenixUtil;
import frc.lib.util.PiecewiseSensitivity;
import frc.lib.util.VirtualBatterySim;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIOLimelightSystemCore;
import frc.lib.vision.VisionIOPhotonVision;
import frc.lib.vision.VisionIOSim;
import frc.robot.Superstructure.State;
import frc.robot.autos.StudentAutoRoutines;
import frc.robot.commands.ShotCalibrationCmd;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Unified 2026 roboRIO robot container configured dynamically by {@link RobotIdentity} and {@link
 * FluxFig} CSVs (eliminating legacy {@code FuelRobotContainer} subclassing).
 *
 * <p>Consumes the auto-synced {@code frc.lib.*} core engine (`Drive`, `Vision`, `BatteryIO`, and
 * `frc.lib.commands.*`) while managing 2026 {@code FUEL}-specific shooting mechanisms (`Intake`,
 * `Tilter`, `Indexer`, `Feeder`, dual `Shooter`, `Hood`), {@link ShotCalculator}, and {@link
 * ShotCalibrationCmd} (plus Ghost Superstructure mode on bare chassis {@code CORAL} / {@code
 * CHASSIS_3}).
 */
public class RobotContainer {
  /**
   * Unified top-level robot operating mode:
   *
   * <ul>
   *   <li>{@link #COMPETITION}: Default ({@code FluxFig.Mode.PRODUCTION}) and automatically forced
   *       whenever connected to FMS ({@code DriverStation.isFMSAttached()}). Locks {@link FluxFig}
   *       and {@link LoggedTunableNumber} to verified Git/CSV values, forces {@link
   *       FieldPositions.FieldProfile#FULL_FIELD} (geofence OFF), and disables pit/blitz test
   *       button bindings while on FMS.
   *   <li>{@link #TRAINING}: Active in shop/practice when {@code FluxFig.Mode.TUNING} is enabled
   *       and not connected to FMS. Enables live dashboard tunables, custom/auto-measured shop or
   *       half-field geofence bounds, and 1-button pit/blitz routines.
   * </ul>
   */
  public enum OperatingMode {
    COMPETITION,
    TRAINING
  }

  /** Blue Alliance Octagonal Hub center in 2026 Wall/Blue coordinates (derived from AprilTag centroid). */
  public static final Translation2d HUB_TARGET_METERS = new Translation2d(4.626, 4.035);

  /** Canonical Blue Alliance Hub shooting alignment pose in 2026 Wall/Blue coordinates. */
  public static final Pose2d BLUE_HUB_ALIGN_POSE = new Pose2d(2.5, 4.1, Rotation2d.kZero);

  /** Safe do-nothing autonomous default that never overwrites odometry or commands drive output. */
  public static final String SAFE_DEFAULT_AUTO_NAME = "Do Nothing (Safe Default)";

  /** Safe low open-loop voltage used for Teleop D-pad manual jogging of Hood and Tilter. */
  public static final double MANUAL_JOG_VOLTS = 1.8;

  private static final int CONFIG_DISK_POLL_INTERVAL_LOOPS = 25;

  private final RobotIdentity identity;
  private final Path configDir;
  private final FluxFig fluxFig;
  private final String bootConfigSha256;
  private final String bootEffectiveVersion;
  private final String bootEffectiveSnapshot;
  private final String bootConfigChannel;
  private final String bootConfigPreset;
  private final long bootDiskFingerprint;
  private final FaultSupervisor faultSupervisor;
  private final BatteryIO batteryIo;
  private final BatteryMonitor batteryMonitor;
  private final VirtualBatterySim virtualBatterySim;
  private final ShotCalculator shotCalculator;
  private final PiecewiseSensitivity joystickSensitivity;
  private final PiecewiseSensitivity rotationSensitivity;

  private final Drive drive;
  private final Vision vision;
  private final int cameraCount;
  private final PreMatchCalibrator preMatchCalibrator;

  private final VelocityMechanism intake;
  private final PositionMechanism tilter;
  private final VelocityMechanism indexer;
  private final VelocityMechanism feeder;
  private final VelocityMechanism shooter;
  private final PositionMechanism hood;
  private final Superstructure superstructure;
  private final boolean ghostSuperstructureActive;
  private final Robot.Mode robotMode;
  private final boolean forceSimIo;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final FluxFig.TuningBackend externalTuningBackend;
  private Nt4FluxFigTuningBackend ownedNt4TuningBackend;
  private boolean tuningBackendEnabled = false;
  private boolean wasTrainingMode = false;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private ShotSolution latestTeleopShotSolution =
      new ShotSolution(Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0);
  private boolean configRestartRequired = false;
  private int disabledConfigPollCounter = 0;

  public RobotContainer(RobotIdentity identity) {
    this(
        identity,
        Filesystem.getDeployDirectory().toPath().resolve("configs"),
        null,
        RobotBase.isReal() ? Robot.Mode.REAL : Robot.Mode.SIM,
        null,
        null);
  }

  /** Deterministic constructor allowing explicit config directory and simulated IO selection. */
  public RobotContainer(RobotIdentity identity, Path configDir, boolean forceSimIo) {
    this(
        identity,
        configDir,
        null,
        forceSimIo ? Robot.Mode.SIM : Robot.Mode.REAL,
        null,
        null);
  }

  /** Constructor accepting a pre-loaded boot {@link FluxFig} snapshot to avoid duplicate disk loads. */
  public RobotContainer(
      RobotIdentity identity, Path configDir, FluxFig preloadedFluxFig, boolean forceSimIo) {
    this(
        identity,
        configDir,
        preloadedFluxFig,
        forceSimIo ? Robot.Mode.SIM : Robot.Mode.REAL,
        null,
        null);
  }

  /** Constructor accepting an explicit AdvantageKit {@link Robot.Mode} ({@code REAL}/{@code SIM}/{@code REPLAY}). */
  public RobotContainer(
      RobotIdentity identity, Path configDir, FluxFig preloadedFluxFig, Robot.Mode robotMode) {
    this(identity, configDir, preloadedFluxFig, robotMode, null, null);
  }

  /**
   * Full deterministic constructor supporting custom {@link FluxFig.TuningBackend} and temporary
   * tuned-snapshot path injection for live tuning integration tests.
   */
  public RobotContainer(
      RobotIdentity identity,
      Path configDir,
      FluxFig preloadedFluxFig,
      boolean forceSimIo,
      FluxFig.TuningBackend tuningBackend,
      Path tmpTunedSnapshotPath) {
    this(
        identity,
        configDir,
        preloadedFluxFig,
        forceSimIo ? Robot.Mode.SIM : Robot.Mode.REAL,
        tuningBackend,
        tmpTunedSnapshotPath);
  }

  public RobotContainer(
      RobotIdentity identity,
      Path configDir,
      FluxFig preloadedFluxFig,
      Robot.Mode robotMode,
      FluxFig.TuningBackend tuningBackend,
      Path tmpTunedSnapshotPath) {
    this.identity = identity;
    this.configDir = configDir;
    this.robotMode = (robotMode != null) ? robotMode : Robot.Mode.SIM;
    this.forceSimIo = (this.robotMode == Robot.Mode.SIM);
    this.externalTuningBackend = tuningBackend;
    this.fluxFig =
        (preloadedFluxFig != null) ? preloadedFluxFig : FluxFig.load(configDir, identity.name());
    this.fluxFig.setTmpTunedSnapshotPath(
        tmpTunedSnapshotPath != null
            ? tmpTunedSnapshotPath
            : FluxFig.DEFAULT_TMP_TUNED_SNAPSHOT_PATH);
    ActiveProfileDirective bootDirective = readActiveProfileDirective(configDir);
    this.bootConfigChannel = resolveBootConfigChannel(bootDirective, this.fluxFig);
    this.bootConfigPreset = resolveBootConfigPreset(bootDirective, this.fluxFig);
    this.bootConfigSha256 = this.fluxFig.getConfigSha256();
    this.bootEffectiveVersion = this.fluxFig.getEffectiveVersion();
    this.bootEffectiveSnapshot = this.fluxFig.getEffectiveSnapshot();
    this.bootDiskFingerprint = computeConfigDiskFingerprint(configDir);

    String coordSysName = fluxFig.getString("coordinateSystem");
    FieldPositions.setCoordinateSystem(
        "CENTER_RED_2027".equalsIgnoreCase(coordSysName)
            ? FieldPositions.CoordinateSystem.CENTER_RED_2027
            : FieldPositions.CoordinateSystem.WALL_BLUE_2026);
    applyFieldAndArenaConfig();

    boolean simMissingMechs = fluxFig.getBoolean("simulateMissingMechanisms");
    this.ghostSuperstructureActive =
        simMissingMechs
            || identity == RobotIdentity.CORAL
            || identity == RobotIdentity.CHASSIS_3;

    double loopPeriodSec = Math.max(0.002, fluxFig.getInt("loopPeriodMs") / 1000.0);
    CameraSpec[] camConfigs =
        fluxFig.getCamerasForRobot(identity.name()).toArray(new CameraSpec[0]);
    this.cameraCount = camConfigs.length;
    this.faultSupervisor = new FaultSupervisor(Math.max(1, camConfigs.length));
    this.virtualBatterySim = new VirtualBatterySim(12.6, 0.015);
    String auxCanBus = fluxFig.getString("auxCanBus");
    this.batteryIo =
        switch (this.robotMode) {
          case REAL -> new BatteryIOBfgCan(auxCanBus);
          case SIM -> new BatteryIOSim(auxCanBus, this.virtualBatterySim);
          case REPLAY -> new BatteryIO() {
            @Override
            public String getAuxCanBus() {
              return auxCanBus;
            }
          };
        };
    this.batteryMonitor = new BatteryMonitor(this.batteryIo, fluxFig);
    this.shotCalculator = ShotCalculator.fromFluxFig(fluxFig);
    double initialJoystickScale =
        fluxFig.hasParameter("joystickSensitivity")
            ? MathUtil.clamp(fluxFig.getDouble("joystickSensitivity"), 0.05, 1.0)
            : 0.80;
    double initialRotationScale =
        fluxFig.hasParameter("rotationSensitivity")
            ? MathUtil.clamp(fluxFig.getDouble("rotationSensitivity"), 0.05, 1.0)
            : 1.00;
    this.joystickSensitivity =
        new PiecewiseSensitivity(
            "Drive/JoystickCurve", 0.09, 0.60, 0.10, 0.30, initialJoystickScale);
    this.rotationSensitivity =
        new PiecewiseSensitivity(
            "Drive/RotationCurve", 0.09, 0.60, 0.10, 0.50, initialRotationScale);

    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fluxFig, identity.name());
    GyroIO gyroIo = new GyroIO() {};
    ModuleIO[] moduleIos = new ModuleIO[4];
    switch (this.robotMode) {
      case REPLAY -> {
        gyroIo = new GyroIO() {};
        for (int i = 0; i < 4; i++) {
          moduleIos[i] = new ModuleIO() {};
        }
      }
      case SIM -> {
        ModuleIOSim[] simModules = new ModuleIOSim[4];
        for (int i = 0; i < 4; i++) {
          simModules[i] = new ModuleIOSim(swerveConfig, virtualBatterySim::terminalVoltageV);
          simModules[i].setLoopPeriodSeconds(loopPeriodSec);
          moduleIos[i] = simModules[i];
        }
        gyroIo = new GyroIOSim(swerveConfig, simModules, virtualBatterySim::terminalVoltageV);
      }
      case REAL -> {
        PhoenixOdometryThread.getInstance().setOdometryFrequencyHz(swerveConfig.odometryFrequencyHz());
        PhoenixUtil.beginCanInitBudget();
        gyroIo = new GyroIOPigeon2(swerveConfig.pigeonId(), swerveConfig.canBus());
        for (int i = 0; i < 4; i++) {
          moduleIos[i] = new ModuleIOTalonFX(swerveConfig, swerveConfig.corners()[i]);
        }
      }
    }
    this.drive = new Drive(swerveConfig, gyroIo, moduleIos);
    this.drive.setLoopPeriodSeconds(loopPeriodSec);

    VisionIO[] visionIos = new VisionIO[camConfigs.length];
    if (camConfigs.length > 0) {
      String visionBackend = fluxFig.getString("visionBackend");
      if (this.robotMode == Robot.Mode.REPLAY) {
        for (int i = 0; i < camConfigs.length; i++) {
          visionIos[i] = new VisionIO() {};
        }
      } else if (this.robotMode == Robot.Mode.SIM) {
        for (int i = 0; i < camConfigs.length; i++) {
          visionIos[i] = new VisionIOSim(camConfigs[i], drive::getPose);
        }
      } else if ("LIMELIGHT_SYSTEMCORE_USB".equalsIgnoreCase(visionBackend)) {
        for (int i = 0; i < camConfigs.length; i++) {
          visionIos[i] = new VisionIOLimelightSystemCore(camConfigs[i]);
        }
      } else {
        AprilTagFieldLayout fieldLayout =
            FieldPositions.configureLayoutOrigin(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
        for (int i = 0; i < camConfigs.length; i++) {
          visionIos[i] = new VisionIOPhotonVision(camConfigs[i], fieldLayout);
        }
      }
    }
    this.vision = new Vision(drive::addVisionMeasurement, faultSupervisor, camConfigs, visionIos);
    this.preMatchCalibrator = new PreMatchCalibrator(drive, vision);

    boolean useRealMechanisms = this.robotMode == Robot.Mode.REAL && !ghostSuperstructureActive;
    if (useRealMechanisms) {
      // Refresh boot CAN retry budget after potentially slow AprilTagFieldLayout loading.
      PhoenixUtil.beginCanInitBudget();
    }
    String mechProfile =
        (identity == RobotIdentity.FUEL) ? fluxFig.getActiveProfile() : "FUEL";

    MechanismSpec intakeSpec = fluxFig.getMechanismSpec(mechProfile, "Intake");
    this.intake =
        new VelocityMechanism(
            createMechanismIo(intakeSpec, useRealMechanisms, false, loopPeriodSec), intakeSpec);

    MechanismSpec tilterSpec = fluxFig.getMechanismSpec(mechProfile, "Tilter");
    this.tilter =
        new PositionMechanism(
            createMechanismIo(tilterSpec, useRealMechanisms, false, loopPeriodSec), tilterSpec);

    MechanismSpec indexerSpec = fluxFig.getMechanismSpec(mechProfile, "Indexer");
    this.indexer =
        new VelocityMechanism(
            createMechanismIo(indexerSpec, useRealMechanisms, false, loopPeriodSec), indexerSpec);

    MechanismSpec feederSpec = fluxFig.getMechanismSpec(mechProfile, "Feeder");
    this.feeder =
        new VelocityMechanism(
            createMechanismIo(feederSpec, useRealMechanisms, false, loopPeriodSec), feederSpec);

    MechanismSpec shooterSpec = fluxFig.getMechanismSpec(mechProfile, "Shooter");
    this.shooter =
        new VelocityMechanism(
            createMechanismIo(shooterSpec, useRealMechanisms, true, loopPeriodSec), shooterSpec);

    MechanismSpec hoodSpec = fluxFig.getMechanismSpec(mechProfile, "Hood");
    this.hood =
        new PositionMechanism(
            createMechanismIo(hoodSpec, useRealMechanisms, false, loopPeriodSec),
            hoodSpec,
            Superstructure.HOOD_TOLERANCE_ROT);

    this.superstructure =
        new Superstructure(
            this.intake,
            this.tilter,
            this.indexer,
            this.feeder,
            this.shooter,
            this.hood,
            this.faultSupervisor);

    registerLiveDrivetrainTunables();
    if (this.fluxFig.getMode() == FluxFig.Mode.TUNING && !DriverStation.isFMSAttached()) {
      enableLiveTuningBackend();
      this.wasTrainingMode = true;
    }

    registerNamedCommands();
    this.autoChooser = new LoggedDashboardChooser<>("Auto/Chooser");
    configureAutoChooser();
    configureButtonBindings();

    Logger.recordOutput("FluxFig/BootConfigSHA256", bootConfigSha256);
    Logger.recordOutput("FluxFig/EffectiveVersion", bootEffectiveVersion);
    Logger.recordOutput("FluxFig/EffectiveSnapshot", bootEffectiveSnapshot);
    Logger.recordOutput("FluxFig/ConfigChannel", bootConfigChannel);
    Logger.recordOutput("FluxFig/ConfigPreset", bootConfigPreset);
    Logger.recordOutput("FluxFig/Modified", false);
    Logger.recordOutput("FluxFig/RestartRequired", false);
    Logger.recordOutput("Drive/OdometryFrequencyHz", swerveConfig.odometryFrequencyHz());
    Logger.recordOutput("Vision/CameraCount", cameraCount);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "intake",
        Commands.runOnce(() -> superstructure.requestState(State.INTAKING), superstructure));
    NamedCommands.registerCommand(
        "startIntake",
        Commands.runOnce(() -> superstructure.requestState(State.INTAKING), superstructure));
    NamedCommands.registerCommand(
        "deployIntake",
        Commands.runOnce(() -> superstructure.requestState(State.INTAKING), superstructure));
    NamedCommands.registerCommand(
        "idle", Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure));
    NamedCommands.registerCommand(
        "stopIntake",
        Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure));
    NamedCommands.registerCommand(
        "stowIntake",
        Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure));
    NamedCommands.registerCommand(
        "stopAll", Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure));
    NamedCommands.registerCommand(
        "setShortRange",
        Commands.runOnce(
            () ->
                superstructure.prepShot(
                    Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                    Superstructure.DEFAULT_SHORT_HOOD_ROT),
            superstructure));
    NamedCommands.registerCommand(
        "feed",
        Commands.runOnce(
            () ->
                superstructure.shoot(
                    Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                    Superstructure.DEFAULT_SHORT_HOOD_ROT),
            superstructure));
    NamedCommands.registerCommand(
        "outtake",
        Commands.runOnce(() -> superstructure.requestState(State.OUTTAKING), superstructure));
    NamedCommands.registerCommand("homeHood", new HardStopHomingCmd(hood));
  }

  private void configureAutoChooser() {
    autoChooser.addDefaultOption(
        SAFE_DEFAULT_AUTO_NAME, Commands.none().withName(SAFE_DEFAULT_AUTO_NAME));
    var namedAutos = StudentAutoRoutines.createNamedAutos(drive, superstructure, shotCalculator);
    for (var entry : namedAutos.entrySet()) {
      autoChooser.addOption(entry.getKey(), entry.getValue());
    }
    autoChooser.addOption("Hub to Depot PathPlanner Path", drive.followPathCommand("Hub to Depot"));
    autoChooser.addOption(
        "Hub DriveToPose (2.5m, 4.1m)",
        Commands.defer(
            () -> new DriveToPose(drive, getHubAlignTargetPose(FieldPositions.isRedAlliance())),
            Set.of(drive)));
    autoChooser.addOption(
        "Shop 5x3 Shuttle Path", drive.followPathCommand("Shop_Center_Shuttle"));
    autoChooser.addOption(
        "Pre-Kickoff 2-Piece Template", drive.followPathCommand("PreKickoff_2Piece_Template"));
    autoChooser.addOption(
        "Pit System Check (20s)",
        createPitSystemCheckCommand().onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "Wheel Radius Characterization (Spin)",
        DriveCommands.wheelRadiusCharacterization(drive).onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Translation Quasistatic Fwd",
        DriveCommands.sysIdTranslationQuasistatic(drive, SysIdRoutine.Direction.kForward)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Translation Quasistatic Rev",
        DriveCommands.sysIdTranslationQuasistatic(drive, SysIdRoutine.Direction.kReverse)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Translation Dynamic Fwd",
        DriveCommands.sysIdTranslationDynamic(drive, SysIdRoutine.Direction.kForward)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Translation Dynamic Rev",
        DriveCommands.sysIdTranslationDynamic(drive, SysIdRoutine.Direction.kReverse)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Steer Quasistatic Fwd",
        DriveCommands.sysIdSteerQuasistatic(drive, SysIdRoutine.Direction.kForward)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Steer Dynamic Fwd",
        DriveCommands.sysIdSteerDynamic(drive, SysIdRoutine.Direction.kForward)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Rotation Quasistatic Fwd",
        DriveCommands.sysIdRotationQuasistatic(drive, SysIdRoutine.Direction.kForward)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "SysId Rotation Dynamic Fwd",
        DriveCommands.sysIdRotationDynamic(drive, SysIdRoutine.Direction.kForward)
            .onlyIf(RobotContainer::isPitTestAllowed));
    autoChooser.addOption(
        "15-Min Floor Blitz",
        createBlitz15MinCommand(Blitz15MinCmd.DEFAULT_CHECKPOINT_PATH)
            .onlyIf(RobotContainer::isPitTestAllowed));
  }

  private MechanismIO createMechanismIo(
      MechanismSpec spec, boolean useRealHardware, boolean coastNeutral, double loopPeriodSec) {
    if (robotMode == Robot.Mode.REPLAY) {
      return new MechanismIO() {};
    }
    if (!useRealHardware) {
      MechanismIOSim sim = new MechanismIOSim(spec);
      sim.setLoopPeriodSeconds(loopPeriodSec);
      return sim;
    }
    if (coastNeutral) {
      return new MechanismIOTalonFX(
          spec, spec.followerMotorId(), spec.invertFollower(), NeutralModeValue.Coast);
    }
    return new MechanismIOTalonFX(spec);
  }

  /** Pit, calibration, and homing triggers require non-FMS WPILib Test mode. */
  public static boolean isPitTestAllowed() {
    return !DriverStation.isFMSAttached() && DriverStation.isTest();
  }

  /**
   * Returns the alliance-mirrored Hub shooting alignment pose using 2026 REBUILT 180-degree
   * rotational symmetry {@code (L - x, W - y, theta + pi)} when on Red Alliance.
   */
  public static Pose2d getHubAlignTargetPose(boolean isRedAlliance) {
    Pose2d activeBlue = FieldPositions.fromWallBlue(BLUE_HUB_ALIGN_POSE);
    return isRedAlliance ? FieldPositions.mirrorRotational(activeBlue) : activeBlue;
  }

  /** Resets field-oriented heading to alliance forward while preserving current X/Y translation. */
  public void resetFieldHeading() {
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), FieldPositions.operatorForward()));
    if (preMatchCalibrator != null) {
      preMatchCalibrator.onManualHeadingReset();
    }
  }

  /** Resets the 6-mechanism superstructure to {@link State#IDLE} across mode transitions. */
  public void resetSuperstructureToIdle() {
    superstructure.requestState(State.IDLE);
  }

  /** Zeroes relative encoders for {@code Tilter} and {@code Hood} at their physically stowed stops. */
  public void resetMechanismEncoders() {
    tilter.stop();
    hood.stop();
    boolean tilterOk = tilter.resetPosition();
    boolean hoodOk = hood.resetPosition();
    Logger.recordOutput("Superstructure/EncodersZeroed", tilterOk && hoodOk);
  }

  /** Creates a Teleop-safe manual open-loop jog command that stops cleanly on button release. */
  public Command createManualJogCommand(PositionMechanism mechanism, double volts) {
    return Commands.run(() -> mechanism.jogVoltage(volts), mechanism, superstructure)
        .beforeStarting(this::resetSuperstructureToIdle)
        .finallyDo(mechanism::stop);
  }

  /**
   * Manual close-range shoot fallback ({@code 33.33 RPS}, {@code 0.0 rot} hood): waits for
   * flywheel and hood tolerances before feeding without requiring AprilTag localization.
   */
  public Command createManualShortShotCommand() {
    return Commands.run(
        () ->
            superstructure.shoot(
                Superstructure.DEFAULT_SHORT_SHOOTER_RPS,
                Superstructure.DEFAULT_SHORT_HOOD_ROT,
                0.0,
                true),
        superstructure);
  }

  /**
   * Selects the active battery voltage fed to {@link FaultSupervisor}: physical roboRIO hardware
   * always uses {@link RobotController#getBatteryVoltage()} even when Ghost Superstructure mode is
   * active on {@code CORAL} / {@code CHASSIS_3}.
   */
  public static double selectEffectiveBatteryVoltage(
      boolean forceSimIo,
      boolean isRealRobot,
      double realBatteryVoltageV,
      double simBatteryVoltageV) {
    return (!forceSimIo && isRealRobot) ? realBatteryVoltageV : simBatteryVoltageV;
  }

  /**
   * Selects the battery internal resistance (mOhm) fed to {@link FaultSupervisor}: when the inline
   * CAN battery gauge is connected, always trust its measured {@code R_int} regardless of health
   * status (so weak batteries > 22 mOhm or < 75% SoC still derate); otherwise use the nominal
   * {@code batteries.csv} resistance.
   */
  public static double selectEffectiveRintMilliOhms(
      boolean batteryConnected,
      double batteryGaugeRintMilliOhms,
      double nominalCsvRintMilliOhms) {
    return batteryConnected ? batteryGaugeRintMilliOhms : nominalCsvRintMilliOhms;
  }

  /** Warms up lazy math, odometry, and AdvantageKit struct buffers during {@code robotInit()}. */
  public void warmupBeforeCompetitionLoop() {
    drive.warmup();
    shotCalculator.calculate(
        drive.getPose(), new ChassisSpeeds(), FieldPositions.forAlliance(HUB_TARGET_METERS));
    intake.periodic();
    tilter.periodic();
    indexer.periodic();
    feeder.periodic();
    shooter.periodic();
    hood.periodic();
    superstructure.periodic();
    Logger.recordOutput(
        "Superstructure/MechanismPoses3d", new Pose3d[] {Pose3d.kZero, Pose3d.kZero});
  }

  private void configureButtonBindings() {
    BooleanSupplier pitTestAllowed = RobotContainer::isPitTestAllowed;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            joystickSensitivity,
            rotationSensitivity,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis() > 0.5));

    if (!ghostSuperstructureActive) {
      driverController
          .rightTrigger()
          .whileTrue(
              DriveCommands.joystickDriveAtAngle(
                  drive,
                  joystickSensitivity,
                  () -> -driverController.getLeftY(),
                  () -> -driverController.getLeftX(),
                  () -> {
                    latestTeleopShotSolution = updateShootOnTheMove(true);
                    return latestTeleopShotSolution.driveAimHeading();
                  },
                  () -> latestTeleopShotSolution.headingFeedforwardRadPerSec(),
                  () -> driverController.getLeftTriggerAxis() > 0.5,
                  superstructure))
          .onFalse(
              Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure)
                  .ignoringDisable(true));
    }

    if (cameraCount > 0) {
      driverController
          .a()
          .whileTrue(
              Commands.defer(
                  () -> {
                    Pose2d targetPose = getHubAlignTargetPose(FieldPositions.isRedAlliance());
                    return new DriveToPose(
                        drive,
                        () -> targetPose,
                        () -> driverController.getLeftTriggerAxis() > 0.5);
                  },
                  Set.of(drive)));
    }
    driverController.b().whileTrue(Commands.run(drive::stopWithX, drive));
    driverController.y().onTrue(Commands.runOnce(this::resetFieldHeading).ignoringDisable(true));
    driverController.start().onTrue(createPitSystemCheckCommand().onlyIf(pitTestAllowed));
    driverController
        .back()
        .whileTrue(
            createBlitz15MinCommand(Blitz15MinCmd.DEFAULT_CHECKPOINT_PATH).onlyIf(pitTestAllowed));

    operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superstructure.requestState(State.INTAKING), superstructure))
        .onFalse(
            Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure)
                .ignoringDisable(true));
    operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.requestState(State.OUTTAKING), superstructure))
        .onFalse(
            Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure)
                .ignoringDisable(true));
    operatorController
        .rightTrigger()
        .whileTrue(createManualShortShotCommand())
        .onFalse(
            Commands.runOnce(() -> superstructure.requestState(State.IDLE), superstructure)
                .ignoringDisable(true));
    operatorController.y().onTrue(createShotCalibrationCommand().onlyIf(pitTestAllowed));
    operatorController.start().onTrue(new HardStopHomingCmd(hood).onlyIf(pitTestAllowed));
    operatorController
        .back()
        .onTrue(Commands.runOnce(this::resetMechanismEncoders, tilter, hood).ignoringDisable(true));
    operatorController.povUp().whileTrue(createManualJogCommand(hood, MANUAL_JOG_VOLTS));
    operatorController.povDown().whileTrue(createManualJogCommand(hood, -MANUAL_JOG_VOLTS));
    operatorController.povLeft().whileTrue(createManualJogCommand(tilter, -MANUAL_JOG_VOLTS));
    operatorController.povRight().whileTrue(createManualJogCommand(tilter, MANUAL_JOG_VOLTS));
  }

  public PitSystemCheckCmd createPitSystemCheckCommand() {
    if (ghostSuperstructureActive) {
      return new PitSystemCheckCmd(
          drive, List.of(), List.of(), this::resetSuperstructureToIdle);
    }
    return new PitSystemCheckCmd(
        drive,
        List.of(
            new PositionCheckTarget(tilter, -2.0, 0.0), new PositionCheckTarget(hood, 2.0, 0.0)),
        List.of(
            new VelocityCheckTarget(intake, 20.0),
            new VelocityCheckTarget(indexer, 20.0),
            new VelocityCheckTarget(feeder, -20.0),
            new VelocityCheckTarget(shooter, 25.0)),
        this::resetSuperstructureToIdle,
        superstructure);
  }

  public ShotCalibrationCmd createShotCalibrationCommand() {
    return new ShotCalibrationCmd(
        drive,
        superstructure,
        shotCalculator,
        () -> FieldPositions.forAlliance(HUB_TARGET_METERS));
  }

  public Blitz15MinCmd createBlitz15MinCommand(Path checkpointPath) {
    BooleanSupplier deadManHeld = () -> forceSimIo || driverController.getLeftTriggerAxis() > 0.5;
    return new Blitz15MinCmd(
        createPitSystemCheckCommand(),
        new HardStopHomingCmd(hood),
        new ZeroPriorBootstrapCmd(drive, vision, deadManHeld),
        new ShopStressSuiteCmd(drive, deadManHeld),
        checkpointPath);
  }

  public OperatingMode getOperatingMode() {
    if (DriverStation.isFMSAttached() || fluxFig.getMode() != FluxFig.Mode.TUNING) {
      return OperatingMode.COMPETITION;
    }
    return OperatingMode.TRAINING;
  }

  private void applyFieldAndArenaConfig() {
    double shopLen = fluxFig.hasParameter("shopLengthMeters") ? fluxFig.getDouble("shopLengthMeters") : 5.0;
    double shopWid = fluxFig.hasParameter("shopWidthMeters") ? fluxFig.getDouble("shopWidthMeters") : 3.0;
    FieldPositions.setShopDimensions(shopLen, shopWid);
    if (fluxFig.hasParameter("fieldProfile")) {
      try {
        FieldPositions.setFieldProfile(
            FieldPositions.FieldProfile.valueOf(fluxFig.getString("fieldProfile").trim()));
      } catch (IllegalArgumentException ignored) {
        FieldPositions.setFieldProfile(FieldPositions.FieldProfile.FULL_FIELD);
      }
    }
  }

  /**
   * Executes passive MegaTag1 gyro seeding and starting pose audit while Disabled. Never mutates
   * the reported boot {@code ConfigSHA256} or motor gains in place; if config files change on disk,
   * sets {@code FluxFig/RestartRequired = true} and warns that a robot code restart is required.
   */
  public void disabledPeriodic() {
    boolean fms = DriverStation.isFMSAttached();
    fluxFig.lockProductionOnFms(fms);
    if (!fms) {
      if (disabledConfigPollCounter == 0) {
        checkConfigDiskChangeNow();
      }
      disabledConfigPollCounter = (disabledConfigPollCounter + 1) % CONFIG_DISK_POLL_INTERVAL_LOOPS;
    }
    Logger.recordOutput("FluxFig/BootConfigSHA256", bootConfigSha256);
    Logger.recordOutput("FluxFig/EffectiveVersion", bootEffectiveVersion);
    Logger.recordOutput("FluxFig/EffectiveSnapshot", bootEffectiveSnapshot);
    Logger.recordOutput("FluxFig/RestartRequired", configRestartRequired);
    preMatchCalibrator.updateDisabled();
  }

  /** Checks disk configs against boot fingerprint without mutating active boot motor gains/SHA. */
  public boolean checkConfigDiskChangeNow() {
    long currentFingerprint = computeConfigDiskFingerprint(configDir);
    if (currentFingerprint != bootDiskFingerprint) {
      if (!configRestartRequired) {
        configRestartRequired = true;
        DriverStation.reportWarning(
            "FluxFig configs changed on disk; restart robot code (frcKillRobot.sh -t -r) to apply"
                + " updated motor gains/limits (still running boot ConfigSHA256="
                + bootConfigSha256
                + ").",
            false);
      }
      return true;
    }
    return false;
  }

  public boolean isConfigRestartRequired() {
    return configRestartRequired;
  }

  public String getBootConfigSha256() {
    return bootConfigSha256;
  }

  private static long computeConfigDiskFingerprint(Path dir) {
    if (dir == null || !Files.isDirectory(dir)) {
      return 0L;
    }
    long hash = 1125899906842597L;
    try (Stream<Path> s = Files.list(dir)) {
      for (Path p :
          s.filter(Files::isRegularFile)
              .filter(
                  f -> {
                    String n = f.getFileName().toString().toLowerCase(Locale.ROOT);
                    return n.endsWith(".csv") || n.equals(FluxFig.ACTIVE_PROFILE_FILE);
                  })
              .sorted(Comparator.comparing(f -> f.getFileName().toString()))
              .toList()) {
        hash = 31L * hash + p.getFileName().toString().hashCode();
        hash = 31L * hash + Files.getLastModifiedTime(p).toMillis();
        hash = 31L * hash + Files.size(p);
      }
    } catch (IOException ignored) {
    }
    return hash;
  }

  /**
   * Executes container-level coordination every loop: updates {@link BatteryIO}, {@link
   * VirtualBatterySim}, and {@link FaultSupervisor} with real or Ghost mechanism currents.
   */
  public void periodicUpdate() {
    boolean fms = DriverStation.isFMSAttached();
    fluxFig.lockProductionOnFms(fms);
    OperatingMode mode = getOperatingMode();
    boolean training = mode == OperatingMode.TRAINING;
    LoggedTunableNumber.setTuningMode(training);
    if (training) {
      if (!tuningBackendEnabled) {
        enableLiveTuningBackend();
      }
      if (fluxFig.pollTuning() > 0) {
        applyLiveTunedDrivetrainParameters();
      }
    } else if (wasTrainingMode) {
      applyLiveTunedDrivetrainParameters();
    }
    wasTrainingMode = training;
    Logger.recordOutput("Robot/OperatingMode", mode.name());
    Logger.recordOutput("FluxFig/ConfigChannel", bootConfigChannel);
    Logger.recordOutput("FluxFig/ConfigPreset", bootConfigPreset);
    Logger.recordOutput("FluxFig/Modified", fluxFig.isModified());
    Logger.recordOutput("FluxFig/EffectiveVersion", fluxFig.getEffectiveVersion());

    joystickSensitivity.pollTunables();
    rotationSensitivity.pollTunables();
    batteryMonitor.periodic();

    vision.setRobotOrientation(
        drive.getPose().getRotation(),
        drive.getChassisSpeeds().omegaRadiansPerSecond,
        preMatchCalibrator.isGyroSeeded(),
        drive.getMaxTiltRad());

    double mechSupplyAmps =
        (forceSimIo || ghostSuperstructureActive)
            ? superstructure.getTotalMechanismSupplyCurrentAmps()
            : 0.0;
    double simVBat =
        virtualBatterySim.update(drive.getTotalSupplyCurrentAmps(), mechSupplyAmps);
    double effectiveVBat =
        (robotMode == Robot.Mode.REPLAY && batteryMonitor.isConnected())
            ? batteryMonitor.getVoltageVolts()
            : selectEffectiveBatteryVoltage(
                forceSimIo, RobotBase.isReal(), RobotController.getBatteryVoltage(), simVBat);
    double effectiveRintMohm =
        selectEffectiveRintMilliOhms(
            batteryMonitor.isConnected(),
            batteryMonitor.getInternalResistanceMilliOhms(),
            batteryMonitor.getNominalRintMilliOhms());
    faultSupervisor.updateBatteryProtection(
        effectiveVBat, effectiveRintMohm, drive.getLoopPeriodSeconds());
    drive.applyBatteryProtection(faultSupervisor);

    Logger.recordOutput("Superstructure/GhostModeActive", ghostSuperstructureActive);
  }

  /** Computes a Shoot-on-the-Move solution toward the Hub and applies it to {@link Superstructure}. */
  public ShotSolution updateShootOnTheMove(boolean fireWhenReady) {
    Pose2d pose = drive.getPose();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), pose.getRotation());
    ShotSolution solution =
        shotCalculator.calculate(
            pose, fieldSpeeds, FieldPositions.forAlliance(HUB_TARGET_METERS));
    double headingErrorRad =
        Math.abs(
            MathUtil.angleModulus(
                solution.driveAimHeading().minus(pose.getRotation()).getRadians()));
    boolean inRange = shotCalculator.isInRange(solution.effectiveDistanceMeters());
    superstructure.applyShotSolution(solution, fireWhenReady, headingErrorRad, inRange);
    return solution;
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.get();
    return (selected != null) ? selected : Commands.none().withName(SAFE_DEFAULT_AUTO_NAME);
  }

  /**
   * Resolves a named 2026 autonomous routine by exact or case-insensitive substring match for
   * {@code ./gradlew renderAuto -Pauto="<name>"}. Empty selector renders the student starter auto.
   */
  public Command resolveAutonomousCommand(String selector) {
    if (selector == null || selector.isBlank()) {
      return StudentAutoRoutines.createStudentDefaultAuto(drive, superstructure, shotCalculator);
    }
    String needle = selector.trim().toLowerCase(java.util.Locale.US);
    var namedAutos = StudentAutoRoutines.createNamedAutos(drive, superstructure, shotCalculator);
    for (var entry : namedAutos.entrySet()) {
      if (entry.getKey().equalsIgnoreCase(selector.trim())) {
        return entry.getValue().withName(entry.getKey());
      }
    }
    for (var entry : namedAutos.entrySet()) {
      if (entry.getKey().toLowerCase(java.util.Locale.US).contains(needle)) {
        return entry.getValue().withName(entry.getKey());
      }
    }
    throw new IllegalArgumentException(
        "Unknown autonomous routine '"
            + selector
            + "'. Available routines: "
            + String.join(" | ", namedAutos.keySet()));
  }

  public Command createExampleAutonomousCommand() {
    return StudentAutoRoutines.createExampleSquareCircleAndShootAuto(
        drive, superstructure, shotCalculator);
  }

  public LoggedDashboardChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public RobotIdentity getIdentity() {
    return identity;
  }

  public FluxFig getFluxFig() {
    return fluxFig;
  }

  public FaultSupervisor getFaultSupervisor() {
    return faultSupervisor;
  }

  public BatteryIO getBatteryIo() {
    return batteryIo;
  }

  public BatteryMonitor getBatteryMonitor() {
    return batteryMonitor;
  }

  public Robot.Mode getRobotMode() {
    return robotMode;
  }

  public VirtualBatterySim getVirtualBatterySim() {
    return virtualBatterySim;
  }

  public ShotCalculator getShotCalculator() {
    return shotCalculator;
  }

  public PiecewiseSensitivity getJoystickSensitivity() {
    return joystickSensitivity;
  }

  public Drive getDrive() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }

  public PreMatchCalibrator getPreMatchCalibrator() {
    return preMatchCalibrator;
  }

  public VelocityMechanism getIntake() {
    return intake;
  }

  public PositionMechanism getTilter() {
    return tilter;
  }

  public VelocityMechanism getIndexer() {
    return indexer;
  }

  public VelocityMechanism getFeeder() {
    return feeder;
  }

  public VelocityMechanism getShooter() {
    return shooter;
  }

  public PositionMechanism getHood() {
    return hood;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public boolean isGhostSuperstructureActive() {
    return ghostSuperstructureActive;
  }

  public int getCameraCount() {
    return cameraCount;
  }

  public PiecewiseSensitivity getRotationSensitivity() {
    return rotationSensitivity;
  }

  public String getBootConfigChannel() {
    return bootConfigChannel;
  }

  public String getBootConfigPreset() {
    return bootConfigPreset;
  }

  public boolean isLiveTuningBackendEnabled() {
    return tuningBackendEnabled;
  }

  /** Closes owned NT4 tuning publishers when the robot JVM or simulation harness shuts down. */
  public void close() {
    if (ownedNt4TuningBackend != null) {
      ownedNt4TuningBackend.close();
      ownedNt4TuningBackend = null;
    }
  }

  private void registerLiveDrivetrainTunables() {
    fluxFig.registerTunableParameters(
        "maxLinearSpeedMps",
        "maxAngularSpeedRadPerSec",
        "joystickSensitivity",
        "rotationSensitivity");
    fluxFig.registerTunableTableCell("mechanisms", "SwerveDriveFL", "kP");
  }

  private void enableLiveTuningBackend() {
    if (DriverStation.isFMSAttached()) {
      return;
    }
    FluxFig.TuningBackend backend = externalTuningBackend;
    if (backend == null) {
      if (ownedNt4TuningBackend == null) {
        ownedNt4TuningBackend = new Nt4FluxFigTuningBackend();
      }
      backend = ownedNt4TuningBackend;
    }
    fluxFig.enableTuning(backend);
    tuningBackendEnabled = true;
  }

  private void applyLiveTunedDrivetrainParameters() {
    double linearMps =
        MathUtil.clamp(safeReadDouble("maxLinearSpeedMps", drive.getMaxLinearSpeedMps()), 0.25, 6.0);
    double angularRadPerSec =
        MathUtil.clamp(
            safeReadDouble("maxAngularSpeedRadPerSec", drive.getMaxAngularSpeedRadPerSec()),
            0.5,
            12.0);
    drive.setRuntimeSpeedLimits(linearMps, angularRadPerSec);
    if (fluxFig.hasParameter("joystickSensitivity")) {
      joystickSensitivity.setMaxScale(
          MathUtil.clamp(
              safeReadDouble("joystickSensitivity", joystickSensitivity.getMaxScale()), 0.05, 1.0));
    }
    if (fluxFig.hasParameter("rotationSensitivity")) {
      rotationSensitivity.setMaxScale(
          MathUtil.clamp(
              safeReadDouble("rotationSensitivity", rotationSensitivity.getMaxScale()), 0.05, 1.0));
    }
    double driveKp = resolveLiveDriveKp();
    if (Double.isFinite(driveKp) && driveKp >= 0.0) {
      drive.setDriveKp(MathUtil.clamp(driveKp, 0.0, 20.0));
    }
    Logger.recordOutput("Drive/RuntimeMaxLinearSpeedMps", drive.getMaxLinearSpeedMps());
    Logger.recordOutput("Drive/RuntimeMaxAngularSpeedRadPerSec", drive.getMaxAngularSpeedRadPerSec());
    Logger.recordOutput("Drive/RuntimeDriveKp", drive.getRuntimeDriveKp());
  }

  private double safeReadDouble(String key, double fallback) {
    try {
      double value = fluxFig.getDouble(key);
      return Double.isFinite(value) ? value : fallback;
    } catch (Exception ignored) {
      return fallback;
    }
  }

  private double resolveLiveDriveKp() {
    try {
      return fluxFig.getMechanismSpec(fluxFig.getActiveProfile(), "SwerveDriveFL").kP();
    } catch (Exception ignored) {
      try {
        return fluxFig.getMechanismSpec(identity.name(), "SwerveDriveFL").kP();
      } catch (Exception fallbackIgnored) {
        return drive.getRuntimeDriveKp();
      }
    }
  }

  public static ActiveProfileDirective readActiveProfileDirective(Path configDir) {
    if (configDir == null) {
      return null;
    }
    Path file = configDir.resolve(FluxFig.ACTIVE_PROFILE_FILE);
    if (!Files.isRegularFile(file)) {
      return null;
    }
    try {
      return FluxFig.parseActiveProfileDirective(Files.readString(file, StandardCharsets.UTF_8));
    } catch (IOException ignored) {
      return null;
    }
  }

  public static String resolveBootConfigChannel(ActiveProfileDirective directive, FluxFig config) {
    if (directive != null && directive.channel() != null && !directive.channel().isBlank()) {
      return directive.channel().trim();
    }
    return (config != null && config.getMode() == FluxFig.Mode.TUNING) ? "EXPERIMENTAL" : "STABLE";
  }

  public static String resolveBootConfigPreset(ActiveProfileDirective directive, FluxFig config) {
    if (directive != null && directive.preset() != null && !directive.preset().isBlank()) {
      return directive.preset().trim();
    }
    return (config != null && config.getMode() == FluxFig.Mode.TUNING) ? "live-tuning" : "STABLE";
  }
}
