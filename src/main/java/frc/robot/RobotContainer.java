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
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIOSim;
import frc.lib.drive.ModuleIOTalonFX;
import frc.lib.drive.SwerveConfig;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIOSim;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.BatteryIO;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.util.LoggedTunableNumber;
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

  private static final int CONFIG_DISK_POLL_INTERVAL_LOOPS = 25;

  private final RobotIdentity identity;
  private final Path configDir;
  private final FluxFig fluxFig;
  private final String bootConfigSha256;
  private final String bootEffectiveVersion;
  private final String bootEffectiveSnapshot;
  private final long bootDiskFingerprint;
  private final FaultSupervisor faultSupervisor;
  private final BatteryIO batteryIo;
  private final VirtualBatterySim virtualBatterySim;
  private final ShotCalculator shotCalculator;
  private final PiecewiseSensitivity joystickSensitivity;
  private final PiecewiseSensitivity rotationSensitivity;

  private final Drive drive;
  private final Vision vision;
  private final PreMatchCalibrator preMatchCalibrator;

  private final VelocityMechanism intake;
  private final PositionMechanism tilter;
  private final VelocityMechanism indexer;
  private final VelocityMechanism feeder;
  private final VelocityMechanism shooter;
  private final PositionMechanism hood;
  private final Superstructure superstructure;
  private final boolean ghostSuperstructureActive;
  private final boolean forceSimIo;
  private final LoggedDashboardChooser<Command> autoChooser;

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
        !RobotBase.isReal());
  }

  /** Deterministic constructor allowing explicit config directory and simulated IO selection. */
  public RobotContainer(RobotIdentity identity, Path configDir, boolean forceSimIo) {
    this(identity, configDir, null, forceSimIo);
  }

  /** Constructor accepting a pre-loaded boot {@link FluxFig} snapshot to avoid duplicate disk loads. */
  public RobotContainer(
      RobotIdentity identity, Path configDir, FluxFig preloadedFluxFig, boolean forceSimIo) {
    this.identity = identity;
    this.configDir = configDir;
    this.forceSimIo = forceSimIo;
    this.fluxFig =
        (preloadedFluxFig != null) ? preloadedFluxFig : FluxFig.load(configDir, identity.name());
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

    String cameraProfile =
        (identity == RobotIdentity.CORAL || identity == RobotIdentity.CHASSIS_3)
            ? RobotIdentity.CORAL.name()
            : (identity == RobotIdentity.FUEL
                ? RobotIdentity.FUEL.name()
                : RobotIdentity.COMP_2027.name());

    boolean simMissingMechs = fluxFig.getBoolean("simulateMissingMechanisms");
    this.ghostSuperstructureActive =
        simMissingMechs
            || identity == RobotIdentity.CORAL
            || identity == RobotIdentity.CHASSIS_3;

    CameraSpec[] camConfigs =
        fluxFig.getCamerasForRobot(cameraProfile).toArray(new CameraSpec[0]);
    this.faultSupervisor = new FaultSupervisor(Math.max(1, camConfigs.length));
    this.batteryIo = new BatteryIO(fluxFig, fluxFig.getString("auxCanBus"));
    this.virtualBatterySim = new VirtualBatterySim(12.6, 0.015);
    this.shotCalculator = ShotCalculator.fromFluxFig(fluxFig);
    this.joystickSensitivity =
        new PiecewiseSensitivity("Drive/JoystickCurve", 0.09, 0.60, 0.10, 0.30, 0.80);
    this.rotationSensitivity =
        new PiecewiseSensitivity("Drive/RotationCurve", 0.09, 0.60, 0.10, 0.50, 1.00);

    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fluxFig, identity.name());
    GyroIO gyroIo;
    ModuleIO[] moduleIos = new ModuleIO[4];
    if (forceSimIo) {
      gyroIo = new GyroIO() {};
      for (int i = 0; i < 4; i++) {
        moduleIos[i] = new ModuleIOSim(swerveConfig);
      }
    } else {
      PhoenixUtil.beginCanInitBudget();
      gyroIo = new GyroIOPigeon2(swerveConfig.pigeonId(), swerveConfig.canBus());
      for (int i = 0; i < 4; i++) {
        moduleIos[i] = new ModuleIOTalonFX(swerveConfig, swerveConfig.corners()[i]);
      }
    }
    this.drive = new Drive(swerveConfig, gyroIo, moduleIos);
    this.drive.setLoopPeriodSeconds(fluxFig.getInt("loopPeriodMs") / 1000.0);

    VisionIO[] visionIos = new VisionIO[camConfigs.length];
    String visionBackend = fluxFig.getString("visionBackend");
    if (forceSimIo) {
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
    this.vision = new Vision(drive::addVisionMeasurement, faultSupervisor, camConfigs, visionIos);
    this.preMatchCalibrator = new PreMatchCalibrator(drive, vision);

    boolean useSimMechanisms = forceSimIo || ghostSuperstructureActive;
    if (!useSimMechanisms) {
      // Refresh boot CAN retry budget after potentially slow AprilTagFieldLayout loading.
      PhoenixUtil.beginCanInitBudget();
    }
    String mechProfile =
        (identity == RobotIdentity.FUEL) ? fluxFig.getActiveProfile() : "FUEL";

    MechanismSpec intakeSpec = fluxFig.getMechanismSpec(mechProfile, "Intake");
    this.intake =
        new VelocityMechanism(
            useSimMechanisms ? new MechanismIOSim(intakeSpec) : new MechanismIOTalonFX(intakeSpec),
            intakeSpec);

    MechanismSpec tilterSpec = fluxFig.getMechanismSpec(mechProfile, "Tilter");
    this.tilter =
        new PositionMechanism(
            useSimMechanisms ? new MechanismIOSim(tilterSpec) : new MechanismIOTalonFX(tilterSpec),
            tilterSpec);

    MechanismSpec indexerSpec = fluxFig.getMechanismSpec(mechProfile, "Indexer");
    this.indexer =
        new VelocityMechanism(
            useSimMechanisms
                ? new MechanismIOSim(indexerSpec)
                : new MechanismIOTalonFX(indexerSpec),
            indexerSpec);

    MechanismSpec feederSpec = fluxFig.getMechanismSpec(mechProfile, "Feeder");
    this.feeder =
        new VelocityMechanism(
            useSimMechanisms ? new MechanismIOSim(feederSpec) : new MechanismIOTalonFX(feederSpec),
            feederSpec);

    MechanismSpec shooterSpec = fluxFig.getMechanismSpec(mechProfile, "Shooter");
    MechanismIO shooterIo =
        useSimMechanisms
            ? new MechanismIOSim(shooterSpec)
            : new MechanismIOTalonFX(
                shooterSpec,
                shooterSpec.followerMotorId(),
                shooterSpec.invertFollower(),
                NeutralModeValue.Coast);
    this.shooter = new VelocityMechanism(shooterIo, shooterSpec);

    MechanismSpec hoodSpec = fluxFig.getMechanismSpec(mechProfile, "Hood");
    this.hood =
        new PositionMechanism(
            useSimMechanisms ? new MechanismIOSim(hoodSpec) : new MechanismIOTalonFX(hoodSpec),
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

    registerNamedCommands();
    this.autoChooser = new LoggedDashboardChooser<>("Auto/Chooser");
    configureAutoChooser();
    configureButtonBindings();

    Logger.recordOutput("FluxFig/BootConfigSHA256", bootConfigSha256);
    Logger.recordOutput("FluxFig/EffectiveVersion", bootEffectiveVersion);
    Logger.recordOutput("FluxFig/EffectiveSnapshot", bootEffectiveSnapshot);
    Logger.recordOutput("FluxFig/RestartRequired", false);
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
    var namedAutos = StudentAutoRoutines.createNamedAutos(drive, superstructure, shotCalculator);
    boolean firstAdded = false;
    for (var entry : namedAutos.entrySet()) {
      if (!firstAdded) {
        autoChooser.addDefaultOption(entry.getKey(), entry.getValue());
        firstAdded = true;
      } else {
        autoChooser.addOption(entry.getKey(), entry.getValue());
      }
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
        "15-Min Floor Blitz",
        createBlitz15MinCommand(Blitz15MinCmd.DEFAULT_CHECKPOINT_PATH)
            .onlyIf(RobotContainer::isPitTestAllowed));
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
  }

  /** Resets the 6-mechanism superstructure to {@link State#IDLE} across mode transitions. */
  public void resetSuperstructureToIdle() {
    superstructure.requestState(State.IDLE);
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

  /** Warms up lazy math, odometry, and AdvantageKit struct buffers during {@code robotInit()}. */
  public void warmupBeforeCompetitionLoop() {
    drive.warmup();
    shotCalculator.calculate(
        drive.getPose(), new ChassisSpeeds(), FieldPositions.forAlliance(HUB_TARGET_METERS));
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
    operatorController.y().onTrue(createShotCalibrationCommand().onlyIf(pitTestAllowed));
    operatorController.start().onTrue(new HardStopHomingCmd(hood).onlyIf(pitTestAllowed));
  }

  public PitSystemCheckCmd createPitSystemCheckCommand() {
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
      fluxFig.pollTuning();
    }
    Logger.recordOutput("Robot/OperatingMode", mode.name());

    joystickSensitivity.pollTunables();
    rotationSensitivity.pollTunables();
    batteryIo.periodic();

    vision.setRobotOrientation(
        drive.getPose().getRotation(),
        drive.getChassisSpeeds().omegaRadiansPerSecond,
        preMatchCalibrator.isGyroSeeded(),
        drive.getMaxTiltRad());

    double ghostSupplyAmps =
        ghostSuperstructureActive ? superstructure.getTotalMechanismSupplyCurrentAmps() : 0.0;
    double simVBat =
        virtualBatterySim.update(drive.getTotalSupplyCurrentAmps(), ghostSupplyAmps);
    double effectiveVBat =
        selectEffectiveBatteryVoltage(
            forceSimIo, RobotBase.isReal(), RobotController.getBatteryVoltage(), simVBat);
    double effectiveRintMohm =
        batteryIo.isHealthy()
            ? batteryIo.getInternalResistanceMilliOhms()
            : virtualBatterySim.estimatedRintMilliOhms();
    faultSupervisor.updateBatteryProtection(effectiveVBat, effectiveRintMohm);
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
    return (selected != null)
        ? selected
        : StudentAutoRoutines.createStudentDefaultAuto(drive, superstructure, shotCalculator);
  }

  /**
   * Resolves a named 2026 autonomous routine by exact or case-insensitive substring match for
   * {@code ./gradlew renderAuto -Pauto="<name>"}.
   */
  public Command resolveAutonomousCommand(String selector) {
    if (selector == null || selector.isBlank()) {
      return getAutonomousCommand();
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
}
