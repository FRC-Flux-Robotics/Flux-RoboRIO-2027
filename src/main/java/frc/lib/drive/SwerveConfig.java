package frc.lib.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import java.util.List;
import java.util.Map;

/**
 * Immutable swerve drivetrain configuration record (chassis geometry, gear ratios, CAN bus,
 * Pigeon 2 ID, 4 corner module definitions, and current limits) constructible from {@link FluxFig}
 * (`parameters.csv`, `mechanisms.csv`, and `swerve_modules.csv`).
 */
public record SwerveConfig(
    String canBus,
    int pigeonId,
    double trackWidthMeters,
    double wheelBaseMeters,
    double wheelRadiusMeters,
    double driveGearRatio,
    double turnGearRatio,
    double maxLinearSpeedMps,
    double maxAngularSpeedRadPerSec,
    double driveStatorLimitAmps,
    double driveSupplyLimitAmps,
    double driveSlipCurrentAmps,
    double turnStatorLimitAmps,
    double turnSupplyLimitAmps,
    double driveKp,
    double driveKi,
    double driveKd,
    double driveKs,
    double driveKv,
    double driveKa,
    double turnKp,
    double turnKd,
    CornerConfig[] corners,
    double odometryFrequencyHz,
    double couplingRatio) {

  /** Default CTRE coaxial swerve azimuth-to-drive coupling gear ratio (4.5:1 on MK4i/MK4n). */
  public static final double DEFAULT_COUPLING_RATIO = 4.5;

  public SwerveConfig(
      String canBus,
      int pigeonId,
      double trackWidthMeters,
      double wheelBaseMeters,
      double wheelRadiusMeters,
      double driveGearRatio,
      double turnGearRatio,
      double maxLinearSpeedMps,
      double maxAngularSpeedRadPerSec,
      double driveStatorLimitAmps,
      double driveSupplyLimitAmps,
      double driveSlipCurrentAmps,
      double turnStatorLimitAmps,
      double turnSupplyLimitAmps,
      double driveKp,
      double driveKi,
      double driveKd,
      double driveKs,
      double driveKv,
      double driveKa,
      double turnKp,
      double turnKd,
      CornerConfig[] corners,
      double odometryFrequencyHz) {
    this(
        canBus,
        pigeonId,
        trackWidthMeters,
        wheelBaseMeters,
        wheelRadiusMeters,
        driveGearRatio,
        turnGearRatio,
        maxLinearSpeedMps,
        maxAngularSpeedRadPerSec,
        driveStatorLimitAmps,
        driveSupplyLimitAmps,
        driveSlipCurrentAmps,
        turnStatorLimitAmps,
        turnSupplyLimitAmps,
        driveKp,
        driveKi,
        driveKd,
        driveKs,
        driveKv,
        driveKa,
        turnKp,
        turnKd,
        corners,
        odometryFrequencyHz,
        DEFAULT_COUPLING_RATIO);
  }

  public SwerveConfig(
      String canBus,
      int pigeonId,
      double trackWidthMeters,
      double wheelBaseMeters,
      double wheelRadiusMeters,
      double driveGearRatio,
      double turnGearRatio,
      double maxLinearSpeedMps,
      double maxAngularSpeedRadPerSec,
      double driveStatorLimitAmps,
      double driveSupplyLimitAmps,
      double driveSlipCurrentAmps,
      double turnStatorLimitAmps,
      double turnSupplyLimitAmps,
      double driveKp,
      double driveKi,
      double driveKd,
      double driveKs,
      double driveKv,
      double driveKa,
      double turnKp,
      double turnKd,
      CornerConfig[] corners) {
    this(
        canBus,
        pigeonId,
        trackWidthMeters,
        wheelBaseMeters,
        wheelRadiusMeters,
        driveGearRatio,
        turnGearRatio,
        maxLinearSpeedMps,
        maxAngularSpeedRadPerSec,
        driveStatorLimitAmps,
        driveSupplyLimitAmps,
        driveSlipCurrentAmps,
        turnStatorLimitAmps,
        turnSupplyLimitAmps,
        driveKp,
        driveKi,
        driveKd,
        driveKs,
        driveKv,
        driveKa,
        turnKp,
        turnKd,
        corners,
        PhoenixOdometryThread.resolveFrequencyHz(canBus, 0.0),
        DEFAULT_COUPLING_RATIO);
  }

  /**
   * Per-corner swerve module hardware IDs, encoder offset, drive/steer inversion flags, and
   * chassis translation.
   */
  public record CornerConfig(
      String name,
      int driveMotorId,
      int turnMotorId,
      int cancoderId,
      double encoderOffsetRot,
      boolean invertDrive,
      boolean invertTurn,
      Translation2d moduleTranslation) {

    public CornerConfig(
        String name,
        int driveMotorId,
        int turnMotorId,
        int cancoderId,
        double encoderOffsetRot,
        Translation2d moduleTranslation) {
      this(
          name,
          driveMotorId,
          turnMotorId,
          cancoderId,
          encoderOffsetRot,
          name.endsWith("R") || moduleTranslation.getY() < 0.0,
          false,
          moduleTranslation);
    }
  }

  /** Builds a complete 4-corner {@link SwerveConfig} for the given robot profile from {@link FluxFig}. */
  public static SwerveConfig fromFluxFig(FluxFig fig, String profileRobot) {
    String prevProfile = fig.getActiveProfile();
    fig.setActiveProfile(profileRobot);
    try {
      String canBus = fig.getString("swerveCanBus");
      double maxLin = fig.getDouble("maxLinearSpeedMps");
      double maxAng = fig.getDouble("maxAngularSpeedRadPerSec");
      double statorLimit = fig.getDouble("driveStatorLimitAmps");
      double supplyLimit = fig.getDouble("driveSupplyLimitAmps");
      double slipLimit = fig.getDouble("driveSlipCurrentAmps");
      int pigeonId = fig.getInt("pigeonId");
      double trackWidthM = fig.getDouble("trackWidthMeters");
      double wheelBaseM = fig.getDouble("wheelBaseMeters");
      double wheelRadiusM = fig.getDouble("wheelRadiusMeters");
      double driveGearRatio = fig.getDouble("driveGearRatio");
      double turnGearRatio = fig.getDouble("turnGearRatio");
      double couplingRatio =
          fig.hasParameter("swerveCouplingRatio")
              ? fig.getDouble("swerveCouplingRatio")
              : DEFAULT_COUPLING_RATIO;
      double configuredOdomHz =
          fig.hasParameter("odometryFrequencyHz") ? fig.getDouble("odometryFrequencyHz") : 0.0;
      double odomFreqHz = PhoenixOdometryThread.resolveFrequencyHz(canBus, configuredOdomHz);

      MechanismSpec flSpec = fig.getMechanismSpec(profileRobot, "SwerveDriveFL");
      if (statorLimit > 60.0 || slipLimit > 60.0 || flSpec.statorLimitAmps() > 60.0) {
        throw new IllegalArgumentException(
            "Swerve stator/slip current limit must be <= 60.0A to prevent brownouts");
      }

      double halfX = wheelBaseM * 0.5;
      double halfY = trackWidthM * 0.5;
      CornerConfig[] corners = loadCornersFromTable(fig, profileRobot, halfX, halfY);

      return new SwerveConfig(
          canBus,
          pigeonId,
          trackWidthM,
          wheelBaseM,
          wheelRadiusM,
          driveGearRatio,
          turnGearRatio,
          maxLin,
          maxAng,
          Math.min(statorLimit, flSpec.statorLimitAmps()),
          Math.min(supplyLimit, flSpec.supplyLimitAmps()),
          Math.min(slipLimit, flSpec.slipCurrentAmps()),
          40.0,
          30.0,
          flSpec.kP(),
          flSpec.kI(),
          flSpec.kD(),
          flSpec.kS(),
          flSpec.kV(),
          flSpec.kA(),
          15.0,
          0.9,
          corners,
          odomFreqHz,
          couplingRatio);
    } finally {
      fig.setActiveProfile(prevProfile);
    }
  }

  private static CornerConfig[] loadCornersFromTable(
      FluxFig fig, String profileRobot, double halfX, double halfY) {
    String baseRobot =
        profileRobot.contains(":")
            ? profileRobot.substring(0, profileRobot.indexOf(':'))
            : profileRobot;
    List<Map<String, String>> rows = fig.getTable("swerve_modules");
    String[] expectedNames = {"FL", "FR", "BL", "BR"};
    Translation2d[] translations = {
      new Translation2d(halfX, halfY),
      new Translation2d(halfX, -halfY),
      new Translation2d(-halfX, halfY),
      new Translation2d(-halfX, -halfY)
    };
    CornerConfig[] corners = new CornerConfig[4];
    for (int i = 0; i < 4; i++) {
      String cornerName = expectedNames[i];
      Map<String, String> match = findCornerRow(rows, baseRobot, cornerName);
      if (match == null) {
        match = findCornerRow(rows, "COMP_2027", cornerName);
      }
      if (match == null) {
        throw new IllegalArgumentException(
            "Missing swerve_modules.csv row for robot=" + baseRobot + ", corner=" + cornerName);
      }
      corners[i] =
          new CornerConfig(
              cornerName,
              Integer.parseInt(match.get("driveMotorId").trim()),
              Integer.parseInt(match.get("turnMotorId").trim()),
              Integer.parseInt(match.get("cancoderId").trim()),
              Double.parseDouble(match.get("encoderOffsetRot").trim()),
              Boolean.parseBoolean(match.get("invertDrive").trim()),
              Boolean.parseBoolean(match.get("invertTurn").trim()),
              translations[i]);
    }
    return corners;
  }

  private static Map<String, String> findCornerRow(
      List<Map<String, String>> rows, String robot, String cornerName) {
    for (Map<String, String> row : rows) {
      if (robot.equalsIgnoreCase(row.getOrDefault("robot", "").trim())
          && cornerName.equalsIgnoreCase(row.getOrDefault("corner", "").trim())) {
        return row;
      }
    }
    return null;
  }
}
