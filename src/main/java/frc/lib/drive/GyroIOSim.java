package frc.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.function.DoubleSupplier;

/**
 * Simulation implementation of {@link GyroIO} integrating multi-substep swerve module kinematics
 * from 4 {@link ModuleIOSim} instances into realistic high-rate gyro yaw, angular velocity, and
 * supply voltage inputs.
 */
public class GyroIOSim implements GyroIO {
  private final SwerveConfig config;
  private final ModuleIOSim[] moduleSims;
  private final SwerveDriveKinematics kinematics;
  private final DoubleSupplier railVoltageSupplier;

  private final double[] prevModuleDistanceMeters = new double[4];
  private final SwerveModulePosition[] scratchDeltas = new SwerveModulePosition[4];
  private boolean initialized = false;
  private double yawRad = 0.0;

  public GyroIOSim(
      SwerveConfig config, ModuleIOSim[] moduleSims, DoubleSupplier railVoltageSupplier) {
    if (moduleSims == null || moduleSims.length != 4) {
      throw new IllegalArgumentException("GyroIOSim requires exactly 4 ModuleIOSim instances");
    }
    this.config = config;
    this.moduleSims = moduleSims;
    this.railVoltageSupplier = (railVoltageSupplier != null) ? railVoltageSupplier : () -> 12.0;
    Translation2d[] translations = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      translations[i] = config.corners()[i].moduleTranslation();
    }
    this.kinematics = new SwerveDriveKinematics(translations);
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    this.yawRad = yaw.getRadians();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double[] timestamps = moduleSims[0].getLastOdometryTimestamps();
    int sampleCount = timestamps.length;
    for (int m = 0; m < 4; m++) {
      sampleCount =
          Math.min(
              sampleCount,
              Math.min(
                  moduleSims[m].getLastOdometryDrivePositionsRad().length,
                  moduleSims[m].getLastOdometryTurnPositions().length));
    }

    double totalDtheta = 0.0;
    Rotation2d[] yawSamples = new Rotation2d[sampleCount];
    double[] yawTimestamps = new double[sampleCount];

    for (int i = 0; i < sampleCount; i++) {
      for (int m = 0; m < 4; m++) {
        double distM =
            moduleSims[m].getLastOdometryDrivePositionsRad()[i] * config.wheelRadiusMeters();
        Rotation2d angle = moduleSims[m].getLastOdometryTurnPositions()[i];
        double deltaM = initialized ? (distM - prevModuleDistanceMeters[m]) : 0.0;
        prevModuleDistanceMeters[m] = distM;
        scratchDeltas[m] = new SwerveModulePosition(deltaM, angle);
      }
      initialized = true;
      Twist2d twist = kinematics.toTwist2d(scratchDeltas);
      yawRad += twist.dtheta;
      totalDtheta += twist.dtheta;
      yawTimestamps[i] = timestamps[i];
      yawSamples[i] = new Rotation2d(yawRad);
    }

    double cycleDt = Math.max(0.002, moduleSims[0].getLoopPeriodSeconds());
    double railV = railVoltageSupplier.getAsDouble();
    inputs.connected = true;
    inputs.usingOnboardFallback = false;
    inputs.yawPosition = new Rotation2d(yawRad);
    inputs.yawVelocityRadPerSec = totalDtheta / cycleDt;
    inputs.rollRad = 0.0;
    inputs.pitchRad = 0.0;
    inputs.supplyVoltageV = Double.isFinite(railV) ? railV : 12.0;
    inputs.odometryYawTimestamps = yawTimestamps;
    inputs.odometryYawPositions = yawSamples;
  }
}
