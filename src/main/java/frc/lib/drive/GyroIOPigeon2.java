package frc.lib.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;
import java.util.List;
import java.util.Queue;

/**
 * Real CTRE {@link Pigeon2} implementation of {@link GyroIO} with 250 Hz {@link
 * PhoenixOdometryThread} registration and SystemCore onboard 6-axis IMU fallback hook.
 */
public class GyroIOPigeon2 implements GyroIO, AutoCloseable {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Voltage> supplyVoltage;
  private final int signalGroup;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  private boolean hasOnboardImuFallback = false;
  private double fallbackYawRad = 0.0;
  private double fallbackYawRateRadPerSec = 0.0;

  public GyroIOPigeon2(int pigeonId, String canBusName) {
    pigeon = new Pigeon2(pigeonId, new CANBus(canBusName));
    // Preserve Tuner X flash MountPose and GyroTrim calibration (do not apply blank Pigeon2Configuration).
    pigeon.setYaw(0.0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    roll = pigeon.getRoll();
    pitch = pigeon.getPitch();
    supplyVoltage = pigeon.getSupplyVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, roll, pitch, supplyVoltage);

    // Yaw and yawVelocity are owned exclusively by 250 Hz PhoenixOdometryThread under ODOMETRY_LOCK.
    signalGroup = PhoenixSignals.register(canBusName, roll, pitch, supplyVoltage);
    PhoenixOdometryThread odomThread = PhoenixOdometryThread.getInstance();
    yawTimestampQueue = odomThread.makeTimestampQueue();
    yawPositionQueue = odomThread.registerLatencyCompensatedSignal(yaw, yawVelocity);
    PhoenixUtil.tryUntilOk(
        "Gyro-Pigeon2-" + pigeonId + "-OptimizeBus",
        () -> ParentDevice.optimizeBusUtilizationForAll(pigeon),
        3);
  }

  public Pigeon2 getPigeon() {
    return pigeon;
  }

  /** Feeds the SystemCore onboard 6-axis IMU reading used if the CAN Pigeon 2 disconnects. */
  public void setOnboardImuFallback(double yawRad, double yawRateRadPerSec) {
    hasOnboardImuFallback = true;
    fallbackYawRad = yawRad;
    fallbackYawRateRadPerSec = yawRateRadPerSec;
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
    fallbackYawRad = yaw.getRadians();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean telemetryOk = PhoenixSignals.isOK(signalGroup);
    boolean canOk = telemetryOk && yaw.getStatus().isOK() && yawVelocity.getStatus().isOK();
    double rawYawDeg = yaw.getValueAsDouble();
    double rawYawVelDps = yawVelocity.getValueAsDouble();
    double rawRollDeg = roll.getValueAsDouble();
    double rawPitchDeg = pitch.getValueAsDouble();
    double rawSupplyV = supplyVoltage.getValueAsDouble();

    double[] drainedTimestamps;
    double[] drainedYawRot;
    PhoenixOdometryThread.ODOMETRY_LOCK.lock();
    try {
      int nTs = yawTimestampQueue.size();
      drainedTimestamps = new double[nTs];
      for (int i = 0; i < nTs; i++) {
        Double ts = yawTimestampQueue.poll();
        drainedTimestamps[i] = (ts != null) ? ts : 0.0;
      }
      int nYaw = yawPositionQueue.size();
      drainedYawRot = new double[nYaw];
      for (int i = 0; i < nYaw; i++) {
        Double rot = yawPositionQueue.poll();
        drainedYawRot[i] = (rot != null) ? rot : (rawYawDeg / 360.0);
      }
    } finally {
      PhoenixOdometryThread.ODOMETRY_LOCK.unlock();
    }

    if (canOk) {
      inputs.connected = true;
      inputs.usingOnboardFallback = false;
      inputs.yawPosition = Rotation2d.fromDegrees(rawYawDeg);
      inputs.yawVelocityRadPerSec = Units.degreesToRadians(rawYawVelDps);
      inputs.rollRad = Units.degreesToRadians(rawRollDeg);
      inputs.pitchRad = Units.degreesToRadians(rawPitchDeg);
      inputs.supplyVoltageV = rawSupplyV;
      int n = Math.min(drainedTimestamps.length, drainedYawRot.length);
      inputs.odometryYawTimestamps = new double[n];
      inputs.odometryYawPositions = new Rotation2d[n];
      for (int i = 0; i < n; i++) {
        inputs.odometryYawTimestamps[i] = drainedTimestamps[i];
        inputs.odometryYawPositions[i] = Rotation2d.fromRotations(drainedYawRot[i]);
      }
    } else if (hasOnboardImuFallback) {
      inputs.connected = true;
      inputs.usingOnboardFallback = true;
      inputs.yawPosition = new Rotation2d(fallbackYawRad);
      inputs.yawVelocityRadPerSec = fallbackYawRateRadPerSec;
      if (drainedTimestamps.length > 0) {
        double latestTs = drainedTimestamps[drainedTimestamps.length - 1];
        inputs.odometryYawTimestamps = drainedTimestamps;
        inputs.odometryYawPositions = new Rotation2d[drainedTimestamps.length];
        for (int i = 0; i < drainedTimestamps.length; i++) {
          double dtBack = latestTs - drainedTimestamps[i];
          inputs.odometryYawPositions[i] =
              new Rotation2d(fallbackYawRad - fallbackYawRateRadPerSec * dtBack);
        }
      } else {
        inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
      }
    } else {
      inputs.connected = false;
      inputs.usingOnboardFallback = false;
      inputs.yawPosition = Rotation2d.fromDegrees(rawYawDeg);
      inputs.yawVelocityRadPerSec = Units.degreesToRadians(rawYawVelDps);
      inputs.rollRad = Units.degreesToRadians(rawRollDeg);
      inputs.pitchRad = Units.degreesToRadians(rawPitchDeg);
      inputs.supplyVoltageV = rawSupplyV;
      inputs.odometryYawTimestamps = new double[0];
      inputs.odometryYawPositions = new Rotation2d[0];
    }
  }

  @Override
  public void close() {
    PhoenixOdometryThread.getInstance()
        .unregisterConsumer(
            List.of(yaw, yawVelocity), List.of(yawPositionQueue), yawTimestampQueue);
    pigeon.close();
  }
}
