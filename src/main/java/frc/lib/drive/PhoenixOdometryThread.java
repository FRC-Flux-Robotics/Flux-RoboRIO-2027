package frc.lib.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.PhoenixUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Configurable high-frequency swerve odometry thread (100 Hz on 1 Mbps CAN 2.0 buses such as
 * {@code "rio"} / {@code can_s*}; 250 Hz on CTRE CANivore FD buses) using latency-compensated
 * {@code refreshAll} + {@code getLatencyCompensatedValue}, plus deterministic {@link
 * #sampleOnce(double)} stepping for tests.
 */
public final class PhoenixOdometryThread {
  public static final double ODOMETRY_FREQUENCY_HZ = 250.0;
  public static final double CAN_20_ODOMETRY_FREQUENCY_HZ = 100.0;
  public static final double CAN_FD_ODOMETRY_FREQUENCY_HZ = 250.0;
  public static final double MIN_ODOMETRY_FREQUENCY_HZ = 20.0;
  public static final Lock ODOMETRY_LOCK = new ReentrantLock();

  private static PhoenixOdometryThread instance = null;

  private final List<BaseStatusSignal> allSignals = new ArrayList<>();
  private final List<DoubleSupplier> signalSuppliers = new ArrayList<>();
  private final List<Queue<Double>> signalQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();
  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(32);
  private BaseStatusSignal[] cachedSignals = new BaseStatusSignal[0];
  private volatile double odometryFrequencyHz = ODOMETRY_FREQUENCY_HZ;
  private final Thread thread;

  private PhoenixOdometryThread() {
    thread =
        new Thread(
            () -> {
              while (!Thread.currentThread().isInterrupted()) {
                try {
                  long sleepMs = Math.max(2L, Math.round(1000.0 / odometryFrequencyHz));
                  Thread.sleep(sleepMs);
                  if (hasRegisteredSignals()) {
                    sampleNow();
                  }
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                  break;
                }
              }
            },
            "PhoenixOdometryThread");
    thread.setDaemon(true);
    thread.start();
  }

  /** Returns true when {@code canBus} is a 1 Mbps CAN 2.0 bus ({@code "rio"} or {@code can_s*}). */
  public static boolean isCan20Bus(String canBus) {
    if (canBus == null || canBus.isBlank()) {
      return true;
    }
    String lower = canBus.trim().toLowerCase(Locale.ROOT);
    return "rio".equals(lower) || lower.startsWith("can_s");
  }

  /**
   * Resolves odometry polling frequency: honors an explicit positive {@code configuredHz} override
   * (clamped to {@code [20, 250] Hz}), otherwise defaults to {@code 100 Hz} on 1 Mbps CAN 2.0
   * ({@code "rio"} / {@code can_s*}) to avoid {@code TxTimeout} bus saturation and {@code 250 Hz}
   * on CTRE CANivore FD buses.
   */
  public static double resolveFrequencyHz(String canBus, double configuredHz) {
    if (Double.isFinite(configuredHz) && configuredHz > 0.0) {
      return Math.max(MIN_ODOMETRY_FREQUENCY_HZ, Math.min(CAN_FD_ODOMETRY_FREQUENCY_HZ, configuredHz));
    }
    return isCan20Bus(canBus) ? CAN_20_ODOMETRY_FREQUENCY_HZ : CAN_FD_ODOMETRY_FREQUENCY_HZ;
  }

  public void setOdometryFrequencyHz(double frequencyHz) {
    if (!Double.isFinite(frequencyHz) || frequencyHz <= 0.0) {
      return;
    }
    this.odometryFrequencyHz =
        Math.max(MIN_ODOMETRY_FREQUENCY_HZ, Math.min(CAN_FD_ODOMETRY_FREQUENCY_HZ, frequencyHz));
  }

  public double getOdometryFrequencyHz() {
    return odometryFrequencyHz;
  }

  public static synchronized PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  /**
   * Registers a CTRE position + velocity signal pair and returns a 250 Hz queue of latency-
   * compensated position values (in rotations).
   */
  public Queue<Double> registerLatencyCompensatedSignal(
      StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {
    Queue<Double> queue = new ArrayBlockingQueue<>(32);
    double targetFreqHz = this.odometryFrequencyHz;
    ODOMETRY_LOCK.lock();
    try {
      PhoenixUtil.tryUntilOk(
          "Odometry-PosFreq",
          () ->
              position.setUpdateFrequency(
                  targetFreqHz, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
          5);
      PhoenixUtil.tryUntilOk(
          "Odometry-VelFreq",
          () ->
              velocity.setUpdateFrequency(
                  targetFreqHz, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
          5);
      allSignals.add(position);
      allSignals.add(velocity);
      cachedSignals = allSignals.toArray(new BaseStatusSignal[0]);
      signalSuppliers.add(
          () ->
              BaseStatusSignal.getLatencyCompensatedValue(position, velocity).in(Units.Rotations));
      signalQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /** Registers an arbitrary double supplier (for simulation or fallback IMU) at 250 Hz. */
  public Queue<Double> registerGenericSignal(DoubleSupplier supplier) {
    Queue<Double> queue = new ArrayBlockingQueue<>(32);
    ODOMETRY_LOCK.lock();
    try {
      signalSuppliers.add(supplier);
      signalQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /**
   * Creates a dedicated 250 Hz timestamp queue synchronized with every {@link #sampleOnce(double)}
   * tick so each {@link ModuleIOTalonFX} and {@link GyroIOPigeon2} logs replayable timestamps.
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(32);
    ODOMETRY_LOCK.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  public void sampleOnce(double timestampSeconds) {
    ODOMETRY_LOCK.lock();
    try {
      if (cachedSignals.length > 0) {
        BaseStatusSignal.refreshAll(cachedSignals);
      }
      if (!timestampQueue.offer(timestampSeconds)) {
        timestampQueue.poll();
        timestampQueue.offer(timestampSeconds);
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        Queue<Double> tq = timestampQueues.get(i);
        if (!tq.offer(timestampSeconds)) {
          tq.poll();
          tq.offer(timestampSeconds);
        }
      }
      for (int i = 0; i < signalSuppliers.size(); i++) {
        double val = signalSuppliers.get(i).getAsDouble();
        Queue<Double> q = signalQueues.get(i);
        if (!q.offer(val)) {
          q.poll();
          q.offer(val);
        }
      }
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  public void sampleNow() {
    sampleOnce(RobotController.getFPGATime() / 1.0e6);
  }

  public boolean hasRegisteredSignals() {
    ODOMETRY_LOCK.lock();
    try {
      return !signalSuppliers.isEmpty();
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  /** Returns a snapshot of all CTRE StatusSignals refreshed on the 250 Hz odometry thread. */
  public List<BaseStatusSignal> getRegisteredSignals() {
    ODOMETRY_LOCK.lock();
    try {
      return List.copyOf(allSignals);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  /**
   * Ensures at least one 250 Hz sample is queued for the current control loop cycle when hardware
   * signals are registered.
   */
  public void ensureSampleReady() {
    ODOMETRY_LOCK.lock();
    try {
      if (!signalSuppliers.isEmpty() && timestampQueue.isEmpty()) {
        sampleNow();
      }
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  public double[] drainTimestamps() {
    ODOMETRY_LOCK.lock();
    try {
      double[] out = new double[timestampQueue.size()];
      int i = 0;
      while (!timestampQueue.isEmpty()) {
        out[i++] = timestampQueue.poll();
      }
      return out;
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  public synchronized void reset() {
    ODOMETRY_LOCK.lock();
    try {
      allSignals.clear();
      signalSuppliers.clear();
      signalQueues.clear();
      timestampQueues.clear();
      timestampQueue.clear();
      cachedSignals = new BaseStatusSignal[0];
      odometryFrequencyHz = ODOMETRY_FREQUENCY_HZ;
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }
}
