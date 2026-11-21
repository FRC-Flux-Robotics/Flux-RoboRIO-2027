package frc.lib.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.drive.Drive;
import frc.lib.vision.Vision;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Passive pre-match calibrator executed during {@code disabledPeriodic()}.
 *
 * <p>Accumulates a 50-sample circular median of unconstrained 3D PnP (`MegaTag1`) vision poses
 * while the robot is stationary on the field, seeds the Pigeon 2 gyro yaw and starting {@code (X,
 * Y)} pose, audits placement against the expected autonomous start pose ({@code < 15 cm}, {@code <
 * 3.0 deg}), and publishes {@code PreMatch/ReadyToPlay}.
 */
public class PreMatchCalibrator {
  public static final int REQUIRED_SAMPLES = 50;
  public static final double MAX_START_POSE_ERROR_METERS = 0.15;
  public static final double MAX_START_HEADING_ERROR_DEG = 3.0;
  public static final double MAX_STATIONARY_OMEGA_RAD_PER_SEC = Math.toRadians(0.5);
  public static final double MAX_STATIONARY_LINEAR_SPEED_MPS = 0.05;
  public static final double MIN_CIRCULAR_RESULTANT_LENGTH = 0.85;
  private static final double RESEED_TRANSLATION_EPSILON_METERS = 0.005;
  private static final double RESEED_HEADING_EPSILON_DEG = 0.15;

  private final Drive drive;
  private final Vision vision;
  private final Deque<Pose2d> window = new ArrayDeque<>(REQUIRED_SAMPLES);

  private Pose2d expectedStartPose = Pose2d.kZero;
  private boolean gyroSeeded = false;
  private boolean readyToPlay = false;
  private boolean manualHeadingOverrideActive = false;
  private double poseErrorMeters = 99.0;
  private double headingErrorDeg = 180.0;
  private int lastConsumedYawTrustedCount = 0;
  private double lastHeadingConsistency = 0.0;
  private Pose2d lastSeededPose = null;

  public PreMatchCalibrator(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
  }

  public void setExpectedStartPose(Pose2d expectedStartPose) {
    this.expectedStartPose = expectedStartPose;
  }

  /** Prevents passive vision reseeding from overwriting an explicit driver heading reset. */
  public void onManualHeadingReset() {
    manualHeadingOverrideActive = true;
    window.clear();
  }

  public void clearManualHeadingReset() {
    manualHeadingOverrideActive = false;
  }

  public boolean isManualHeadingOverrideActive() {
    return manualHeadingOverrideActive;
  }

  /** Runs every loop during {@code disabledPeriodic()} to seed the gyro and audit start pose. */
  public void updateDisabled() {
    if (manualHeadingOverrideActive || !vision.isVisionEnabled()) {
      publishTelemetry();
      return;
    }
    var speeds = drive.getChassisSpeeds();
    boolean stationary =
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                <= MAX_STATIONARY_LINEAR_SPEED_MPS
            && Math.abs(speeds.omegaRadiansPerSecond) <= MAX_STATIONARY_OMEGA_RAD_PER_SEC;
    Optional<Pose2d> latestVision = vision.getLatestYawTrustedPose();
    int yawTrustedCount = vision.getYawTrustedObservationCount();
    if (stationary && latestVision.isPresent() && yawTrustedCount > lastConsumedYawTrustedCount) {
      lastConsumedYawTrustedCount = yawTrustedCount;
      addObservation(latestVision.get());
    }
    publishTelemetry();
  }

  /** Records a stationary yaw-trusted observation and updates the circular median seed once full. */
  public void addObservation(Pose2d observation) {
    if (manualHeadingOverrideActive) {
      return;
    }
    if (window.size() >= REQUIRED_SAMPLES) {
      window.removeFirst();
    }
    window.addLast(observation);

    if (window.size() >= REQUIRED_SAMPLES) {
      Optional<Pose2d> seededPoseOpt = computeCircularMedianPose();
      if (seededPoseOpt.isEmpty()) {
        return;
      }
      Pose2d seededPose = seededPoseOpt.get();
      if (lastSeededPose == null
          || seededPose.getTranslation().getDistance(lastSeededPose.getTranslation())
              > RESEED_TRANSLATION_EPSILON_METERS
          || Math.abs(seededPose.getRotation().minus(lastSeededPose.getRotation()).getDegrees())
              > RESEED_HEADING_EPSILON_DEG) {
        drive.setPose(seededPose);
        lastSeededPose = seededPose;
      }
      gyroSeeded = true;
      poseErrorMeters =
          seededPose.getTranslation().getDistance(expectedStartPose.getTranslation());
      headingErrorDeg =
          Math.abs(seededPose.getRotation().minus(expectedStartPose.getRotation()).getDegrees());
      readyToPlay =
          poseErrorMeters <= MAX_START_POSE_ERROR_METERS
              && headingErrorDeg <= MAX_START_HEADING_ERROR_DEG;
    }
  }

  private Optional<Pose2d> computeCircularMedianPose() {
    List<Double> xs = new ArrayList<>(window.size());
    List<Double> ys = new ArrayList<>(window.size());
    double sumSin = 0.0;
    double sumCos = 0.0;
    for (Pose2d p : window) {
      xs.add(p.getX());
      ys.add(p.getY());
      sumSin += p.getRotation().getSin();
      sumCos += p.getRotation().getCos();
    }
    // Guard against symmetric 180-deg bimodal yaw flips cancelling (sumCos, sumSin) to (0, 0).
    double resultantLength = Math.hypot(sumCos, sumSin) / window.size();
    lastHeadingConsistency = resultantLength;
    if (!Double.isFinite(resultantLength) || resultantLength < MIN_CIRCULAR_RESULTANT_LENGTH) {
      return Optional.empty();
    }
    Collections.sort(xs);
    Collections.sort(ys);
    int mid = xs.size() / 2;
    return Optional.of(
        new Pose2d(new Translation2d(xs.get(mid), ys.get(mid)), new Rotation2d(sumCos, sumSin)));
  }

  private void publishTelemetry() {
    Logger.recordOutput("PreMatch/SampleCount", window.size());
    Logger.recordOutput("PreMatch/GyroSeeded", gyroSeeded);
    Logger.recordOutput("PreMatch/PoseErrorMeters", poseErrorMeters);
    Logger.recordOutput("PreMatch/HeadingErrorDeg", headingErrorDeg);
    Logger.recordOutput("PreMatch/HeadingConsistency", lastHeadingConsistency);
    Logger.recordOutput("PreMatch/ReadyToPlay", readyToPlay);
    Logger.recordOutput("PreMatch/ManualHeadingOverride", manualHeadingOverrideActive);
  }

  public boolean isGyroSeeded() {
    return gyroSeeded;
  }

  public boolean isReadyToPlay() {
    return readyToPlay;
  }

  public double getPoseErrorMeters() {
    return poseErrorMeters;
  }

  public double getHeadingErrorDeg() {
    return headingErrorDeg;
  }
}
