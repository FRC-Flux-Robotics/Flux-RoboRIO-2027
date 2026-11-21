package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Real PhotonVision hardware implementation of {@link VisionIO} that drains 100% of {@link
 * PhotonCamera#getAllUnreadResults()} every cycle without dropping intermediate frames.
 */
@SuppressWarnings({"deprecation", "removal"})
public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private Transform3d robotToCamera;

  public VisionIOPhotonVision(CameraSpec config, AprilTagFieldLayout fieldLayout) {
    camera = new PhotonCamera(config.cameraName());
    robotToCamera = VisionIO.toTransform3d(config);
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void setTransform(Transform3d robotToCamera) {
    this.robotToCamera = robotToCamera;
    poseEstimator.setRobotToCameraTransform(robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
    inputs.framesDrainedThisCycle = unreadResults.size();

    if (unreadResults.isEmpty()) {
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagObservations = new RawTagObservation[0];
      inputs.tagIds = new int[0];
      return;
    }

    List<PoseObservation> poseObsList = new ArrayList<>(unreadResults.size());
    List<RawTagObservation> tagObsList = new ArrayList<>();
    Set<Integer> seenTagIds = new HashSet<>();

    for (PhotonPipelineResult result : unreadResults) {
      if (!result.hasTargets()) {
        continue;
      }

      for (PhotonTrackedTarget target : result.getTargets()) {
        int id = target.getFiducialId();
        seenTagIds.add(id);
        double dist = target.getBestCameraToTarget().getTranslation().getNorm();
        tagObsList.add(
            new RawTagObservation(
                id,
                Units.degreesToRadians(target.getYaw()),
                Units.degreesToRadians(target.getPitch()),
                dist,
                target.getPoseAmbiguity()));
      }

      Optional<EstimatedRobotPose> estOpt = poseEstimator.update(result);
      if (estOpt.isPresent()) {
        buildPoseObservation(estOpt.get()).ifPresent(poseObsList::add);
      }
    }

    inputs.poseObservations = poseObsList.toArray(new PoseObservation[0]);
    inputs.tagObservations = tagObsList.toArray(new RawTagObservation[0]);
    inputs.tagIds = seenTagIds.stream().mapToInt(Integer::intValue).sorted().toArray();
  }

  /**
   * Builds a {@link PoseObservation} strictly from {@code est.targetsUsed}, rejecting invalid
   * single-tag ambiguity sentinels ({@code < 0.0} or non-finite) so bad solves never bypass the
   * ambiguity gate.
   */
  public static Optional<PoseObservation> buildPoseObservation(EstimatedRobotPose est) {
    if (est == null || est.estimatedPose == null || est.targetsUsed == null || est.targetsUsed.isEmpty()) {
      return Optional.empty();
    }
    int tagCount = est.targetsUsed.size();
    double totalDist = 0.0;
    for (PhotonTrackedTarget target : est.targetsUsed) {
      double dist = target.getBestCameraToTarget().getTranslation().getNorm();
      if (!Double.isFinite(dist) || dist < 0.0) {
        return Optional.empty();
      }
      totalDist += dist;
    }
    double avgDist = totalDist / tagCount;
    double ambiguity;
    if (tagCount > 1) {
      ambiguity = 0.0;
    } else {
      double rawAmbiguity = est.targetsUsed.get(0).getPoseAmbiguity();
      if (!Double.isFinite(rawAmbiguity) || rawAmbiguity < 0.0) {
        return Optional.empty();
      }
      ambiguity = rawAmbiguity;
    }
    return Optional.of(
        new PoseObservation(
            est.timestampSeconds,
            est.estimatedPose,
            ambiguity,
            tagCount,
            avgDist,
            ObservationType.PHOTONVISION));
  }
}
