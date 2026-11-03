package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.fluxfig.FluxFig;
import java.util.Map;

/**
 * 2026 {@code FUEL}-only Shoot-on-the-Move ballistic solver (Team 254 / 1678 pattern) using WPILib
 * {@link InterpolatingDoubleTreeMap} tables and iterative time-of-flight velocity lead compensation
 * {@code p_virtual = p_target - v_robot * t_flight}.
 *
 * <p>Managed exclusively in {@code Flux-RoboRIO-2027/src/main/java/frc/robot/util/} since shooting
 * mechanisms exist only on the 2026 {@code FUEL} robot.
 */
public class ShotCalculator {

  /** Complete Shoot-on-the-Move aiming and mechanism setpoint solution. */
  public record ShotSolution(
      Rotation2d driveAimHeading,
      double effectiveDistanceMeters,
      double shooterRps,
      double hoodRotations,
      double timeOfFlightSeconds,
      double headingFeedforwardRadPerSec) {}

  private final InterpolatingDoubleTreeMap shooterRpsMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodRotMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofSecondsMap = new InterpolatingDoubleTreeMap();
  private double minDistMeters = Double.POSITIVE_INFINITY;
  private double maxDistMeters = 0.0;

  public ShotCalculator() {}

  /** Populates the interpolation maps from a {@link FluxFig} `range_table.csv` table. */
  public static ShotCalculator fromFluxFig(FluxFig fig) {
    ShotCalculator calc = new ShotCalculator();
    for (Map<String, String> row : fig.getTable("range_table")) {
      double dist = Double.parseDouble(row.get("distanceMeters"));
      double rps = Double.parseDouble(row.get("shooterRps"));
      double hood = Double.parseDouble(row.get("hoodRotations"));
      String tofVal =
          row.containsKey("timeOfFlightSec")
              ? row.get("timeOfFlightSec")
              : row.get("timeOfFlightSeconds");
      double tof = Double.parseDouble(tofVal);
      calc.addPoint(dist, rps, hood, tof);
    }
    return calc;
  }

  public void addPoint(
      double distanceMeters, double shooterRps, double hoodRotations, double timeOfFlightSeconds) {
    shooterRpsMap.put(distanceMeters, shooterRps);
    hoodRotMap.put(distanceMeters, hoodRotations);
    tofSecondsMap.put(distanceMeters, timeOfFlightSeconds);
    minDistMeters = Math.min(minDistMeters, distanceMeters);
    maxDistMeters = Math.max(maxDistMeters, distanceMeters);
  }

  /**
   * Computes a velocity-compensated Shoot-on-the-Move solution given current robot pose, field
   * velocity, and target translation.
   */
  public ShotSolution calculate(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetPosition) {
    Translation2d robotPos = robotPose.getTranslation();
    Translation2d toTarget = targetPosition.minus(robotPos);
    double staticDist = Math.max(1e-4, toTarget.getNorm());

    double tof = tofSecondsMap.get(staticDist);
    Translation2d virtualTarget = targetPosition;
    double effectiveDist = staticDist;

    // 3-step fixed-point lookahead iteration for p_virtual = p_target - v_robot * t_flight
    for (int i = 0; i < 3; i++) {
      Translation2d leadOffset =
          new Translation2d(
              fieldRelativeSpeeds.vxMetersPerSecond * tof,
              fieldRelativeSpeeds.vyMetersPerSecond * tof);
      virtualTarget = targetPosition.minus(leadOffset);
      effectiveDist = Math.max(1e-4, virtualTarget.minus(robotPos).getNorm());
      tof = tofSecondsMap.get(effectiveDist);
    }

    Translation2d aimVector = virtualTarget.minus(robotPos);
    Rotation2d aimHeading =
        (aimVector.getNorm() > 1e-4)
            ? new Rotation2d(aimVector.getX(), aimVector.getY())
            : robotPose.getRotation();

    // Angular velocity feedforward d(theta)/dt = (r x (-v_robot)) / |r|^2
    double rx = aimVector.getX();
    double ry = aimVector.getY();
    double rSq = Math.max(0.25, rx * rx + ry * ry);
    double omegaFf =
        (rx * (-fieldRelativeSpeeds.vyMetersPerSecond)
                - ry * (-fieldRelativeSpeeds.vxMetersPerSecond))
            / rSq;

    return new ShotSolution(
        aimHeading,
        effectiveDist,
        shooterRpsMap.get(effectiveDist),
        hoodRotMap.get(effectiveDist),
        tof,
        omegaFf);
  }

  public boolean isInRange(double distanceMeters) {
    return distanceMeters >= minDistMeters && distanceMeters <= maxDistMeters;
  }
}
