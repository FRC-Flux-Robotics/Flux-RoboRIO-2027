package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import frc.robot.Superstructure;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * 2026 {@code FUEL}-only automated range-table calibration command ({@code ShotCalibrationCmd}).
 *
 * <p>Steps through 5 target distances (or records the current distance to the Hub), commands the
 * {@link Superstructure} to {@code PREPPING_SHOT} (never feeding the hopper), and records verified
 * {@code (distanceMeters, shooterRps, hoodRotations, timeOfFlightSeconds)} calibration rows to
 * {@code ShotCalibration/*} so `FluxProcessor --calibrate-sim` or `FluxFig` can promote them into
 * {@code range_table.csv}.
 */
public class ShotCalibrationCmd extends Command {
  public static final double[] CALIBRATION_DISTANCES_METERS = {1.22, 1.93, 2.84, 3.76, 4.67};
  public static final int LOOPS_PER_DISTANCE_STEP = 12;

  private final Drive drive;
  private final Superstructure superstructure;
  private final ShotCalculator shotCalculator;
  private final Supplier<Translation2d> targetSupplier;

  private final List<String> recordedCsvRows = new ArrayList<>();
  private int loopCount = 0;

  public ShotCalibrationCmd(
      Drive drive,
      Superstructure superstructure,
      ShotCalculator shotCalculator,
      Supplier<Translation2d> targetSupplier) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.shotCalculator = shotCalculator;
    this.targetSupplier = targetSupplier;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    loopCount = 0;
    recordedCsvRows.clear();
    Logger.recordOutput("ShotCalibration/Completed", false);
    Logger.recordOutput("ShotCalibration/RecordedPoints", 0);
  }

  @Override
  public void execute() {
    loopCount++;
    int stepIdx =
        Math.min(
            CALIBRATION_DISTANCES_METERS.length - 1,
            (loopCount - 1) / LOOPS_PER_DISTANCE_STEP);
    double testDist = CALIBRATION_DISTANCES_METERS[stepIdx];

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getChassisSpeeds(), drive.getPose().getRotation());
    ShotSolution sol =
        shotCalculator.calculate(
            drive.getPose(),
            fieldSpeeds,
            drive.getPose().getTranslation().plus(new Translation2d(testDist, 0.0)));
    double headingErrRad =
        Math.abs(
            MathUtil.angleModulus(
                sol.driveAimHeading().minus(drive.getPose().getRotation()).getRadians()));
    superstructure.applyShotSolution(
        sol, false, headingErrRad, shotCalculator.isInRange(sol.effectiveDistanceMeters()));

    if (loopCount % LOOPS_PER_DISTANCE_STEP == 0) {
      String row =
          String.format(
              "%.2f,%.2f,%.2f,%.3f",
              testDist, sol.shooterRps(), sol.hoodRotations(), sol.timeOfFlightSeconds());
      recordedCsvRows.add(row);
      Logger.recordOutput("ShotCalibration/LatestRow", row);
      Logger.recordOutput("ShotCalibration/RecordedPoints", recordedCsvRows.size());
    }
    Logger.recordOutput("ShotCalibration/TargetHubMeters", targetSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return loopCount >= CALIBRATION_DISTANCES_METERS.length * LOOPS_PER_DISTANCE_STEP;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestState(Superstructure.State.IDLE);
    Logger.recordOutput("ShotCalibration/Completed", !interrupted && isFinished());
  }

  public List<String> getRecordedCsvRows() {
    return Collections.unmodifiableList(recordedCsvRows);
  }
}
