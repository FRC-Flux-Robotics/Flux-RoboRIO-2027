package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.drive.Drive;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.Superstructure;
import frc.robot.exercises.StudentExercises;
import frc.robot.exercises.StudentExercises.ShotGateDecision;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Automated Grader for the 4-Level Student Exercise Kit ({@link StudentExercises}) in {@code
 * Flux-RoboRIO-2027}.
 *
 * <p>Run via {@code ./gradlew checkExercises} or {@code ./gradlew test}.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class StudentExercisesTest {

  private static int passedCount = 0;

  @BeforeAll
  public static void initHal() {
    HAL.initialize(500, 0);
    passedCount = 0;
  }

  @Test
  @Order(1)
  public void testExercise1_fixFuelMechanismSpec() {
    MechanismSpec broken = StudentExercises.createBrokenHoodSpecExample();
    assertFalse(
        StudentExercises.isMechanismSpecSafe(broken),
        "Broken example spec (can_s1, 120A stator, inverted soft limits) must be flagged unsafe");

    MechanismSpec fixed = StudentExercises.exercise1_fixFuelMechanismSpec(broken);
    assertTrue(
        StudentExercises.isMechanismSpecSafe(fixed),
        "Exercise 1 fixed MechanismSpec must pass hardware safety rules");
    assertEquals("Mech", fixed.canBus(), "Invalid odd port can_s1 must be remapped to Mech CANivore");
    assertEquals(60.0, fixed.statorLimitAmps(), 1e-6, "Stator current clamped <= 60.0 A");
    assertEquals(40.0, fixed.supplyLimitAmps(), 1e-6, "Supply current clamped <= 40.0 A");
    assertEquals(60.0, fixed.slipCurrentAmps(), 1e-6, "Slip current clamped <= 60.0 A");
    assertTrue(fixed.minPosRot() <= fixed.maxPosRot(), "minPosRot must be <= maxPosRot");
    passedCount++;
  }

  @Test
  @Order(2)
  public void testExercise2_computeShotReadinessAndFeederCommand() {
    ShotGateDecision spinningUp =
        StudentExercises.exercise2_computeShotReadinessAndFeederCommand(
            35.0, 69.0, 5.10, 5.10, -40.0, 45.0);
    assertFalse(spinningUp.readyToFire(), "Must block firing while Shooter is still spinning up");
    assertEquals(0.0, spinningUp.feederCommandRps(), 1e-6);
    assertEquals(0.0, spinningUp.indexerCommandRps(), 1e-6);

    ShotGateDecision ready =
        StudentExercises.exercise2_computeShotReadinessAndFeederCommand(
            68.2, 69.0, 5.05, 5.10, -40.0, 45.0);
    assertTrue(ready.readyToFire(), "Must unlock firing gate when Shooter and Hood are within tolerance");
    assertEquals(-40.0, ready.feederCommandRps(), 1e-6);
    assertEquals(45.0, ready.indexerCommandRps(), 1e-6);
    passedCount++;
  }

  @Test
  @Order(3)
  public void testExercise3_computePoseAlignmentSpeeds() {
    Pose2d startPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(175.0));
    Pose2d targetPose = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(-175.0));
    ChassisSpeeds cmd =
        StudentExercises.exercise3_computePoseAlignmentSpeeds(
            startPose, targetPose, 2.5, 5.0, 3.5, 6.0);
    assertTrue(
        cmd.omegaRadiansPerSecond > 0.0,
        "Shortest path from +175 deg to -175 deg must rotate counter-clockwise (+omega)");
    assertEquals(
        0.0,
        StudentExercises.exercise3_computePoseAlignmentSpeeds(
                targetPose,
                new Pose2d(2.01, -0.01, Rotation2d.fromDegrees(-174.5)),
                2.5,
                5.0,
                3.5,
                6.0)
            .vxMetersPerSecond,
        1e-6,
        "Pose inside 2.0 cm / 1.0 deg deadband must command 0.0 m/s");
    passedCount++;
  }

  @Test
  @Order(4)
  public void testExercise4_createMultiVolleyHubAuto() {
    RobotContainer container =
        new RobotContainer(RobotIdentity.FUEL, Paths.get("src/main/deploy/configs"), true);
    Drive drive = container.getDrive();
    Superstructure ss = container.getSuperstructure();
    var multiVolleyAuto =
        StudentExercises.exercise4_createMultiVolleyHubAuto(
            drive, ss, container.getShotCalculator());

    multiVolleyAuto.initialize();
    double minX = 99.0;
    double maxY = 0.0;
    int shootingLoops = 0;
    for (int loop = 0; loop < 750 && !multiVolleyAuto.isFinished(); loop++) {
      multiVolleyAuto.execute();
      drive.periodic();
      container.getIntake().periodic();
      container.getTilter().periodic();
      container.getIndexer().periodic();
      container.getFeeder().periodic();
      container.getShooter().periodic();
      container.getHood().periodic();
      ss.periodic();
      Pose2d p = drive.getPose();
      minX = Math.min(minX, p.getX());
      maxY = Math.max(maxY, p.getY());
      if (ss.getState() == Superstructure.State.SHOOTING) {
        shootingLoops++;
      }
    }
    multiVolleyAuto.end(false);
    assertTrue(
        minX < 1.35 && maxY > 5.8,
        "Exercise 4 2026 auto must follow PathPlanner 'Hub to Depot' path into Blue Depot");
    assertTrue(shootingLoops >= 20, "Exercise 4 2026 auto must execute SHOOTING volleys");
    passedCount++;
  }

  @AfterAll
  public static void reportScoreboard() {
    PrecisionReporter.recordExactTarget(
        2,
        "tier2_student_exercises_passed",
        "Student Exercises Completed (1..4)",
        passedCount,
        4.0,
        0.0,
        "exercises");

    System.out.println(
        "FLUX 10413 Student Exercise Scoreboard: " + passedCount + "/4 EXERCISES PASSED");
  }
}
