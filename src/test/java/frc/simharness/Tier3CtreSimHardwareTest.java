package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.commands.HardStopHomingCmd;
import frc.lib.drive.Drive;
import frc.lib.drive.GyroIO.GyroIOInputs;
import frc.lib.drive.GyroIOPigeon2;
import frc.lib.drive.ModuleIO.ModuleIOInputs;
import frc.lib.drive.ModuleIOTalonFX;
import frc.lib.drive.PhoenixOdometryThread;
import frc.lib.drive.SwerveConfig;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO.MechanismIOInputs;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tier 3 Verification Gate: Real CTRE Phoenix 6 C++ Hardware Simulation (libCTRE_SimProTalonFX.so)
 * + WPILib DCMotorSim exercising {@link MechanismIOTalonFX} directly.
 *
 * <p>Verifies that applied TalonFXConfiguration registers enforce 60A Stator / 40A Supply limits
 * (preventing the NYTR 120A SlipCurrent brownout bug) and tests 250 Hz closed-loop velocity
 * tracking precision.
 */
public class Tier3CtreSimHardwareTest {
  private static TalonFX keepAliveTalon;

  @BeforeAll
  public static void initHal() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0), "WPILib HAL must initialize in headless Linux container");
    SimHooks.stepTiming(0.020);
    keepAliveTalon = new TalonFX(62, new CANBus("rio"));
    Unmanaged.feedEnable(100);
    Thread.sleep(200);
  }

  @BeforeEach
  public void settleCanBusBetweenTests() throws InterruptedException {
    PhoenixOdometryThread.getInstance().reset();
    PhoenixSignals.reset();
    PhoenixUtil.resetConfigBudgetForTest();
    Unmanaged.feedEnable(100);
    Thread.sleep(150);
  }

  @AfterAll
  public static void closeKeepAlive() {
    PhoenixOdometryThread.getInstance().reset();
    PhoenixSignals.reset();
    if (keepAliveTalon != null) {
      keepAliveTalon.close();
      keepAliveTalon = null;
    }
  }

  @Test
  public void testMechanismIoTalonFxConfigReadbackAndSlipCurrentGuard() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec swerveSpec = fig.getMechanismSpec("FUEL", "SwerveDriveFL");
    MechanismSpec rioSpec =
        new MechanismSpec(
            swerveSpec.robot(),
            swerveSpec.mechanism(),
            31,
            "rio",
            swerveSpec.statorLimitAmps(),
            swerveSpec.supplyLimitAmps(),
            swerveSpec.slipCurrentAmps(),
            swerveSpec.gravityType(),
            swerveSpec.kP(),
            swerveSpec.kI(),
            swerveSpec.kD(),
            swerveSpec.kS(),
            swerveSpec.kV(),
            swerveSpec.kA(),
            swerveSpec.kG(),
            swerveSpec.minPosRot(),
            swerveSpec.maxPosRot());

    try (MechanismIOTalonFX io = new MechanismIOTalonFX(rioSpec)) {
      TalonFXConfiguration readback = io.readAppliedConfig();

      assertTrue(readback.CurrentLimits.StatorCurrentLimitEnable);
      assertTrue(readback.CurrentLimits.SupplyCurrentLimitEnable);

      PrecisionReporter.recordExactTarget(
          3,
          "tier3_talonfx_stator_limit_amps",
          "CTRE TalonFX Stator Limit Readback",
          readback.CurrentLimits.StatorCurrentLimit,
          60.0,
          0.1,
          "A");
      PrecisionReporter.recordExactTarget(
          3,
          "tier3_talonfx_supply_limit_amps",
          "CTRE TalonFX Supply Limit Readback",
          readback.CurrentLimits.SupplyCurrentLimit,
          40.0,
          0.1,
          "A");

      assertEquals(60.0, readback.CurrentLimits.StatorCurrentLimit, 0.1);
      assertEquals(40.0, readback.CurrentLimits.SupplyCurrentLimit, 0.1);
      assertEquals(
          11.0,
          readback.Voltage.PeakForwardVoltage,
          0.05,
          "Mechanism leader PeakForwardVoltage must be clamped to +11.0 V");
      assertEquals(
          -11.0,
          readback.Voltage.PeakReverseVoltage,
          0.05,
          "Mechanism leader PeakReverseVoltage must be clamped to -11.0 V");
      assertEquals(
          NeutralModeValue.Brake,
          readback.MotorOutput.NeutralMode,
          "Non-flywheel mechanisms must default to Brake neutral mode");
    }

    // Verify FUEL Shooter flywheel configures Coast neutral mode and +-11V peak on leader + follower
    MechanismSpec baseShooter = fig.getMechanismSpec("FUEL", "Shooter");
    MechanismSpec rioShooter =
        new MechanismSpec(
            baseShooter.robot(),
            baseShooter.mechanism(),
            35,
            "rio",
            baseShooter.statorLimitAmps(),
            baseShooter.supplyLimitAmps(),
            baseShooter.slipCurrentAmps(),
            baseShooter.gravityType(),
            baseShooter.kP(),
            baseShooter.kI(),
            baseShooter.kD(),
            baseShooter.kS(),
            baseShooter.kV(),
            baseShooter.kA(),
            baseShooter.kG(),
            baseShooter.minPosRot(),
            baseShooter.maxPosRot(),
            36,
            true,
            baseShooter.motorInverted(),
            baseShooter.gearRatio());
    try (MechanismIOTalonFX shooterIo = new MechanismIOTalonFX(rioShooter)) {
      TalonFXConfiguration shooterReadback = shooterIo.readAppliedConfig();
      TalonFXConfiguration followerReadback = shooterIo.readAppliedFollowerConfig();
      assertEquals(
          NeutralModeValue.Coast,
          shooterReadback.MotorOutput.NeutralMode,
          "FUEL Shooter flywheel must apply NeutralModeValue.Coast in TalonFX config");
      assertEquals(
          11.0,
          shooterReadback.Voltage.PeakForwardVoltage,
          0.05,
          "Shooter leader PeakForwardVoltage must be +11.0 V");
      assertEquals(
          -11.0,
          shooterReadback.Voltage.PeakReverseVoltage,
          0.05,
          "Shooter leader PeakReverseVoltage must be -11.0 V");
      assertEquals(
          11.0,
          followerReadback.Voltage.PeakForwardVoltage,
          0.05,
          "Shooter follower PeakForwardVoltage must be +11.0 V");
      assertEquals(
          -11.0,
          followerReadback.Voltage.PeakReverseVoltage,
          0.05,
          "Shooter follower PeakReverseVoltage must be -11.0 V");
    }

    // Verify that attempting the NYTR 120A SlipCurrent overwrite is caught immediately
    MechanismSpec buggyNytrSpec =
        new MechanismSpec(
            "FUEL", "BuggySwerve", 1, "rio", 60.0, 40.0, 120.0, "NONE",
            0.22, 0.0, 0.0, 0.14, 0.12, 0.015, 0.0, -1000.0, 1000.0);
    assertThrows(
        IllegalArgumentException.class,
        () -> new MechanismIOTalonFX(buggyNytrSpec),
        "MechanismIOTalonFX must reject SlipCurrent > 60A");
  }

  @Test
  public void testClosedLoopKrakenX60VelocityTrackingAndStatorClampInCtreSim()
      throws InterruptedException {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec baseSpec = fig.getMechanismSpec("COMP_2027", "SwerveDriveFL");
    double gearRatio = baseSpec.gearRatio();
    MechanismSpec velocitySpec =
        new MechanismSpec(
            baseSpec.robot(),
            baseSpec.mechanism(),
            41,
            "rio",
            baseSpec.statorLimitAmps(),
            baseSpec.supplyLimitAmps(),
            baseSpec.slipCurrentAmps(),
            baseSpec.gravityType(),
            baseSpec.kP(),
            baseSpec.kI(),
            baseSpec.kD(),
            baseSpec.kS(),
            baseSpec.kV(),
            baseSpec.kA(),
            baseSpec.kG(),
            baseSpec.minPosRot(),
            baseSpec.maxPosRot(),
            -1,
            false,
            baseSpec.motorInverted(),
            gearRatio);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    try (MechanismIOTalonFX io = new MechanismIOTalonFX(velocitySpec)) {
      TalonFXSimState simState = io.getMotor().getSimState();
      simState.setSupplyVoltage(12.0);

      DCMotor kraken = DCMotor.getKrakenX60Foc(1);
      DCMotorSim plant =
          new DCMotorSim(LinearSystemId.createDCMotorSystem(kraken, 0.025, gearRatio), kraken);

      MechanismIOInputs inputs = new MechanismIOInputs();
      double targetRps = 12.0; // Geared wheel RPS (~3.83 m/s on 2-inch radius wheels)

      double peakStatorAmps = 0.0;
      double dt = 0.004; // 250 Hz physics loop
      for (int step = 0; step < 300; step++) {
        Unmanaged.feedEnable(100);
        io.setVelocity(targetRps);
        Thread.sleep(2);

        double motorVoltage = simState.getMotorVoltage();
        plant.setInputVoltage(motorVoltage);
        plant.update(dt);

        double wheelPosRot = plant.getAngularPositionRotations();
        double wheelVelRps = plant.getAngularVelocityRPM() / 60.0;
        simState.setRawRotorPosition(wheelPosRot * gearRatio);
        simState.setRotorVelocity(wheelVelRps * gearRatio);

        PhoenixSignals.refreshAll();
        io.updateInputs(inputs);
        peakStatorAmps = Math.max(peakStatorAmps, Math.abs(inputs.statorCurrentA));
      }

      double finalVelRps = inputs.velocityRPS;
      double trackingErrRps = Math.abs(finalVelRps - targetRps);

      PrecisionReporter.recordLowerIsBetter(
          3,
          "tier3_kraken_velocity_tracking_err_rps",
          "Kraken X60 Sim Velocity Error",
          trackingErrRps,
          0.05,
          1.50,
          "RPS");
      PrecisionReporter.recordLowerIsBetter(
          3,
          "tier3_kraken_peak_stator_current_amps",
          "Kraken X60 Sim Peak Stator Current",
          peakStatorAmps,
          45.0,
          61.0,
          "A");

      assertTrue(
          trackingErrRps < 1.50,
          "Kraken X60 closed-loop velocity error must be < 1.5 RPS (measured: "
              + trackingErrRps
              + ")");
      assertTrue(
          peakStatorAmps <= 61.0,
          "Kraken X60 stator current must respect 60A limit (measured: " + peakStatorAmps + ")");
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  public void testGearedFuelSwerveDriveTracksFullStickVelocityInCtreSim()
      throws InterruptedException {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec fuelDriveSpec = fig.getMechanismSpec("FUEL", "SwerveDriveFL");
    assertEquals(6.3947, fuelDriveSpec.gearRatio(), 1e-3);
    assertEquals(0.79, fuelDriveSpec.kV(), 1e-3);
    assertEquals(0.90, fuelDriveSpec.kP(), 1e-3);

    SwerveConfig baseCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    SwerveConfig rioCfg =
        new SwerveConfig(
            "rio",
            baseCfg.pigeonId(),
            baseCfg.trackWidthMeters(),
            baseCfg.wheelBaseMeters(),
            baseCfg.wheelRadiusMeters(),
            baseCfg.driveGearRatio(),
            baseCfg.turnGearRatio(),
            baseCfg.maxLinearSpeedMps(),
            baseCfg.maxAngularSpeedRadPerSec(),
            baseCfg.driveStatorLimitAmps(),
            baseCfg.driveSupplyLimitAmps(),
            baseCfg.driveSlipCurrentAmps(),
            baseCfg.turnStatorLimitAmps(),
            baseCfg.turnSupplyLimitAmps(),
            baseCfg.driveKp(),
            baseCfg.driveKi(),
            baseCfg.driveKd(),
            baseCfg.driveKs(),
            baseCfg.driveKv(),
            baseCfg.driveKa(),
            baseCfg.turnKp(),
            baseCfg.turnKd(),
            baseCfg.corners());

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    try (ModuleIOTalonFX modFl = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[0])) {
      TalonFXSimState driveSim = modFl.getDriveTalon().getSimState();
      driveSim.setSupplyVoltage(12.2);

      DCMotor kraken = DCMotor.getKrakenX60Foc(1);
      double gearRatio = rioCfg.driveGearRatio();
      DCMotorSim wheelPlant =
          new DCMotorSim(LinearSystemId.createDCMotorSystem(kraken, 0.025, gearRatio), kraken);

      double targetLinearMps = 4.5;
      double targetRadPerSec = targetLinearMps / rioCfg.wheelRadiusMeters();
      ModuleIOInputs inputs = new ModuleIOInputs();

      for (int step = 0; step < 350; step++) {
        Unmanaged.feedEnable(100);
        modFl.setDriveVelocity(targetRadPerSec);
        Thread.sleep(2);

        double motorVolts = driveSim.getMotorVoltage();
        wheelPlant.setInputVoltage(motorVolts);
        wheelPlant.update(0.004);

        double wheelPosRot = wheelPlant.getAngularPositionRotations();
        double wheelVelRps = wheelPlant.getAngularVelocityRPM() / 60.0;
        driveSim.setRawRotorPosition(wheelPosRot * gearRatio);
        driveSim.setRotorVelocity(wheelVelRps * gearRatio);

        PhoenixSignals.refreshAll();
        modFl.updateInputs(inputs);
      }

      double actualLinearMps = inputs.driveVelocityRadPerSec * rioCfg.wheelRadiusMeters();
      double actualWheelRps = inputs.driveVelocityRadPerSec / (2.0 * Math.PI);
      assertTrue(
          actualLinearMps >= 4.15 && actualLinearMps <= 4.85,
          "Geared FUEL SwerveDriveFL must track full-stick 4.5 m/s (~14.1 wheel RPS), not stall at"
              + " ~1.55 m/s (measured "
              + actualLinearMps
              + " m/s, "
              + actualWheelRps
              + " RPS)");
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  public void testHoodHomingIgnoresReverseSoftLimitAtZeroRotInCtreSim()
      throws InterruptedException {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec baseHood = fig.getMechanismSpec("FUEL", "Hood");
    MechanismSpec rioHood =
        new MechanismSpec(
            baseHood.robot(),
            baseHood.mechanism(),
            43,
            "rio",
            baseHood.statorLimitAmps(),
            baseHood.supplyLimitAmps(),
            baseHood.slipCurrentAmps(),
            baseHood.gravityType(),
            baseHood.kP(),
            baseHood.kI(),
            baseHood.kD(),
            baseHood.kS(),
            baseHood.kV(),
            baseHood.kA(),
            baseHood.kG(),
            baseHood.minPosRot(),
            baseHood.maxPosRot(),
            -1,
            false,
            baseHood.motorInverted(),
            baseHood.gearRatio());

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    try (MechanismIOTalonFX hoodIo = new MechanismIOTalonFX(rioHood)) {
      TalonFXConfiguration cfg = hoodIo.readAppliedConfig();
      assertTrue(
          cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable,
          "FUEL Hood must keep ReverseSoftLimitEnable active");
      assertEquals(0.0, cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold, 1e-4);

      TalonFXSimState simState = hoodIo.getMotor().getSimState();
      simState.setSupplyVoltage(12.2);
      simState.setRawRotorPosition(-0.02);
      simState.setRotorVelocity(0.0);
      Thread.sleep(30);

      // Normal open-loop reverse voltage at/past the 0.0 rot reverse soft limit must stay blocked
      for (int i = 0; i < 20; i++) {
        Unmanaged.feedEnable(100);
        hoodIo.setVoltage(HardStopHomingCmd.DEFAULT_HOMING_VOLTS);
        Thread.sleep(3);
      }
      assertEquals(
          0.0,
          simState.getMotorVoltage(),
          0.05,
          "Normal setVoltage() must remain blocked by reverse soft limit at/below 0.0 rot");

      // HardStopHomingCmd starting from 0.0 rot must ignore soft limits and drive into hard stop
      PositionMechanism hoodMech = new PositionMechanism(hoodIo, rioHood);
      HardStopHomingCmd homingCmd = new HardStopHomingCmd(hoodMech);
      simState.setRawRotorPosition(0.0);
      simState.setRotorVelocity(0.0);
      homingCmd.initialize();

      double mostNegativeVolts = 0.0;
      double mostNegativePastSoftLimitVolts = 0.0;
      for (int i = 0; i < 120 && !homingCmd.isFinished(); i++) {
        Unmanaged.feedEnable(100);
        double simPosRot = (i == 0) ? 0.0 : -0.02;
        simState.setRawRotorPosition(simPosRot);
        simState.setRotorVelocity(0.0);
        homingCmd.execute();
        Thread.sleep(4);
        PhoenixSignals.refreshAll();
        hoodMech.periodic();
        double v = simState.getMotorVoltage();
        mostNegativeVolts = Math.min(mostNegativeVolts, v);
        if (i > 0) {
          mostNegativePastSoftLimitVolts = Math.min(mostNegativePastSoftLimitVolts, v);
        }
      }
      homingCmd.end(false);

      assertTrue(
          mostNegativeVolts <= -0.8 && mostNegativePastSoftLimitVolts <= -0.8,
          "HardStopHomingCmd must apply non-zero negative voltage across the 0.0 rot soft limit"
              + " (measured min="
              + mostNegativeVolts
              + " V, pastSoftLimit="
              + mostNegativePastSoftLimitVolts
              + " V)");
      assertTrue(homingCmd.isFinished(), "Hood homing command must finish without timing out");
      assertTrue(homingCmd.isHomed(), "Hood homing starting at 0.0 rot must succeed");
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  public void testOptimizeBusPreservesCancoderAndFollowerLeaderFrequenciesInCtreSim() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig baseCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    SwerveConfig rioCfg =
        new SwerveConfig(
            "rio",
            baseCfg.pigeonId(),
            baseCfg.trackWidthMeters(),
            baseCfg.wheelBaseMeters(),
            baseCfg.wheelRadiusMeters(),
            baseCfg.driveGearRatio(),
            baseCfg.turnGearRatio(),
            baseCfg.maxLinearSpeedMps(),
            baseCfg.maxAngularSpeedRadPerSec(),
            baseCfg.driveStatorLimitAmps(),
            baseCfg.driveSupplyLimitAmps(),
            baseCfg.driveSlipCurrentAmps(),
            baseCfg.turnStatorLimitAmps(),
            baseCfg.turnSupplyLimitAmps(),
            baseCfg.driveKp(),
            baseCfg.driveKi(),
            baseCfg.driveKd(),
            baseCfg.driveKs(),
            baseCfg.driveKv(),
            baseCfg.driveKa(),
            baseCfg.turnKp(),
            baseCfg.turnKd(),
            baseCfg.corners());

    MechanismSpec baseShooter = fig.getMechanismSpec("FUEL", "Shooter");
    MechanismSpec rioShooter =
        new MechanismSpec(
            baseShooter.robot(),
            baseShooter.mechanism(),
            45,
            "rio",
            baseShooter.statorLimitAmps(),
            baseShooter.supplyLimitAmps(),
            baseShooter.slipCurrentAmps(),
            baseShooter.gravityType(),
            baseShooter.kP(),
            baseShooter.kI(),
            baseShooter.kD(),
            baseShooter.kS(),
            baseShooter.kV(),
            baseShooter.kA(),
            baseShooter.kG(),
            baseShooter.minPosRot(),
            baseShooter.maxPosRot(),
            46,
            true,
            baseShooter.motorInverted(),
            baseShooter.gearRatio());

    try (ModuleIOTalonFX modFl = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[0]);
        MechanismIOTalonFX shooterIo = new MechanismIOTalonFX(rioShooter)) {
      double cancoderPosHz = modFl.getCancoder().getPosition().getAppliedUpdateFrequency();
      double cancoderVelHz = modFl.getCancoder().getVelocity().getAppliedUpdateFrequency();
      assertTrue(
          cancoderPosHz >= 100.0,
          "Swerve CANcoder position frequency must remain >= 100 Hz after optimizeBus (got "
              + cancoderPosHz
              + " Hz)");
      assertTrue(
          cancoderVelHz >= 100.0,
          "Swerve CANcoder velocity frequency must remain >= 100 Hz after optimizeBus (got "
              + cancoderVelHz
              + " Hz)");

      double dutyHz = shooterIo.getMotor().getDutyCycle().getAppliedUpdateFrequency();
      double voltsHz = shooterIo.getMotor().getMotorVoltage().getAppliedUpdateFrequency();
      double torqueHz = shooterIo.getMotor().getTorqueCurrent().getAppliedUpdateFrequency();
      assertTrue(
          dutyHz >= 100.0,
          "Shooter leader DutyCycle frequency must remain >= 100 Hz for follower (got "
              + dutyHz
              + " Hz)");
      assertTrue(
          voltsHz >= 100.0,
          "Shooter leader MotorVoltage frequency must remain >= 100 Hz for follower (got "
              + voltsHz
              + " Hz)");
      assertTrue(
          torqueHz >= 100.0,
          "Shooter leader TorqueCurrent frequency must remain >= 100 Hz for follower (got "
              + torqueHz
              + " Hz)");
    }
  }

  @Test
  public void testModuleIoTalonFxAndGyroIoPigeon2Readback() throws InterruptedException {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig baseCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    SwerveConfig rioCfg =
        new SwerveConfig(
            "rio",
            baseCfg.pigeonId(),
            baseCfg.trackWidthMeters(),
            baseCfg.wheelBaseMeters(),
            baseCfg.wheelRadiusMeters(),
            baseCfg.driveGearRatio(),
            baseCfg.turnGearRatio(),
            baseCfg.maxLinearSpeedMps(),
            baseCfg.maxAngularSpeedRadPerSec(),
            baseCfg.driveStatorLimitAmps(),
            baseCfg.driveSupplyLimitAmps(),
            baseCfg.driveSlipCurrentAmps(),
            baseCfg.turnStatorLimitAmps(),
            baseCfg.turnSupplyLimitAmps(),
            baseCfg.driveKp(),
            baseCfg.driveKi(),
            baseCfg.driveKd(),
            baseCfg.driveKs(),
            baseCfg.driveKv(),
            baseCfg.driveKa(),
            baseCfg.turnKp(),
            baseCfg.turnKd(),
            baseCfg.corners());

    try (ModuleIOTalonFX modFl = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[0]);
        ModuleIOTalonFX modFr = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[1]);
        ModuleIOTalonFX modBl = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[2]);
        ModuleIOTalonFX modBr = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[3]);
        GyroIOPigeon2 gyroIo = new GyroIOPigeon2(55, "rio")) {
      TalonFXConfiguration flDriveCfg = modFl.readAppliedDriveConfig();
      TalonFXConfiguration frDriveCfg = modFr.readAppliedDriveConfig();
      assertTrue(flDriveCfg.CurrentLimits.StatorCurrentLimitEnable);
      assertTrue(flDriveCfg.CurrentLimits.SupplyCurrentLimitEnable);
      assertEquals(60.0, flDriveCfg.CurrentLimits.StatorCurrentLimit, 0.1);
      assertEquals(40.0, flDriveCfg.CurrentLimits.SupplyCurrentLimit, 0.1);
      assertEquals(
          false,
          modFl.isUsingFusedCANcoder(),
          "Non-FD / unlicensed CAN 2.0 rio fallback must select RemoteCANcoder instead of FusedCANcoder");

      // Verify left vs. right drive motor inversion and non-zero CANcoder magnet offset on hardware
      CANcoderConfiguration flCancoderCfg = new CANcoderConfiguration();
      modFl.getCancoder().getConfigurator().refresh(flCancoderCfg, 0.100);

      // Verify end-to-end Drive odometry integration with real ModuleIOTalonFX + PhoenixOdometryThread
      // WITHOUT manually calling PhoenixOdometryThread.sampleNow() from the test!
      Drive hardwareDrive = new Drive(rioCfg, gyroIo, modFl, modFr, modBl, modBr);
      hardwareDrive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
      ModuleIOTalonFX[] allMods = {modFl, modFr, modBl, modBr};
      double rotorPos = 0.0;
      for (int cycle = 0; cycle < 20; cycle++) {
        Unmanaged.feedEnable(100);
        rotorPos += 1.25; // advance wheel rotation
        for (ModuleIOTalonFX m : allMods) {
          m.getDriveTalon().getSimState().setSupplyVoltage(12.2);
          m.getDriveTalon().getSimState().setRawRotorPosition(rotorPos);
          m.getDriveTalon().getSimState().setRotorVelocity(62.5);
        }
        Thread.sleep(6);
        PhoenixSignals.refreshAll();
        hardwareDrive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0));
        hardwareDrive.periodic();
      }
      PrecisionReporter.recordHigherIsBetter(
          3,
          "tier3_hardware_odometry_samples",
          "Real ModuleIOTalonFX Odometry Samples",
          hardwareDrive.getOdometrySampleCount(),
          20.0,
          20.0,
          "samples");

      assertEquals(
          InvertedValue.CounterClockwise_Positive,
          flDriveCfg.MotorOutput.Inverted,
          "FL drive motor must be CounterClockwise_Positive");
      assertEquals(
          InvertedValue.Clockwise_Positive,
          frDriveCfg.MotorOutput.Inverted,
          "FR drive motor must be Clockwise_Positive (right-side bevel gear inversion)");
      assertEquals(
          0.105225,
          flCancoderCfg.MagnetSensor.MagnetOffset,
          1e-3,
          "FL CANcoder hardware configurator must apply calibrated magnet offset");
      assertTrue(
          hardwareDrive.getOdometrySampleCount() >= 20,
          "Drive + ModuleIOTalonFX must integrate >= 20 odometry samples automatically (actual="
              + hardwareDrive.getOdometrySampleCount()
              + ")");
      assertTrue(
          hardwareDrive.getPose().getX() > 0.20 && hardwareDrive.getPose().getX() < 2.00,
          "Drive pose X must advance in [0.20m, 2.00m] on real ModuleIOTalonFX hardware IOs without 2*pi radian inflation (actual="
              + hardwareDrive.getPose().getX()
              + ")");

      // Verify ModuleIOTalonFX and GyroIOPigeon2 populate replayable odometry timestamps
      PhoenixOdometryThread.getInstance().sampleOnce(12.345);
      ModuleIOInputs modInputs = new ModuleIOInputs();
      GyroIOInputs gyroInputs = new GyroIOInputs();
      PhoenixSignals.refreshAll();
      modFl.updateInputs(modInputs);
      gyroIo.updateInputs(gyroInputs);
      assertTrue(
          modInputs.odometryTimestamps.length > 0,
          "ModuleIOTalonFX must populate non-empty odometryTimestamps for AdvantageKit replay");
      assertEquals(
          modInputs.odometryTimestamps.length,
          modInputs.odometryDrivePositionsRad.length,
          "ModuleIOTalonFX odometryTimestamps length must match drive position samples");
      assertTrue(
          gyroInputs.odometryYawTimestamps.length > 0,
          "GyroIOPigeon2 must populate non-empty odometryYawTimestamps for AdvantageKit replay");
      assertEquals(
          gyroInputs.odometryYawTimestamps.length,
          gyroInputs.odometryYawPositions.length,
          "GyroIOPigeon2 odometryYawTimestamps length must match yaw position samples");

      // HB-01: Assert ZERO StatusSignal objects are registered in both PhoenixSignals and
      // PhoenixOdometryThread across all 4 swerve corners + Pigeon 2 (18 odometry signals).
      java.util.List<BaseStatusSignal> signals50Hz = PhoenixSignals.getRegisteredSignals();
      java.util.List<BaseStatusSignal> signals250Hz =
          PhoenixOdometryThread.getInstance().getRegisteredSignals();
      assertEquals(
          18,
          signals250Hz.size(),
          "Expected 18 odometry signals (4 corners x 4 + Pigeon yaw/yawVel) on PhoenixOdometryThread");
      int overlapCount = 0;
      for (BaseStatusSignal sig50 : signals50Hz) {
        for (BaseStatusSignal sig250 : signals250Hz) {
          if (sig50 == sig250) {
            overlapCount++;
          }
        }
      }
      assertEquals(
          0,
          overlapCount,
          "Zero StatusSignal objects may be registered in both PhoenixSignals and PhoenixOdometryThread");
    }
  }

  @Test
  public void testGyroIoPigeon2PreservesMountPoseAndGyroTrimOnBoot() {
    // Pre-flash a non-default Tuner X MountPose & GyroTrim onto Pigeon 2 ID 56
    Pigeon2Configuration preFlashed = new Pigeon2Configuration();
    preFlashed.MountPose.MountPoseYaw = 90.0;
    preFlashed.MountPose.MountPosePitch = 12.5;
    preFlashed.MountPose.MountPoseRoll = -7.25;
    preFlashed.GyroTrim.GyroScalarZ = 1.75;

    try (Pigeon2 preFlashDevice = new Pigeon2(56, new CANBus("rio"))) {
      PhoenixUtil.tryUntilOk(
          "PreFlash-Pigeon2-56",
          () -> preFlashDevice.getConfigurator().apply(preFlashed, 0.100),
          5);
    }

    // Booting GyroIOPigeon2 must preserve flash MountPose and GyroTrim without factory-wiping
    try (GyroIOPigeon2 gyroIo = new GyroIOPigeon2(56, "rio")) {
      Pigeon2Configuration afterBoot = new Pigeon2Configuration();
      PhoenixUtil.tryUntilOk(
          "Readback-Pigeon2-56",
          () -> gyroIo.getPigeon().getConfigurator().refresh(afterBoot, 0.100),
          5);
      assertEquals(
          90.0,
          afterBoot.MountPose.MountPoseYaw,
          0.1,
          "GyroIOPigeon2 must preserve Tuner X flash MountPoseYaw on boot");
      assertEquals(
          12.5,
          afterBoot.MountPose.MountPosePitch,
          0.1,
          "GyroIOPigeon2 must preserve Tuner X flash MountPosePitch on boot");
      assertEquals(
          -7.25,
          afterBoot.MountPose.MountPoseRoll,
          0.1,
          "GyroIOPigeon2 must preserve Tuner X flash MountPoseRoll on boot");
      assertEquals(
          1.75,
          afterBoot.GyroTrim.GyroScalarZ,
          0.05,
          "GyroIOPigeon2 must preserve Tuner X flash GyroScalarZ trim on boot");
    }
  }
}
