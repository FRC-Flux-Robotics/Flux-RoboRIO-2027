package frc.lib.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.DoubleSupplier;

/**
 * WPILib {@link DCMotorSim} drive + steer physics simulation of {@link ModuleIO} parameterized on
 * loop {@code dt}, real steer {@code kP}/{@code kD} gains, {@link frc.lib.util.VirtualBatterySim}
 * rail-voltage coupling, and physical Kraken X60 FOC stator/supply current clamping.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double TARGET_SUBSTEP_DT_SEC = 0.004;
  private static final double MIN_RAIL_VOLTS = 4.5;
  private static final double NOMINAL_RAIL_VOLTS = 12.0;
  private static final double STEER_KS_VOLTS = 0.1;

  private final SwerveConfig config;
  private final DCMotor driveMotor;
  private final DCMotor turnMotor;
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private final DoubleSupplier railVoltageSupplier;

  private double loopPeriodSec = 0.020;
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveTargetVelRadPerSec = 0.0;
  private Rotation2d turnTargetAngle = Rotation2d.kZero;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private double activeStatorLimitAmps;
  private double runtimeDriveKp;

  private double[] lastOdometryTimestamps = new double[0];
  private double[] lastOdometryDrivePositionsRad = new double[0];
  private Rotation2d[] lastOdometryTurnPositions = new Rotation2d[0];
  private double lastOdometryEndSec = Double.NaN;

  public ModuleIOSim(SwerveConfig config) {
    this(config, () -> NOMINAL_RAIL_VOLTS);
  }

  public ModuleIOSim(SwerveConfig config, DoubleSupplier railVoltageSupplier) {
    this.config = config;
    this.railVoltageSupplier =
        (railVoltageSupplier != null) ? railVoltageSupplier : () -> NOMINAL_RAIL_VOLTS;
    this.activeStatorLimitAmps =
        Math.min(config.driveStatorLimitAmps(), config.driveSlipCurrentAmps());
    this.runtimeDriveKp = config.driveKp();
    this.driveMotor = DCMotor.getKrakenX60Foc(1);
    this.turnMotor = DCMotor.getKrakenX60Foc(1);
    // 135 lb (~62 kg) robot with battery & bumpers distributed across 4 x 2" wheels (~0.040 kg*m^2)
    // keeps 60 A stator braking near physical ~0.96 g FRC carpet traction (~9.45 m/s^2).
    this.driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotor, 0.040, config.driveGearRatio()),
            driveMotor);
    this.turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotor, 0.004, config.turnGearRatio()),
            turnMotor);
  }

  @Override
  public void setLoopPeriodSeconds(double loopPeriodSec) {
    if (Double.isFinite(loopPeriodSec) && loopPeriodSec > 0.0) {
      this.loopPeriodSec = MathUtil.clamp(loopPeriodSec, 0.002, 0.100);
    }
  }

  public double getLoopPeriodSeconds() {
    return loopPeriodSec;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveTargetVelRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d angle) {
    turnClosedLoop = true;
    turnTargetAngle = angle;
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = MathUtil.clamp(volts, -NOMINAL_RAIL_VOLTS, NOMINAL_RAIL_VOLTS);
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = MathUtil.clamp(volts, -NOMINAL_RAIL_VOLTS, NOMINAL_RAIL_VOLTS);
  }

  @Override
  public void setDriveKp(double kP) {
    if (Double.isFinite(kP) && kP >= 0.0) {
      this.runtimeDriveKp = kP;
    }
  }

  public double getRuntimeDriveKp() {
    return runtimeDriveKp;
  }

  public double[] getLastOdometryTimestamps() {
    return lastOdometryTimestamps;
  }

  public double[] getLastOdometryDrivePositionsRad() {
    return lastOdometryDrivePositionsRad;
  }

  public Rotation2d[] getLastOdometryTurnPositions() {
    return lastOdometryTurnPositions;
  }

  @Override
  public void stop() {
    driveClosedLoop = false;
    turnClosedLoop = false;
    driveAppliedVolts = 0.0;
    turnAppliedVolts = 0.0;
  }

  @Override
  public void warmup() {
    driveSim.setInputVoltage(0.0);
    turnSim.setInputVoltage(0.0);
    driveSim.update(TARGET_SUBSTEP_DT_SEC);
    turnSim.update(TARGET_SUBSTEP_DT_SEC);
    driveSim.setState(0.0, 0.0);
    turnSim.setState(0.0, 0.0);
  }

  /**
   * Clamps commanded motor voltage against live battery rail voltage, stator current limit, and
   * supply current limit before driving the physics plant so sim acceleration reflects real limits.
   */
  public static double clampVoltageForCurrentLimits(
      double requestedVolts,
      double backEmfVolts,
      double windingResistanceOhms,
      double statorLimitAmps,
      double supplyLimitAmps,
      double railVolts) {
    double safeRail = MathUtil.clamp(railVolts, MIN_RAIL_VOLTS, 14.0);
    double v = MathUtil.clamp(requestedVolts, -safeRail, safeRail);
    double maxStatorDeltaV = Math.max(0.0, statorLimitAmps) * windingResistanceOhms;
    v = MathUtil.clamp(v, backEmfVolts - maxStatorDeltaV, backEmfVolts + maxStatorDeltaV);

    if (supplyLimitAmps > 0.0 && windingResistanceOhms > 1e-6) {
      double statorA = Math.abs(v - backEmfVolts) / windingResistanceOhms;
      double supplyA = statorA * (Math.abs(v) / safeRail);
      if (supplyA > supplyLimitAmps) {
        double absE = Math.abs(backEmfVolts);
        double maxPowerTerm = 4.0 * supplyLimitAmps * windingResistanceOhms * safeRail;
        if (Math.signum(v) == Math.signum(backEmfVolts) || absE < 1e-6) {
          double vMagMax = 0.5 * (absE + Math.sqrt(absE * absE + maxPowerTerm));
          v = Math.copySign(Math.min(Math.abs(v), Math.min(safeRail, vMagMax)), v);
        } else {
          double vMagMax = 0.5 * (-absE + Math.sqrt(absE * absE + maxPowerTerm));
          v = Math.copySign(Math.min(Math.abs(v), Math.min(safeRail, Math.max(0.0, vMagMax))), v);
        }
      }
    }
    return MathUtil.clamp(v, -safeRail, safeRail);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double railVolts = railVoltageSupplier.getAsDouble();
    if (!Double.isFinite(railVolts) || railVolts < MIN_RAIL_VOLTS) {
      railVolts = NOMINAL_RAIL_VOLTS;
    } else {
      railVolts = MathUtil.clamp(railVolts, MIN_RAIL_VOLTS, 14.0);
    }

    int substeps = Math.max(1, (int) Math.round(loopPeriodSec / TARGET_SUBSTEP_DT_SEC));
    double substepDt = loopPeriodSec / substeps;
    double fpgaNowSec = Timer.getFPGATimestamp();
    double nowSec =
        (!Double.isNaN(lastOdometryEndSec) && fpgaNowSec <= lastOdometryEndSec)
            ? lastOdometryEndSec + 1e-4
            : fpgaNowSec;
    double timestampSpanSec =
        Double.isNaN(lastOdometryEndSec)
            ? loopPeriodSec
            : MathUtil.clamp(nowSec - lastOdometryEndSec, 1e-4, loopPeriodSec);
    double timestampStepSec = timestampSpanSec / substeps;
    double[] timestamps = new double[substeps];
    double[] drivePositions = new double[substeps];
    Rotation2d[] turnPositions = new Rotation2d[substeps];

    double lastDriveClampedVolts = 0.0;
    double lastTurnClampedVolts = 0.0;
    double lastDriveStatorA = 0.0;
    double lastTurnStatorA = 0.0;

    for (int step = 0; step < substeps; step++) {
      if (driveClosedLoop) {
        double targetRps = driveTargetVelRadPerSec / (2.0 * Math.PI);
        double actualRps = driveSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        double velErrRps = targetRps - actualRps;
        double ff = config.driveKs() * Math.signum(targetRps) + targetRps * config.driveKv();
        driveAppliedVolts = MathUtil.clamp(ff + velErrRps * runtimeDriveKp, -railVolts, railVolts);
      }
      if (turnClosedLoop) {
        double angleErrRad =
            MathUtil.angleModulus(
                turnTargetAngle.getRadians() - turnSim.getAngularPositionRad());
        double angleErrRot = angleErrRad / (2.0 * Math.PI);
        double turnVelRps = turnSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        double ks = (Math.abs(angleErrRot) > 1e-4) ? STEER_KS_VOLTS * Math.signum(angleErrRot) : 0.0;
        turnAppliedVolts =
            MathUtil.clamp(
                angleErrRot * config.turnKp() - turnVelRps * config.turnKd() + ks,
                -railVolts,
                railVolts);
      }

      double driveBackEmfV =
          (driveSim.getAngularVelocityRadPerSec() * config.driveGearRatio())
              / driveMotor.KvRadPerSecPerVolt;
      lastDriveClampedVolts =
          clampVoltageForCurrentLimits(
              driveAppliedVolts,
              driveBackEmfV,
              driveMotor.rOhms,
              activeStatorLimitAmps,
              config.driveSupplyLimitAmps(),
              railVolts);
      lastDriveStatorA = Math.abs(lastDriveClampedVolts - driveBackEmfV) / driveMotor.rOhms;

      double turnBackEmfV =
          (turnSim.getAngularVelocityRadPerSec() * config.turnGearRatio())
              / turnMotor.KvRadPerSecPerVolt;
      lastTurnClampedVolts =
          clampVoltageForCurrentLimits(
              turnAppliedVolts,
              turnBackEmfV,
              turnMotor.rOhms,
              config.turnStatorLimitAmps(),
              config.turnSupplyLimitAmps(),
              railVolts);
      lastTurnStatorA = Math.abs(lastTurnClampedVolts - turnBackEmfV) / turnMotor.rOhms;

      driveSim.setInputVoltage(lastDriveClampedVolts);
      turnSim.setInputVoltage(lastTurnClampedVolts);
      driveSim.update(substepDt);
      turnSim.update(substepDt);

      timestamps[step] = nowSec - (substeps - 1 - step) * timestampStepSec;
      drivePositions[step] = driveSim.getAngularPositionRad();
      turnPositions[step] = new Rotation2d(turnSim.getAngularPositionRad());
    }
    lastOdometryEndSec = nowSec;

    lastOdometryTimestamps = timestamps;
    lastOdometryDrivePositionsRad = drivePositions;
    lastOdometryTurnPositions = turnPositions;

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = lastDriveClampedVolts;
    inputs.driveStatorCurrentAmps = Math.min(activeStatorLimitAmps, lastDriveStatorA);
    inputs.driveSupplyCurrentAmps =
        Math.min(
            config.driveSupplyLimitAmps(),
            inputs.driveStatorCurrentAmps * (Math.abs(lastDriveClampedVolts) / railVolts));

    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnAbsolutePosition = inputs.turnPosition;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = lastTurnClampedVolts;
    inputs.turnStatorCurrentAmps = Math.min(config.turnStatorLimitAmps(), lastTurnStatorA);
    inputs.turnSupplyCurrentAmps =
        Math.min(
            config.turnSupplyLimitAmps(),
            inputs.turnStatorCurrentAmps * (Math.abs(lastTurnClampedVolts) / railVolts));

    inputs.odometryTimestamps = timestamps;
    inputs.odometryDrivePositionsRad = drivePositions;
    inputs.odometryTurnPositions = turnPositions;
  }
}
