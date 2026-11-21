package frc.lib.mechanism;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;
import java.util.Locale;

/**
 * Real CTRE TalonFX implementation of {@link MechanismIO} (supports single motor or leader +
 * follower) configured directly from a {@link MechanismSpec}. Enforces <= 60A Stator and <= 40A
 * Supply limits, supports {@link NeutralModeValue#Coast} on flywheel mechanisms ({@code Shooter}),
 * and registers 50 Hz status signals with {@link PhoenixSignals}.
 */
public class MechanismIOTalonFX implements MechanismIO, AutoCloseable {
  public static final double PEAK_FORWARD_VOLTAGE = 11.0;
  public static final double PEAK_REVERSE_VOLTAGE = -11.0;

  private final String mechanismName;
  private final TalonFX motor;
  private final TalonFX follower;
  private final GravityTypeValue gravityType;
  private final NeutralModeValue neutralMode;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VoltageOut homingVoltageRequest =
      new VoltageOut(0).withIgnoreSoftwareLimits(true);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final Slot0Configs slot0Configs = new Slot0Configs();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final int signalGroup;

  public MechanismIOTalonFX(MechanismSpec spec) {
    this(spec, spec.followerMotorId(), spec.invertFollower(), defaultNeutralModeFor(spec));
  }

  public MechanismIOTalonFX(MechanismSpec spec, int followerMotorId, boolean opposeLeader) {
    this(spec, followerMotorId, opposeLeader, defaultNeutralModeFor(spec));
  }

  public MechanismIOTalonFX(
      MechanismSpec spec,
      int followerMotorId,
      boolean opposeLeader,
      NeutralModeValue neutralMode) {
    if (spec.statorLimitAmps() > 60.0 || spec.slipCurrentAmps() > 60.0) {
      throw new IllegalArgumentException(
          "Refusing stator/slip current > 60A on "
              + spec.mechanism()
              + " (stator="
              + spec.statorLimitAmps()
              + "A, slip="
              + spec.slipCurrentAmps()
              + "A) to prevent brownouts.");
    }
    if (spec.supplyLimitAmps() > 40.0) {
      throw new IllegalArgumentException(
          "Refusing supply current > 40A on "
              + spec.mechanism()
              + " (supply="
              + spec.supplyLimitAmps()
              + "A) to prevent breaker trips and brownouts.");
    }

    this.mechanismName = spec.mechanism();
    CANBus bus = new CANBus(spec.canBus());
    motor = new TalonFX(spec.motorId(), bus);
    follower = followerMotorId >= 0 ? new TalonFX(followerMotorId, bus) : null;
    this.neutralMode = (neutralMode != null) ? neutralMode : NeutralModeValue.Brake;

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = this.neutralMode;
    cfg.MotorOutput.Inverted =
        spec.motorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    cfg.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = Math.min(spec.statorLimitAmps(), spec.slipCurrentAmps());
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = spec.supplyLimitAmps();
    cfg.Feedback.SensorToMechanismRatio = spec.gearRatio() > 0.0 ? spec.gearRatio() : 1.0;

    cfg.Slot0.kP = spec.kP();
    cfg.Slot0.kI = spec.kI();
    cfg.Slot0.kD = spec.kD();
    cfg.Slot0.kS = spec.kS();
    cfg.Slot0.kV = spec.kV();
    cfg.Slot0.kA = spec.kA();
    cfg.Slot0.kG = spec.kG();
    if ("ARM_COSINE".equalsIgnoreCase(spec.gravityType())) {
      gravityType = GravityTypeValue.Arm_Cosine;
    } else {
      gravityType = GravityTypeValue.Elevator_Static;
    }
    cfg.Slot0.GravityType = gravityType;
    slot0Configs.kP = cfg.Slot0.kP;
    slot0Configs.kI = cfg.Slot0.kI;
    slot0Configs.kD = cfg.Slot0.kD;
    slot0Configs.kS = cfg.Slot0.kS;
    slot0Configs.kV = cfg.Slot0.kV;
    slot0Configs.kA = cfg.Slot0.kA;
    slot0Configs.kG = cfg.Slot0.kG;
    slot0Configs.GravityType = gravityType;

    if (spec.minPosRot() > -9999.0 || spec.maxPosRot() < 9999.0) {
      cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = spec.minPosRot();
      cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = spec.maxPosRot();
    }

    PhoenixUtil.tryUntilOk(
        "Mech-" + spec.mechanism() + "-Leader",
        () -> motor.getConfigurator().apply(cfg, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
        5);
    if (follower != null) {
      PhoenixUtil.tryUntilOk(
          "Mech-" + spec.mechanism() + "-Follower",
          () ->
              follower.getConfigurator().apply(cfg, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
          5);
      follower.setControl(
          new Follower(
              spec.motorId(),
              opposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    position = motor.getPosition();
    velocity = motor.getVelocity();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    voltage = motor.getMotorVoltage();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, statorCurrent, supplyCurrent, voltage, temp);

    if (follower != null) {
      followerStatorCurrent = follower.getStatorCurrent();
      followerSupplyCurrent = follower.getSupplyCurrent();
      BaseStatusSignal.setUpdateFrequencyForAll(
          50.0, followerStatorCurrent, followerSupplyCurrent);
      PhoenixUtil.tryUntilOk(
          "Mech-" + spec.mechanism() + "-FollowerLeaderFreq",
          () ->
              BaseStatusSignal.setUpdateFrequencyForAll(
                  200.0,
                  motor.getDutyCycle(),
                  voltage,
                  motor.getTorqueCurrent()),
          3);
      signalGroup =
          PhoenixSignals.register(
              spec.canBus(),
              position,
              velocity,
              statorCurrent,
              supplyCurrent,
              voltage,
              temp,
              followerStatorCurrent,
              followerSupplyCurrent);
      PhoenixUtil.tryUntilOk(
          "Mech-" + spec.mechanism() + "-OptimizeBus",
          () -> ParentDevice.optimizeBusUtilizationForAll(motor, follower),
          3);
    } else {
      followerStatorCurrent = null;
      followerSupplyCurrent = null;
      signalGroup =
          PhoenixSignals.register(
              spec.canBus(),
              position,
              velocity,
              statorCurrent,
              supplyCurrent,
              voltage,
              temp);
      PhoenixUtil.tryUntilOk(
          "Mech-" + spec.mechanism() + "-OptimizeBus",
          () -> ParentDevice.optimizeBusUtilizationForAll(motor),
          3);
    }
  }

  /** Returns Coast for flywheel/shooter mechanisms so high-inertia wheels spin down safely. */
  public static NeutralModeValue defaultNeutralModeFor(MechanismSpec spec) {
    if (spec != null && spec.mechanism() != null) {
      String lower = spec.mechanism().toLowerCase(Locale.ROOT);
      if (lower.contains("shooter") || lower.contains("flywheel")) {
        return NeutralModeValue.Coast;
      }
    }
    return NeutralModeValue.Brake;
  }

  public NeutralModeValue getNeutralMode() {
    return neutralMode;
  }

  /** Reads back the active TalonFXConfiguration registers from hardware/sim. */
  public TalonFXConfiguration readAppliedConfig() {
    TalonFXConfiguration readback = new TalonFXConfiguration();
    com.ctre.phoenix6.StatusCode status =
        PhoenixUtil.tryUntilOk(
            "Mech-ReadCfg",
            () ->
                motor
                    .getConfigurator()
                    .refresh(readback, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
            5);
    if (!status.isOK()) {
      throw new IllegalStateException("Failed to read back TalonFX config: " + status);
    }
    return readback;
  }

  /** Reads back the active follower TalonFXConfiguration registers from hardware/sim. */
  public TalonFXConfiguration readAppliedFollowerConfig() {
    if (follower == null) {
      throw new IllegalStateException("Mechanism " + mechanismName + " has no follower motor");
    }
    TalonFXConfiguration readback = new TalonFXConfiguration();
    com.ctre.phoenix6.StatusCode status =
        PhoenixUtil.tryUntilOk(
            "Mech-ReadFollowerCfg",
            () ->
                follower
                    .getConfigurator()
                    .refresh(readback, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
            5);
    if (!status.isOK()) {
      throw new IllegalStateException("Failed to read back follower TalonFX config: " + status);
    }
    return readback;
  }

  public TalonFX getMotor() {
    return motor;
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setVelocity(double velocityRPS) {
    motor.setControl(velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setHomingVoltage(double volts) {
    motor.setControl(homingVoltageRequest.withOutput(volts));
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kG) {
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kG = kG;
    slot0Configs.GravityType = gravityType;
    motor.getConfigurator().apply(slot0Configs, 0.0);
    if (follower != null) {
      follower.getConfigurator().apply(slot0Configs, 0.0);
    }
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public boolean resetPosition() {
    com.ctre.phoenix6.StatusCode status =
        PhoenixUtil.tryUntilOk(
            "Mech-" + mechanismName + "-ResetPos",
            () -> motor.setPosition(0.0, PhoenixUtil.DEFAULT_CAN_CALL_TIMEOUT_SECONDS),
            3);
    return status.isOK();
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRPS = velocity.getValueAsDouble();
    double leaderStator = statorCurrent.getValueAsDouble();
    double followerStator =
        followerStatorCurrent != null ? followerStatorCurrent.getValueAsDouble() : 0.0;
    inputs.leaderStatorCurrentA = leaderStator;
    inputs.followerStatorCurrentA = followerStator;
    inputs.statorCurrentA = leaderStator + followerStator;
    inputs.supplyCurrentA =
        supplyCurrent.getValueAsDouble()
            + (followerSupplyCurrent != null ? followerSupplyCurrent.getValueAsDouble() : 0.0);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.motorConnected = PhoenixSignals.isOK(signalGroup);
  }

  @Override
  public void close() {
    motor.close();
    if (follower != null) {
      follower.close();
    }
  }
}
