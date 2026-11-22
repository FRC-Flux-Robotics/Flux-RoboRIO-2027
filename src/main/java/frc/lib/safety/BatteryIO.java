package frc.lib.safety;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware abstraction port for the inline Playing With Fusion {@code ROB-70001} Battery Fuel Gauge
 * wired to {@code auxCanBus} ({@code "rio"} on roboRIO / {@code "can_s4"} on SystemCore).
 *
 * <p>Hardware access lives in {@link BatteryIOBfgCan}, simulation in {@link BatteryIOSim}, and
 * replayable battery name, filtered {@code R_int}, and health policy in {@link BatteryMonitor}.
 */
public interface BatteryIO {

  /** FRC 5-bit CAN Device Type for Miscellaneous / Power sensors ({@code 0x0A}). */
  int BFG_DEVICE_TYPE_ID = 10;

  /** WPILib HAL CAN Manufacturer ID for Playing With Fusion ({@code 0x0B}). */
  int PWF_MANUFACTURER_ID = 11;

  /**
   * 10-bit FRC CAN API ID for PWF {@code Power} ({@code 0x0A0B0000}): LE {@code [0..3] i32
   * current_mA, [4..5] u16 voltage_mV, [6..7] i16 dod_mAh}.
   */
  int API_ID_POWER = 0x000;

  /**
   * 10-bit FRC CAN API ID for PWF {@code BatteryHealth} ({@code 0x0A0B0100}): LE {@code [0..3] u32
   * battAge_s, [4..5] u16 capacity_mAh, [6..7] u16 numCycles}.
   */
  int API_ID_BATTERY_HEALTH = 0x004;

  /**
   * 10-bit FRC CAN API ID for PWF {@code StateOfCharge} ({@code 0x0A0B0700}): LE {@code [0..1] i16
   * soc_mAh, [2..3] i16 effectiveCap_mAh, [4] u8 chargeState, [5..7] i24 dod_Ws}.
   */
  int API_ID_STATE_OF_CHARGE = 0x01C;

  /** 10-bit FRC CAN API ID for PWF {@code Nickname1} ({@code 0x0A0B0F00}): ASCII chars [0..7]. */
  int API_ID_NICKNAME_1 = 0x03C;

  /** 10-bit FRC CAN API ID for PWF {@code Nickname2} ({@code 0x0A0B1000}): ASCII chars [8..10]. */
  int API_ID_NICKNAME_2 = 0x040;

  /** Nominal FRC 12V SLA battery capacity (mAh) used when deriving SoH % from capacity_mAh. */
  double DEFAULT_NOMINAL_CAPACITY_MAH = 18000.0;

  /** Default CAN node ID for a single inline battery sensor on the main battery lead. */
  int DEFAULT_BFG_DEVICE_ID = 0;

  /** Loggable inputs for the inline Playing With Fusion {@code ROB-70001} Battery Fuel Gauge. */
  class BatteryIOInputs implements LoggableInputs {
    public boolean connected = false;
    public String rawSerialOrCode = "";
    public double voltageVolts = 12.6;
    public double currentAmps = 0.0;
    public double stateOfChargePct = 100.0;
    public double stateOfHealthPct = 100.0;
    public double internalResistanceMilliOhms = Double.NaN;
    public double consumedChargeMah = 0.0;
    public double minVoltageVolts = 12.6;
    public double peakCurrentAmps = 0.0;
    public double temperatureCelsius = 25.0;
    public double dodWattHours = 0.0;
    public long batteryAgeSeconds = 0L;
    public int cycleCount = 0;
    public int chargeState = 0;
    public int framesReceived = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("RawSerialOrCode", rawSerialOrCode);
      table.put("VoltageVolts", voltageVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("StateOfChargePct", stateOfChargePct);
      table.put("StateOfHealthPct", stateOfHealthPct);
      table.put("InternalResistanceMilliOhms", internalResistanceMilliOhms);
      table.put("ConsumedChargeMah", consumedChargeMah);
      table.put("MinVoltageVolts", minVoltageVolts);
      table.put("PeakCurrentAmps", peakCurrentAmps);
      table.put("TemperatureCelsius", temperatureCelsius);
      table.put("DodWattHours", dodWattHours);
      table.put("BatteryAgeSeconds", batteryAgeSeconds);
      table.put("CycleCount", cycleCount);
      table.put("ChargeState", chargeState);
      table.put("FramesReceived", framesReceived);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      rawSerialOrCode = table.get("RawSerialOrCode", rawSerialOrCode);
      voltageVolts = table.get("VoltageVolts", voltageVolts);
      currentAmps = table.get("CurrentAmps", currentAmps);
      stateOfChargePct = table.get("StateOfChargePct", stateOfChargePct);
      stateOfHealthPct = table.get("StateOfHealthPct", stateOfHealthPct);
      internalResistanceMilliOhms =
          table.get("InternalResistanceMilliOhms", internalResistanceMilliOhms);
      consumedChargeMah = table.get("ConsumedChargeMah", consumedChargeMah);
      minVoltageVolts = table.get("MinVoltageVolts", minVoltageVolts);
      peakCurrentAmps = table.get("PeakCurrentAmps", peakCurrentAmps);
      temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
      dodWattHours = table.get("DodWattHours", dodWattHours);
      batteryAgeSeconds = table.get("BatteryAgeSeconds", batteryAgeSeconds);
      cycleCount = table.get("CycleCount", cycleCount);
      chargeState = table.get("ChargeState", chargeState);
      framesReceived = table.get("FramesReceived", framesReceived);
    }
  }

  /** Rejects high-rate CAN FD buses that cannot host the 1 Mbps CAN 2.0 BFG sensor. */
  static void validateAuxCanBus(String auxCanBus) {
    if (auxCanBus == null || auxCanBus.isBlank()) {
      throw new IllegalArgumentException("auxCanBus must not be blank");
    }
    if ("Drivetrain".equalsIgnoreCase(auxCanBus)
        || "Mech".equalsIgnoreCase(auxCanBus)
        || "CANdace".equalsIgnoreCase(auxCanBus)
        || "can_s0".equalsIgnoreCase(auxCanBus)
        || "can_s2".equalsIgnoreCase(auxCanBus)) {
      throw new IllegalArgumentException(
          "Playing With Fusion BFG (ROB-70001) is 1 Mbps CAN 2.0 and must be wired to auxCanBus"
              + " ('rio' on roboRIO or 'can_s4' on SystemCore), never high-rate CAN FD bus: "
              + auxCanBus);
    }
  }

  default void updateInputs(BatteryIOInputs inputs) {}

  default String getAuxCanBus() {
    return "rio";
  }

  default int getDeviceId() {
    return DEFAULT_BFG_DEVICE_ID;
  }
}
