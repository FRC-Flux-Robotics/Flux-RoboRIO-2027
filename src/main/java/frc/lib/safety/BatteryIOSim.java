package frc.lib.safety;

import frc.lib.util.VirtualBatterySim;

/**
 * Simulation implementation of {@link BatteryIO} backed by {@link VirtualBatterySim} and explicit
 * test injection hooks (keeping test backdoors out of hardware {@link BatteryIOBfgCan}).
 */
public class BatteryIOSim implements BatteryIO {
  private final String auxCanBus;
  private final int deviceId;
  private final VirtualBatterySim batterySim;

  private boolean connected = false;
  private String rawSerialOrCode = "";
  private double voltageVolts = 12.6;
  private double currentAmps = 0.0;
  private double stateOfChargePct = 100.0;
  private double stateOfHealthPct = 100.0;
  private double explicitRintMilliOhms = Double.NaN;
  private double minVoltageVolts = 12.6;
  private double peakCurrentAmps = 0.0;
  private int framesReceived = 0;

  public BatteryIOSim(String auxCanBus, VirtualBatterySim batterySim) {
    this(auxCanBus, DEFAULT_BFG_DEVICE_ID, batterySim);
  }

  public BatteryIOSim(String auxCanBus, int deviceId, VirtualBatterySim batterySim) {
    BatteryIO.validateAuxCanBus(auxCanBus);
    this.auxCanBus = auxCanBus;
    this.deviceId = deviceId;
    this.batterySim = batterySim;
  }

  /** Injects a simulated BFG reading for desktop sim or deterministic unit testing. */
  public void simulateReading(
      String serialOrQrCode,
      double voltageVolts,
      double socPct,
      double sohPct,
      double rIntMilliOhms) {
    this.connected = true;
    this.rawSerialOrCode = (serialOrQrCode != null) ? serialOrQrCode : "";
    this.voltageVolts = voltageVolts;
    this.stateOfChargePct = socPct;
    this.stateOfHealthPct = sohPct;
    this.explicitRintMilliOhms = rIntMilliOhms;
    this.minVoltageVolts = Math.min(this.minVoltageVolts, voltageVolts);
    this.framesReceived++;
  }

  public void setConnected(boolean connected) {
    this.connected = connected;
  }

  public void setCurrentAmps(double currentAmps) {
    this.currentAmps = currentAmps;
    this.peakCurrentAmps = Math.max(this.peakCurrentAmps, Math.abs(currentAmps));
  }

  @Override
  public void updateInputs(BatteryIOInputs inputs) {
    double v = (batterySim != null && !connected) ? batterySim.terminalVoltageV() : voltageVolts;
    double rInt =
        Double.isFinite(explicitRintMilliOhms)
            ? explicitRintMilliOhms
            : (batterySim != null ? batterySim.estimatedRintMilliOhms() : Double.NaN);
    if (batterySim != null && connected && !Double.isFinite(explicitRintMilliOhms)) {
      v = batterySim.terminalVoltageV();
    }
    minVoltageVolts = Math.min(minVoltageVolts, v);

    inputs.connected = connected;
    inputs.rawSerialOrCode = rawSerialOrCode;
    inputs.voltageVolts = v;
    inputs.currentAmps = currentAmps;
    inputs.stateOfChargePct = stateOfChargePct;
    inputs.stateOfHealthPct = stateOfHealthPct;
    inputs.internalResistanceMilliOhms = rInt;
    inputs.minVoltageVolts = minVoltageVolts;
    inputs.peakCurrentAmps = peakCurrentAmps;
    inputs.temperatureCelsius = 25.0;
    inputs.framesReceived = framesReceived;
  }

  @Override
  public String getAuxCanBus() {
    return auxCanBus;
  }

  @Override
  public int getDeviceId() {
    return deviceId;
  }
}
