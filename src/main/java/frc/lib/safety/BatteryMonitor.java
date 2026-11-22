package frc.lib.safety;

import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.BatterySpec;
import frc.lib.safety.BatteryIO.BatteryIOInputs;
import java.util.Arrays;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Pure AdvantageKit battery telemetry and health monitor consuming {@link BatteryIO} inputs.
 *
 * <p>Resolves battery identity from {@code batteries.csv}, computes noise-resistant median-filtered
 * internal resistance ({@code R_int}) strictly from logged {@link BatteryIOInputs} so {@code
 * Mode.REPLAY} reproduces identical identity and health decisions, and publishes {@code Battery/*}
 * outputs.
 */
public class BatteryMonitor {

  /** Minimum current step (Amps) required to estimate dynamic {@code R_int = -dV / dI}. */
  public static final double MIN_RINT_STEP_CURRENT_AMPS = 25.0;

  /** Number of confirming transient steps required before filtered {@code R_int} updates. */
  public static final int RINT_MEDIAN_WINDOW = 3;

  private final BatteryIO io;
  private final FluxFig fluxFig;
  private final BatteryIOInputs inputs = new BatteryIOInputs();

  private String lastSerialLookup = null;
  private String resolvedBatteryName = "UNKNOWN_BATTERY";
  private double maxAllowedRintMilliOhms = 24.0;
  private double nominalRintMilliOhms = 16.0;
  private double filteredRintMilliOhms = 16.0;
  private boolean rIntMeasuredFromSteps = false;

  private double prevSampleVoltageV = Double.NaN;
  private double prevSampleCurrentA = Double.NaN;
  private int prevFramesReceived = -1;
  private final double[] rIntStepWindow = new double[RINT_MEDIAN_WINDOW];
  private int rIntStepCount = 0;

  public BatteryMonitor(BatteryIO io, FluxFig fluxFig) {
    this.io = io;
    this.fluxFig = fluxFig;
    this.nominalRintMilliOhms = computeNominalCsvRintMilliOhms(fluxFig);
    this.filteredRintMilliOhms = this.nominalRintMilliOhms;
  }

  private static double computeNominalCsvRintMilliOhms(FluxFig fig) {
    if (fig == null) {
      return 16.0;
    }
    double sum = 0.0;
    int count = 0;
    for (Map<String, String> row : fig.getTable("batteries")) {
      String raw = row.get("nominalRintMilliOhms");
      if (raw != null && !raw.isBlank()) {
        try {
          double val = Double.parseDouble(raw.trim());
          if (Double.isFinite(val) && val > 0.0) {
            sum += val;
            count++;
          }
        } catch (NumberFormatException ignored) {
        }
      }
    }
    if (count > 0) {
      return sum / count;
    }
    for (BatterySpec spec : fig.getBatteries().values()) {
      if (Double.isFinite(spec.nominalRintMilliOhms()) && spec.nominalRintMilliOhms() > 0.0) {
        sum += spec.nominalRintMilliOhms();
        count++;
      }
    }
    return (count > 0) ? (sum / count) : 16.0;
  }

  /** Polls {@link BatteryIO}, processes replayable inputs, and updates battery health policy. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Battery", inputs);
    updateDerivedStateFromInputs();

    Logger.recordOutput("Battery/AuxCanBus", io.getAuxCanBus());
    Logger.recordOutput("Battery/DeviceId", io.getDeviceId());
    Logger.recordOutput("Battery/Name", resolvedBatteryName);
    Logger.recordOutput("Battery/FilteredRintMilliOhms", filteredRintMilliOhms);
    Logger.recordOutput("Battery/Healthy", isHealthy());
  }

  private void updateDerivedStateFromInputs() {
    String serial = (inputs.rawSerialOrCode != null) ? inputs.rawSerialOrCode.trim() : "";
    if (!serial.equals(lastSerialLookup)) {
      lastSerialLookup = serial;
      resolveBatteryFromFluxFig(serial);
    }

    if (Double.isFinite(inputs.internalResistanceMilliOhms)
        && inputs.internalResistanceMilliOhms >= 5.0
        && inputs.internalResistanceMilliOhms <= 60.0) {
      filteredRintMilliOhms = inputs.internalResistanceMilliOhms;
      rIntMeasuredFromSteps = true;
    } else if (inputs.connected && inputs.framesReceived != prevFramesReceived) {
      updateStepFilteredRint(inputs.voltageVolts, inputs.currentAmps);
    }
    prevFramesReceived = inputs.framesReceived;
  }

  private void updateStepFilteredRint(double voltageV, double currentA) {
    if (!Double.isFinite(voltageV) || !Double.isFinite(currentA)) {
      return;
    }
    if (!Double.isNaN(prevSampleVoltageV)
        && Math.abs(currentA - prevSampleCurrentA) >= MIN_RINT_STEP_CURRENT_AMPS) {
      double dV = prevSampleVoltageV - voltageV;
      double dI = currentA - prevSampleCurrentA;
      double estMohm = (dV / dI) * 1000.0;
      if (Double.isFinite(estMohm) && estMohm >= 5.0 && estMohm <= 60.0) {
        rIntStepWindow[rIntStepCount % RINT_MEDIAN_WINDOW] = estMohm;
        rIntStepCount++;
        // Require a full 3-sample median window so a single CAN voltage glitch never latches throttle.
        if (rIntStepCount >= RINT_MEDIAN_WINDOW) {
          double[] sorted = Arrays.copyOf(rIntStepWindow, RINT_MEDIAN_WINDOW);
          Arrays.sort(sorted);
          double median = sorted[RINT_MEDIAN_WINDOW / 2];
          filteredRintMilliOhms =
              rIntMeasuredFromSteps ? (0.70 * filteredRintMilliOhms + 0.30 * median) : median;
          rIntMeasuredFromSteps = true;
        }
      }
    }
    prevSampleVoltageV = voltageV;
    prevSampleCurrentA = currentA;
  }

  private void resolveBatteryFromFluxFig(String serialOrQrCode) {
    if (fluxFig != null && serialOrQrCode != null && !serialOrQrCode.isBlank()) {
      String trimmed = serialOrQrCode.trim();
      String prefixed = "FLUX-BAT-" + trimmed;
      for (Map<String, String> row : fluxFig.getTable("batteries")) {
        String name = row.getOrDefault("batteryName", "");
        String qr = row.getOrDefault("qrCode", "");
        if (name.equalsIgnoreCase(trimmed)
            || qr.equalsIgnoreCase(trimmed)
            || qr.equalsIgnoreCase(prefixed)) {
          resolvedBatteryName = name;
          maxAllowedRintMilliOhms =
              Double.parseDouble(row.getOrDefault("maxAllowedRintMilliOhms", "24.0"));
          double rowNominalRint =
              Double.parseDouble(row.getOrDefault("nominalRintMilliOhms", "16.0"));
          nominalRintMilliOhms = rowNominalRint;
          if (!rIntMeasuredFromSteps) {
            filteredRintMilliOhms = rowNominalRint;
          }
          return;
        }
      }
      for (BatterySpec spec : fluxFig.getBatteries().values()) {
        if (spec.batteryName().equalsIgnoreCase(trimmed)
            || spec.qrCode().equalsIgnoreCase(trimmed)
            || spec.qrCode().equalsIgnoreCase(prefixed)) {
          resolvedBatteryName = spec.batteryName();
          maxAllowedRintMilliOhms = spec.maxAllowedRintMilliOhms();
          nominalRintMilliOhms = spec.nominalRintMilliOhms();
          if (!rIntMeasuredFromSteps) {
            filteredRintMilliOhms = spec.nominalRintMilliOhms();
          }
          return;
        }
      }
    }
    resolvedBatteryName =
        (serialOrQrCode == null || serialOrQrCode.isBlank())
            ? "UNKNOWN_BATTERY"
            : serialOrQrCode.trim();
  }

  public boolean isHealthy() {
    return inputs.connected
        && inputs.stateOfChargePct >= 75.0
        && filteredRintMilliOhms <= maxAllowedRintMilliOhms;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getResolvedBatteryName() {
    return resolvedBatteryName;
  }

  public String getAuxCanBus() {
    return io.getAuxCanBus();
  }

  public int getDeviceId() {
    return io.getDeviceId();
  }

  public double getVoltageVolts() {
    return inputs.voltageVolts;
  }

  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  public double getStateOfChargePct() {
    return inputs.stateOfChargePct;
  }

  public double getStateOfHealthPct() {
    return inputs.stateOfHealthPct;
  }

  public double getInternalResistanceMilliOhms() {
    return filteredRintMilliOhms;
  }

  public double getNominalRintMilliOhms() {
    return nominalRintMilliOhms;
  }

  public double getConsumedChargeMah() {
    return inputs.consumedChargeMah;
  }

  public double getMinVoltageVolts() {
    return inputs.minVoltageVolts;
  }

  public double getPeakCurrentAmps() {
    return inputs.peakCurrentAmps;
  }

  public int getCycleCount() {
    return inputs.cycleCount;
  }

  public long getBatteryAgeSeconds() {
    return inputs.batteryAgeSeconds;
  }

  public int getFramesReceived() {
    return inputs.framesReceived;
  }

  public BatteryIO getIo() {
    return io;
  }
}
