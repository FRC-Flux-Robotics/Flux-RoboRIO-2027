package frc.lib.safety;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.nio.charset.StandardCharsets;
import java.util.function.DoubleSupplier;

/**
 * Hardware CAN adapter for the inline Playing With Fusion {@code ROB-70001} Battery Fuel Gauge
 * wired to {@code auxCanBus} ({@code "rio"} on roboRIO / {@code "can_s4"} on SystemCore).
 *
 * <p>Decodes little-endian 29-bit FRC CAN frames (Mfg ID {@code 11}, Device Type {@code 10}),
 * reports CAN initialization failures via {@link DriverStation#reportWarning}, and clears {@code
 * inputs.connected} whenever no valid frame arrives within {@link #CAN_DISCONNECT_TIMEOUT_SEC}.
 */
public class BatteryIOBfgCan implements BatteryIO, AutoCloseable {

  /** Duration without CAN frames after which {@code inputs.connected} resets to {@code false}. */
  public static final double CAN_DISCONNECT_TIMEOUT_SEC = 2.0;

  private final String auxCanBus;
  private final int deviceId;
  private final DoubleSupplier clockSecSupplier;
  private final CANData canData = new CANData();

  private CAN wpiCanHandle;
  private boolean canInitAttempted = false;
  private String nicknamePart1 = "";
  private String nicknamePart2 = "";
  private String rawSerialOrCode = "";
  private double voltageVolts = 12.6;
  private double currentAmps = 0.0;
  private double stateOfChargePct = 100.0;
  private double stateOfHealthPct = 100.0;
  private double consumedChargeMah = 0.0;
  private double minVoltageVolts = 12.6;
  private double peakCurrentAmps = 0.0;
  private double dodWattHours = 0.0;
  private long batteryAgeSeconds = 0L;
  private int cycleCount = 0;
  private int chargeState = 0;
  private int framesReceived = 0;
  private double lastFrameTimestampSec = Double.NEGATIVE_INFINITY;

  public BatteryIOBfgCan(String auxCanBus) {
    this(auxCanBus, DEFAULT_BFG_DEVICE_ID, Timer::getFPGATimestamp);
  }

  public BatteryIOBfgCan(String auxCanBus, int deviceId) {
    this(auxCanBus, deviceId, Timer::getFPGATimestamp);
  }

  public BatteryIOBfgCan(String auxCanBus, int deviceId, DoubleSupplier clockSecSupplier) {
    BatteryIO.validateAuxCanBus(auxCanBus);
    if (deviceId < 0 || deviceId > 63) {
      throw new IllegalArgumentException(
          "BFG 6-bit FRC CAN deviceId must be in [0, 63], got: " + deviceId);
    }
    this.auxCanBus = auxCanBus;
    this.deviceId = deviceId;
    this.clockSecSupplier =
        (clockSecSupplier != null) ? clockSecSupplier : Timer::getFPGATimestamp;
  }

  /**
   * Packs a 29-bit FRC Extended CAN arbitration ID: {@code [deviceType:5 | manufacturer:8 |
   * apiId:10 | deviceId:6]}.
   */
  public static int buildArbitrationId(
      int deviceType, int manufacturer, int apiId, int deviceId) {
    return ((deviceType & 0x1F) << 24)
        | ((manufacturer & 0xFF) << 16)
        | ((apiId & 0x3FF) << 6)
        | (deviceId & 0x3F);
  }

  /** Packs a 29-bit PWF {@code ROB-70001} arbitration ID for the given 10-bit {@code apiId}. */
  public static int buildBfgArbitrationId(int apiId, int deviceId) {
    return buildArbitrationId(BFG_DEVICE_TYPE_ID, PWF_MANUFACTURER_ID, apiId, deviceId);
  }

  public static int extractDeviceType(int arbitrationId) {
    return (arbitrationId >>> 24) & 0x1F;
  }

  public static int extractManufacturer(int arbitrationId) {
    return (arbitrationId >>> 16) & 0xFF;
  }

  public static int extractApiId(int arbitrationId) {
    return (arbitrationId >>> 6) & 0x3FF;
  }

  public static int extractDeviceId(int arbitrationId) {
    return arbitrationId & 0x3F;
  }

  /** Encodes an 8-byte ASCII fragment (chars 0..7) for {@link #API_ID_NICKNAME_1}. */
  public static byte[] encodeNickname1Frame(String nickname) {
    return encodeAsciiSlice(nickname, 0, 8);
  }

  /** Encodes an 8-byte ASCII tail fragment (chars 8..10) for {@link #API_ID_NICKNAME_2}. */
  public static byte[] encodeNickname2Frame(String nickname) {
    return encodeAsciiSlice(nickname, 8, 3);
  }

  private static byte[] encodeAsciiSlice(String text, int offset, int maxLen) {
    byte[] out = new byte[8];
    byte[] raw = (text == null ? "" : text.trim()).getBytes(StandardCharsets.US_ASCII);
    if (offset < raw.length) {
      int copyLen = Math.min(maxLen, raw.length - offset);
      System.arraycopy(raw, offset, out, 0, copyLen);
    }
    return out;
  }

  /** Encodes an 8-byte little-endian PWF {@code Power} frame ({@link #API_ID_POWER}). */
  public static byte[] encodePowerFrame(
      double currentAmps, double voltageVolts, double dodMah) {
    int currentMa = (int) Math.round(currentAmps * 1000.0);
    int voltageMv = Math.max(0, Math.min(65535, (int) Math.round(voltageVolts * 1000.0)));
    int dodSignedMah = Math.max(-32768, Math.min(32767, (int) Math.round(dodMah)));
    return new byte[] {
      (byte) (currentMa & 0xFF),
      (byte) ((currentMa >>> 8) & 0xFF),
      (byte) ((currentMa >>> 16) & 0xFF),
      (byte) ((currentMa >>> 24) & 0xFF),
      (byte) (voltageMv & 0xFF),
      (byte) ((voltageMv >>> 8) & 0xFF),
      (byte) (dodSignedMah & 0xFF),
      (byte) ((dodSignedMah >>> 8) & 0xFF)
    };
  }

  /** Encodes an 8-byte little-endian PWF {@code BatteryHealth} frame. */
  public static byte[] encodeBatteryHealthFrame(
      long batteryAgeSeconds, double capacityMah, int numCycles) {
    long ageSec = Math.max(0L, batteryAgeSeconds);
    int capMah = Math.max(0, Math.min(65535, (int) Math.round(capacityMah)));
    int cycles = Math.max(0, Math.min(65535, numCycles));
    return new byte[] {
      (byte) (ageSec & 0xFF),
      (byte) ((ageSec >>> 8) & 0xFF),
      (byte) ((ageSec >>> 16) & 0xFF),
      (byte) ((ageSec >>> 24) & 0xFF),
      (byte) (capMah & 0xFF),
      (byte) ((capMah >>> 8) & 0xFF),
      (byte) (cycles & 0xFF),
      (byte) ((cycles >>> 8) & 0xFF)
    };
  }

  /** Encodes an 8-byte little-endian PWF {@code StateOfCharge} frame. */
  public static byte[] encodeStateOfChargeFrame(
      double socMah, double effectiveCapMah, int chargeState, double dodWattHours) {
    int socSignedMah = Math.max(-32768, Math.min(32767, (int) Math.round(socMah)));
    int effCapSignedMah = Math.max(-32768, Math.min(32767, (int) Math.round(effectiveCapMah)));
    int stateU8 = chargeState & 0xFF;
    int dodWs = Math.max(-8_388_608, Math.min(8_388_607, (int) Math.round(dodWattHours * 3600.0)));
    return new byte[] {
      (byte) (socSignedMah & 0xFF),
      (byte) ((socSignedMah >>> 8) & 0xFF),
      (byte) (effCapSignedMah & 0xFF),
      (byte) ((effCapSignedMah >>> 8) & 0xFF),
      (byte) stateU8,
      (byte) (dodWs & 0xFF),
      (byte) ((dodWs >>> 8) & 0xFF),
      (byte) ((dodWs >>> 16) & 0xFF)
    };
  }

  /** Decodes a raw 29-bit FRC CAN frame matching this Playing With Fusion {@code ROB-70001}. */
  public boolean processCanFrame(int arbitrationId, byte[] data, int length) {
    if (data == null || length <= 0 || length > data.length) {
      return false;
    }
    int devType = extractDeviceType(arbitrationId);
    int mfg = extractManufacturer(arbitrationId);
    int devId = extractDeviceId(arbitrationId);
    if (devType != BFG_DEVICE_TYPE_ID || mfg != PWF_MANUFACTURER_ID || devId != deviceId) {
      return false;
    }
    return processApiPayload(extractApiId(arbitrationId), data, length);
  }

  /** Decodes a Playing With Fusion {@code ROB-70001} payload for the given 10-bit {@code apiId}. */
  public boolean processApiPayload(int apiId, byte[] data, int length) {
    if (data == null || length <= 0 || length > data.length) {
      return false;
    }
    switch (apiId) {
      case API_ID_POWER -> {
        if (length < 8) {
          return false;
        }
        int currentMa = readLeI32(data, 0);
        int voltageMv = readLeU16(data, 4);
        int dodMah = readLeI16(data, 6);

        double v = voltageMv / 1000.0;
        double i = currentMa / 1000.0;
        recordFrameTimestamp();
        voltageVolts = v;
        currentAmps = i;
        consumedChargeMah = dodMah;
        if (v < minVoltageVolts || framesReceived == 1) {
          minVoltageVolts = v;
        }
        if (Math.abs(i) > peakCurrentAmps) {
          peakCurrentAmps = Math.abs(i);
        }
        return true;
      }
      case API_ID_BATTERY_HEALTH -> {
        if (length < 8) {
          return false;
        }
        long battAgeS = readLeU32(data, 0);
        int capacityMah = readLeU16(data, 4);
        int numCycles = readLeU16(data, 6);

        recordFrameTimestamp();
        batteryAgeSeconds = battAgeS;
        cycleCount = numCycles;
        stateOfHealthPct =
            Math.max(0.0, Math.min(100.0, (100.0 * capacityMah) / DEFAULT_NOMINAL_CAPACITY_MAH));
        return true;
      }
      case API_ID_STATE_OF_CHARGE -> {
        if (length < 8) {
          return false;
        }
        int socMah = readLeI16(data, 0);
        int effCapMah = readLeI16(data, 2);
        int state = data[4] & 0xFF;
        int dodWs = readLeI24(data, 5);

        recordFrameTimestamp();
        chargeState = state;
        dodWattHours = dodWs / 3600.0;
        if (effCapMah > 0) {
          stateOfChargePct = Math.max(0.0, Math.min(100.0, (100.0 * socMah) / effCapMah));
        }
        return true;
      }
      case API_ID_NICKNAME_1 -> {
        nicknamePart1 = decodeAsciiFragment(data, Math.min(8, length));
        rawSerialOrCode = (nicknamePart1 + nicknamePart2).trim();
        recordFrameTimestamp();
        return true;
      }
      case API_ID_NICKNAME_2 -> {
        nicknamePart2 = decodeAsciiFragment(data, Math.min(3, length));
        rawSerialOrCode = (nicknamePart1 + nicknamePart2).trim();
        recordFrameTimestamp();
        return true;
      }
      default -> {
        return false;
      }
    }
  }

  private void recordFrameTimestamp() {
    framesReceived++;
    lastFrameTimestampSec = clockSecSupplier.getAsDouble();
  }

  private void pollHardwareCan() {
    if (!RobotBase.isReal()) {
      return;
    }
    if (!canInitAttempted) {
      canInitAttempted = true;
      try {
        wpiCanHandle = new CAN(deviceId, PWF_MANUFACTURER_ID, BFG_DEVICE_TYPE_ID);
      } catch (Exception e) {
        wpiCanHandle = null;
        DriverStation.reportWarning(
            "BatteryIOBfgCan: failed to initialize CAN handle on auxCanBus='"
                + auxCanBus
                + "' deviceId="
                + deviceId
                + ": "
                + e.getMessage(),
            false);
      }
    }
    if (wpiCanHandle == null) {
      return;
    }
    if (wpiCanHandle.readPacketNew(API_ID_NICKNAME_1, canData)) {
      processApiPayload(API_ID_NICKNAME_1, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_NICKNAME_2, canData)) {
      processApiPayload(API_ID_NICKNAME_2, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_POWER, canData)) {
      processApiPayload(API_ID_POWER, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_BATTERY_HEALTH, canData)) {
      processApiPayload(API_ID_BATTERY_HEALTH, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_STATE_OF_CHARGE, canData)) {
      processApiPayload(API_ID_STATE_OF_CHARGE, canData.data, canData.length);
    }
  }

  @Override
  public void updateInputs(BatteryIOInputs inputs) {
    pollHardwareCan();
    double nowSec = clockSecSupplier.getAsDouble();
    boolean fresh =
        Double.isFinite(lastFrameTimestampSec)
            && (nowSec - lastFrameTimestampSec) >= -0.05
            && (nowSec - lastFrameTimestampSec) <= CAN_DISCONNECT_TIMEOUT_SEC;

    inputs.connected = fresh;
    inputs.rawSerialOrCode = rawSerialOrCode;
    inputs.voltageVolts = voltageVolts;
    inputs.currentAmps = currentAmps;
    inputs.stateOfChargePct = stateOfChargePct;
    inputs.stateOfHealthPct = stateOfHealthPct;
    // ROB-70001 CAN frames carry voltage/current only; BatteryMonitor computes filtered R_int.
    inputs.internalResistanceMilliOhms = Double.NaN;
    inputs.consumedChargeMah = consumedChargeMah;
    inputs.minVoltageVolts = minVoltageVolts;
    inputs.peakCurrentAmps = peakCurrentAmps;
    inputs.temperatureCelsius = 25.0;
    inputs.dodWattHours = dodWattHours;
    inputs.batteryAgeSeconds = batteryAgeSeconds;
    inputs.cycleCount = cycleCount;
    inputs.chargeState = chargeState;
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

  @Override
  public void close() {
    if (wpiCanHandle != null) {
      wpiCanHandle.close();
      wpiCanHandle = null;
    }
  }

  private static String decodeAsciiFragment(byte[] data, int maxLen) {
    int nonNullLen = 0;
    for (int i = 0; i < maxLen; i++) {
      int b = data[i] & 0xFF;
      if (b == 0) {
        break;
      }
      if (b >= 0x20 && b <= 0x7E) {
        nonNullLen++;
      } else {
        break;
      }
    }
    return (nonNullLen > 0)
        ? new String(data, 0, nonNullLen, StandardCharsets.US_ASCII)
        : "";
  }

  private static int readLeU16(byte[] data, int off) {
    return (data[off] & 0xFF) | ((data[off + 1] & 0xFF) << 8);
  }

  private static int readLeI16(byte[] data, int off) {
    return (short) readLeU16(data, off);
  }

  private static int readLeI24(byte[] data, int off) {
    int raw =
        (data[off] & 0xFF) | ((data[off + 1] & 0xFF) << 8) | ((data[off + 2] & 0xFF) << 16);
    if ((raw & 0x800000) != 0) {
      raw |= 0xFF000000;
    }
    return raw;
  }

  private static int readLeI32(byte[] data, int off) {
    return (data[off] & 0xFF)
        | ((data[off + 1] & 0xFF) << 8)
        | ((data[off + 2] & 0xFF) << 16)
        | ((data[off + 3] & 0xFF) << 24);
  }

  private static long readLeU32(byte[] data, int off) {
    return readLeI32(data, off) & 0xFFFF_FFFFL;
  }
}
