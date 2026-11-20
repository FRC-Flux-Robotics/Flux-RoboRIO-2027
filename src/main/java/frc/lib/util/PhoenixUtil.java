package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Utility methods for CTRE Phoenix 6 hardware configuration with a bounded boot wall-clock budget
 * so offline/faulted CAN buses never stall robotInit() across multiple devices.
 */
public final class PhoenixUtil {
  /** Default total wall-clock budget (seconds) shared across boot-time CTRE config retries. */
  public static final double DEFAULT_CAN_CONFIG_BUDGET_SECONDS = 3.0;

  /** Bounded per-call CTRE configurator timeout (seconds). */
  public static final double DEFAULT_CAN_CALL_TIMEOUT_SECONDS = 0.100;

  private static final long RETRY_SLEEP_MS = 25L;

  private static volatile double configBudgetSeconds = DEFAULT_CAN_CONFIG_BUDGET_SECONDS;
  private static volatile long budgetStartNanos = System.nanoTime();
  private static volatile int skippedCalls = 0;
  private static volatile int failedCalls = 0;
  private static volatile boolean budgetWarningReported = false;
  private static volatile String lastFailureMessage = "";

  private PhoenixUtil() {}

  /**
   * Starts or refreshes the boot CAN configuration retry budget immediately before hardware init
   * phases (e.g., after slow {@code AprilTagFieldLayout} loads).
   */
  public static void beginCanInitBudget() {
    beginCanInitBudget(DEFAULT_CAN_CONFIG_BUDGET_SECONDS);
  }

  public static void beginCanInitBudget(double budgetSeconds) {
    configBudgetSeconds = Math.max(0.0, budgetSeconds);
    budgetStartNanos = System.nanoTime();
    budgetWarningReported = false;
  }

  /**
   * Retries a CTRE configuration call until it succeeds, max attempts are exhausted, or the shared
   * boot {@code CanConfigBudget} expires. Always executes at least 1 attempt so motor current and
   * soft limits are never silently skipped after slow boot phases.
   */
  public static StatusCode tryUntilOk(Supplier<StatusCode> configCall, int maxAttempts) {
    return tryUntilOk("CTREConfig", configCall, maxAttempts);
  }

  /**
   * Retries a named CTRE configuration call under the shared boot {@code CanConfigBudget}. Always
   * performs at least 1 attempt even if the retry budget has elapsed, skipping only subsequent
   * retries when out of budget, and reports any final failure loudly to Driver Station.
   */
  public static StatusCode tryUntilOk(
      String label, Supplier<StatusCode> configCall, int maxAttempts) {
    int attempts = Math.max(1, maxAttempts);
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < attempts; i++) {
      if (i > 0 && getRemainingConfigBudgetSeconds() <= 0.0) {
        skippedCalls++;
        reportBudgetExhaustedOnce(label);
        break;
      }
      status = configCall.get();
      if (status.isOK()) {
        return status;
      }
      if (i + 1 < attempts) {
        if (getRemainingConfigBudgetSeconds() <= 0.0) {
          skippedCalls++;
          reportBudgetExhaustedOnce(label);
          break;
        }
        try {
          Thread.sleep(RETRY_SLEEP_MS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          break;
        }
      }
    }
    if (!status.isOK()) {
      failedCalls++;
      lastFailureMessage =
          "PhoenixUtil CAN config FAILED for '"
              + label
              + "' (status="
              + status
              + "); device may retain unsafe defaults.";
      DriverStation.reportError(lastFailureMessage, false);
    }
    return status;
  }

  public static double getRemainingConfigBudgetSeconds() {
    double elapsedSec = (System.nanoTime() - budgetStartNanos) / 1e9;
    return Math.max(0.0, configBudgetSeconds - elapsedSec);
  }

  public static int getSkippedConfigCallCount() {
    return skippedCalls;
  }

  public static int getFailedConfigCallCount() {
    return failedCalls;
  }

  public static String getLastFailureMessage() {
    return lastFailureMessage;
  }

  /** Resets the shared boot config budget (intended for isolated unit/sim test suites). */
  public static void resetConfigBudgetForTest() {
    resetConfigBudgetForTest(DEFAULT_CAN_CONFIG_BUDGET_SECONDS);
  }

  public static void resetConfigBudgetForTest(double budgetSeconds) {
    configBudgetSeconds = Math.max(0.0, budgetSeconds);
    budgetStartNanos = System.nanoTime();
    skippedCalls = 0;
    failedCalls = 0;
    budgetWarningReported = false;
    lastFailureMessage = "";
  }

  private static void reportBudgetExhaustedOnce(String label) {
    if (!budgetWarningReported) {
      budgetWarningReported = true;
      DriverStation.reportWarning(
          "PhoenixUtil CAN config budget ("
              + configBudgetSeconds
              + " s) exhausted before completing '"
              + label
              + "'; skipping remaining retries.",
          false);
    }
  }
}
