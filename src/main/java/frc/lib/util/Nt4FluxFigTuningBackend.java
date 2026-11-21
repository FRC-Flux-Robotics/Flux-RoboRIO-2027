package frc.lib.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import frc.lib.fluxfig.FluxFig;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

/**
 * WPILib NetworkTables 4 adapter for {@link FluxFig.TuningBackend}. Publishes numeric tunables as
 * native {@link DoubleTopic} widgets (and non-numeric values as {@link StringTopic}) under {@code
 * /FluxFig/Tuning/*} while suppressing formatting-only floating-point baseline diffs.
 */
public final class Nt4FluxFigTuningBackend implements FluxFig.TuningBackend, AutoCloseable {

  private static final class NumericBinding {
    final DoublePublisher publisher;
    final DoubleSubscriber subscriber;
    final String baselineString;
    final double baselineValue;

    NumericBinding(
        DoublePublisher publisher,
        DoubleSubscriber subscriber,
        String baselineString,
        double baselineValue) {
      this.publisher = publisher;
      this.subscriber = subscriber;
      this.baselineString = baselineString;
      this.baselineValue = baselineValue;
    }
  }

  private static final class TextBinding {
    final StringPublisher publisher;
    final StringSubscriber subscriber;

    TextBinding(StringPublisher publisher, StringSubscriber subscriber) {
      this.publisher = publisher;
      this.subscriber = subscriber;
    }
  }

  private final NetworkTableInstance ntInstance;
  private final Map<String, NumericBinding> numericBindings = new LinkedHashMap<>();
  private final Map<String, TextBinding> textBindings = new LinkedHashMap<>();

  public Nt4FluxFigTuningBackend() {
    this(NetworkTableInstance.getDefault());
  }

  public Nt4FluxFigTuningBackend(NetworkTableInstance ntInstance) {
    this.ntInstance = (ntInstance != null) ? ntInstance : NetworkTableInstance.getDefault();
  }

  @Override
  public synchronized void publish(String topic, String value) {
    if (topic == null || topic.isBlank() || value == null) {
      return;
    }
    String trimmed = value.trim();
    Double parsed = tryParseFiniteDouble(trimmed);
    if (parsed != null) {
      NumericBinding existing = numericBindings.get(topic);
      if (existing != null) {
        existing.publisher.set(parsed);
        numericBindings.put(
            topic, new NumericBinding(existing.publisher, existing.subscriber, trimmed, parsed));
        return;
      }
      DoubleTopic ntTopic = ntInstance.getDoubleTopic(topic);
      DoublePublisher pub = ntTopic.publish();
      pub.set(parsed);
      DoubleSubscriber sub = ntTopic.subscribe(parsed);
      numericBindings.put(topic, new NumericBinding(pub, sub, trimmed, parsed));
      return;
    }

    TextBinding existingText = textBindings.get(topic);
    if (existingText != null) {
      existingText.publisher.set(trimmed);
      return;
    }
    StringTopic ntTopic = ntInstance.getStringTopic(topic);
    StringPublisher pub = ntTopic.publish();
    pub.set(trimmed);
    StringSubscriber sub = ntTopic.subscribe(trimmed);
    textBindings.put(topic, new TextBinding(pub, sub));
  }

  @Override
  public synchronized Optional<String> read(String topic) {
    NumericBinding numeric = numericBindings.get(topic);
    if (numeric != null) {
      double current = numeric.subscriber.get();
      if (!Double.isFinite(current)) {
        return Optional.empty();
      }
      if (Math.abs(current - numeric.baselineValue) <= 1e-9) {
        return Optional.of(numeric.baselineString);
      }
      return Optional.of(Double.toString(current));
    }
    TextBinding text = textBindings.get(topic);
    if (text != null) {
      return Optional.ofNullable(text.subscriber.get());
    }
    return Optional.empty();
  }

  public synchronized int getPublishedTopicCount() {
    return numericBindings.size() + textBindings.size();
  }

  private static Double tryParseFiniteDouble(String raw) {
    try {
      double d = Double.parseDouble(raw);
      return Double.isFinite(d) ? d : null;
    } catch (NumberFormatException e) {
      return null;
    }
  }

  @Override
  public synchronized void close() {
    for (NumericBinding b : numericBindings.values()) {
      b.subscriber.close();
      b.publisher.close();
    }
    numericBindings.clear();
    for (TextBinding b : textBindings.values()) {
      b.subscriber.close();
      b.publisher.close();
    }
    textBindings.clear();
  }
}
