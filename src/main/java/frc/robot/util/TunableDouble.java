package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Unit;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * A tunable double backed by {@link DogLog#tunable}. Provides a clean {@code .get()} API
 * and listener-based change notification — no manual polling or previous-value tracking needed.
 *
 * <p>
 * Change detection and FMS gating are handled by DogLog's internal notifier thread.
 * Listeners fire on that thread, not the main robot loop.
 *
 * <p>
 * Inspired by Team 340 (GRR).
 */
public final class TunableDouble implements DoubleSupplier {

	private final DoubleSubscriber subscriber;
	private final List<DoubleConsumer> listeners = new ArrayList<>();

	TunableDouble(String key, double defaultValue) {
		this.subscriber = DogLog.tunable(key, defaultValue, this::notifyListeners);
	}

	TunableDouble(String key, double defaultValue, Unit unit) {
		this.subscriber = DogLog.tunable(key, defaultValue, unit, this::notifyListeners);
	}

	/** Returns the current tunable value. */
	public double get() {
		return subscriber.getAsDouble();
	}

	@Override
	public double getAsDouble() {
		return get();
	}

	/**
	 * Registers a listener that fires when the value changes.
	 * Listeners are invoked on DogLog's notifier thread.
	 *
	 * @return this, for chaining
	 */
	public TunableDouble addListener(DoubleConsumer listener) {
		listeners.add(listener);
		return this;
	}

	private void notifyListeners(double value) {
		for (DoubleConsumer listener : listeners) {
			listener.accept(value);
		}
	}
}
