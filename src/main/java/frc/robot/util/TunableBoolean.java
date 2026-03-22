package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.util.function.BooleanConsumer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * A tunable boolean backed by {@link DogLog#tunable}. Provides a clean {@code .get()} API
 * and listener-based change notification.
 *
 * <p>
 * Inspired by Team 340 (GRR).
 */
public final class TunableBoolean implements BooleanSupplier {

	private final BooleanSubscriber subscriber;
	private final List<BooleanConsumer> listeners = new ArrayList<>();

	TunableBoolean(String key, boolean defaultValue) {
		this.subscriber = DogLog.tunable(key, defaultValue, this::notifyListeners);
	}

	/** Returns the current tunable value. */
	public boolean get() {
		return subscriber.getAsBoolean();
	}

	@Override
	public boolean getAsBoolean() {
		return get();
	}

	/**
	 * Registers a listener that fires when the value changes.
	 *
	 * @return this, for chaining
	 */
	public TunableBoolean addListener(BooleanConsumer listener) {
		listeners.add(listener);
		return this;
	}

	private void notifyListeners(boolean value) {
		for (BooleanConsumer listener : listeners) {
			listener.accept(value);
		}
	}
}
