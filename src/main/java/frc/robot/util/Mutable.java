package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A simple mutable wrapper for use in command factory lambdas.
 *
 * <p>
 * Java requires variables captured by lambdas to be effectively final.
 * Wrapping a value in {@code Mutable} lets command factories hold per-instance
 * state that can be mutated inside {@code onExecute}/{@code onEnd} lambdas.
 *
 * <p>
 * This is safe because WPILib commands run synchronously on the main thread.
 *
 * <p>
 * Inspired by Team 340 (GRR).
 */
public class Mutable<T> implements Supplier<T>, Consumer<T> {

	public T value;

	public Mutable(T value) {
		this.value = value;
	}

	@Override
	public T get() {
		return value;
	}

	@Override
	public void accept(T value) {
		this.value = value;
	}
}
