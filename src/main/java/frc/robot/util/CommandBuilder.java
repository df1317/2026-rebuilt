package frc.robot.util;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ConcurrentModificationException;
import java.util.function.BooleanSupplier;

/**
 * Fluent command builder. Stylistic alternative to WPILib's decorator chains.
 *
 * <p>
 * Usage:
 *
 * <pre>{@code
 * new CommandBuilder("Shoot", this)
 * 		.onExecute(() -> motor.setControl(velocity))
 * 		.onEnd(() -> motor.stopMotor());
 * }</pre>
 *
 * <p>
 * Inspired by Team 340 (GRR).
 */
public class CommandBuilder extends Command {

	private Runnable onInitialize = () -> {
	};
	private Runnable onExecute = () -> {
	};
	private BooleanConsumer onEnd = interrupted -> {
	};
	private BooleanSupplier isFinished = () -> false;

	public CommandBuilder(Subsystem... requirements) {
		addRequirements(requirements);
	}

	public CommandBuilder(String name, Subsystem... requirements) {
		this(requirements);
		setName(name);
	}

	public CommandBuilder onInitialize(Runnable onInitialize) {
		guardScheduled();
		this.onInitialize = onInitialize;
		return this;
	}

	public CommandBuilder onExecute(Runnable onExecute) {
		guardScheduled();
		this.onExecute = onExecute;
		return this;
	}

	public CommandBuilder onEnd(Runnable onEnd) {
		return onEnd(interrupted -> onEnd.run());
	}

	public CommandBuilder onEnd(BooleanConsumer onEnd) {
		guardScheduled();
		this.onEnd = onEnd;
		return this;
	}

	public CommandBuilder isFinished(boolean isFinished) {
		return isFinished(() -> isFinished);
	}

	public CommandBuilder isFinished(BooleanSupplier isFinished) {
		guardScheduled();
		this.isFinished = isFinished;
		return this;
	}

	@Override
	public void initialize() {
		onInitialize.run();
	}

	@Override
	public void execute() {
		onExecute.run();
	}

	@Override
	public void end(boolean interrupted) {
		onEnd.accept(interrupted);
	}

	@Override
	public boolean isFinished() {
		return isFinished.getAsBoolean();
	}

	private void guardScheduled() {
		if (this.isScheduled()) {
			throw new ConcurrentModificationException("Cannot change methods of a command while it is scheduled");
		}
	}
}
