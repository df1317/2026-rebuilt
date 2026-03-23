package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotLog;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class ClimberSubsystem extends SubsystemBase {

	// ==================== Hardware Config ====================
	private static final int MOTOR_LEFT_ID = 29;
	private static final boolean INVERTED = true;
	private static final int CURRENT_LIMIT = 20;
	private static final double ROTATIONS_PER_METER = 42.4;
	private static final double POSITION_TOLERANCE_M = 0.02;
	private static final double JOG_SPEED_METERS_PER_SECOND = 0.3;
	private static final double JOG_SOFT_LIMIT_ROTATIONS = 9999.0;
	private static final double GO_TO_HEIGHT_TIMEOUT_SECONDS = 5.0;

	// ==================== Position Enum ====================

	enum Position {
		TOP(3.96), BOTTOM(0.0), HANG(1.75), RELEASE(1.3);

		public final TunableDouble height;

		Position(double heightMeters) {
			this.height = tunables.value("heights/" + name(), heightMeters, Meters);
		}
	}

	// ==================== Hardware ====================
	final TalonFX motorLeft;
	private final TrapezoidProfile profile;
	private final ElevatorFeedforward feedforward;
	private final PositionVoltage positionRequest = new PositionVoltage(0);

	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Climber");
	private final TunableDouble jogCurrentLimit = tunables.value("JogCurrentLimit", 1.0);
	private final ClimberTelemetry telemetry;

	// ==================== Profile State ====================
	TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	TrapezoidProfile.State goalState = new TrapezoidProfile.State();
	boolean prevEnabled = false;
	private double lastUpdateTimestamp;

	public ClimberSubsystem() {
		motorLeft = new TalonFX(MOTOR_LEFT_ID);

		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.Slot0.kP = 0.0;
		configs.Slot0.kI = 0.0;
		configs.Slot0.kD = 0.0;
		configs.Slot0.kV = 4.7;
		configs.Slot0.kG = 0.0;
		configs.Slot0.kS = 0.31;

		configs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		configs.CurrentLimits.SupplyCurrentLimitEnable = true;

		configs.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;

		motorLeft.getConfigurator().apply(configs);

		profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.5, 3.0));
		feedforward = new ElevatorFeedforward(0.31, 0.0, 4.7);

		lastUpdateTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

		tunables.pidTalonFX("Motor", motorLeft, 0.0, 0.0, 0.0, 4.7, 0.31, 0.0);

		telemetry = new ClimberTelemetry(this);

		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;

		// Enum warmup
		Position.TOP.height.get();
	}

	public void onEnabled() {
		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
	}

	@Override
	public void periodic() {
		if (DriverStation.isEnabled() && !prevEnabled) {
			onEnabled();
		}
		prevEnabled = DriverStation.isEnabled();

		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		double dt = now - lastUpdateTimestamp;
		lastUpdateTimestamp = now;

		if (DriverStation.isEnabled()) {
			currentState.position = getHeightMeters();
			currentState = profile.calculate(dt, currentState, goalState);
			double ff = feedforward.calculate(currentState.velocity);
			motorLeft.setControl(
					positionRequest.withPosition(currentState.position * ROTATIONS_PER_METER).withFeedForward(ff));
		}

		telemetry.log();
	}

	double getHeightMeters() {
		return motorLeft.getPosition().getValueAsDouble() / ROTATIONS_PER_METER;
	}

	public boolean isAtGoal() {
		return MathUtil.isNear(goalState.position, getHeightMeters(), POSITION_TOLERANCE_M);
	}

	boolean isAtTop() {
		return getHeightMeters() >= Position.TOP.height.get();
	}

	boolean isAtBottom() {
		return getHeightMeters() <= Position.BOTTOM.height.get();
	}

	Color getStatusColor() {
		if (isAtGoal() && isAtBottom()) {
			return RobotLog.RED;
		} else if (isAtGoal()) {
			return RobotLog.GREEN;
		}
		return RobotLog.YELLOW;
	}

	private void setGoalHeight(double heightMeters) {
		goalState.position = heightMeters;
		goalState.velocity = 0.0;
	}

	public void stop() {
		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
		motorLeft.stopMotor();
	}

	public void resetEncoders() {
		motorLeft.setPosition(0);
		currentState = new TrapezoidProfile.State(0, 0);
		goalState = new TrapezoidProfile.State(0, 0);
	}

	// ==================== Command Factory Methods ====================

	private Command goTo(Position position) {
		return Commands.runOnce(() -> setGoalHeight(position.height.get()), this)
				.andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS)
				.withName("Climber." + position.name().toLowerCase());
	}

	/** Position-based jog using a joystick axis [-1, 1]. Uses wide soft limits and a low current limit. */
	public Command jogVoltageCommand(DoubleSupplier axis) {
		return Commands.runOnce(() -> {
			applyJogConfig();
			goalState.position = getHeightMeters();
			currentState.position = getHeightMeters();
			currentState.velocity = 0.0;
		}, this)
				.andThen(Commands.run(() -> {
					double increment = axis.getAsDouble() * JOG_SPEED_METERS_PER_SECOND * 0.02;
					goalState.position += increment;
					DogLog.log("Climber/JogGoal", goalState.position);
				}, this))
				.finallyDo(() -> {
					disableJogSoftLimits();
					stop();
				})
				.withName("Jog Climber");
	}

	private TalonFXConfiguration baseConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		return config;
	}

	private void applyJogConfig() {
		TalonFXConfiguration config = baseConfig();
		config.CurrentLimits.SupplyCurrentLimit = jogCurrentLimit.get();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = JOG_SOFT_LIMIT_ROTATIONS;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -JOG_SOFT_LIMIT_ROTATIONS;
		motorLeft.getConfigurator().apply(config);
	}

	private void disableJogSoftLimits() {
		TalonFXConfiguration config = baseConfig();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		motorLeft.getConfigurator().apply(config);
	}

	public Command climbBottomCommand() {
		return goTo(Position.BOTTOM);
	}

	public Command climbTopCommand() {
		return goTo(Position.TOP);
	}

	public Command climbHangCommand() {
		return goTo(Position.HANG);
	}

	public Command climbReleaseCommand() {
		return goTo(Position.RELEASE);
	}

	public Command zeroCommand() {
		return Commands.runOnce(this::resetEncoders, this).withName("Climber Zero");
	}

}
