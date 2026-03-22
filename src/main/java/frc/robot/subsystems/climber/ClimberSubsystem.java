package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.RobotLog;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {

	private static final double GO_TO_HEIGHT_TIMEOUT_SECONDS = 5.0;
	final TalonFX motorLeft;
	private final TrapezoidProfile profile;
	private final ElevatorFeedforward feedforward;
	private final MutVoltage appliedVoltage = Volts.mutable(0);
	private final MutDistance distance = Meters.mutable(0);
	private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
	private final SysIdRoutine sysIdRoutine;
	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Climber");
	private final TunableDouble testClimberHeight = tunables.value("Height", MAX_HEIGHT.in(Meters), Meters);
	private final TunableDouble jogCurrentLimit = tunables.value("JogCurrentLimit", JOG_CURRENT_LIMIT);
	private final ClimberTelemetry telemetry;
	TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	TrapezoidProfile.State goalState = new TrapezoidProfile.State();
	boolean prevEnabled = false;
	private double lastUpdateTimestamp;

	public ClimberSubsystem() {
		motorLeft = new TalonFX(MOTOR_LEFT_ID);

		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.Slot0.kP = KP;
		configs.Slot0.kI = KI;
		configs.Slot0.kD = KD;
		configs.Slot0.kV = KV;
		configs.Slot0.kG = KG;
		configs.Slot0.kS = KS;

		configs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		configs.CurrentLimits.SupplyCurrentLimitEnable = true;

		configs.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;

		motorLeft.getConfigurator().apply(configs);

		profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VELOCITY.in(MetersPerSecond),
				MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
		feedforward = new ElevatorFeedforward(KS, KG, KV);

		lastUpdateTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

		sysIdRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(null, Voltage.ofBaseUnits(5, Volts), null, null),
				new SysIdRoutine.Mechanism(
						(Voltage volts) -> motorLeft.setVoltage(volts.baseUnitMagnitude()),
						log -> log.motor("climber")
								.voltage(appliedVoltage.mut_replace(motorLeft.getMotorVoltage().getValueAsDouble()
										* motorLeft.getDutyCycle().getValueAsDouble(), Volts))
								.linearPosition(distance.mut_replace(getHeightMeters(), Meters)).linearVelocity(
										velocity.mut_replace(getVelocityMetersPerSecond(), MetersPerSecond)),
						this));

		tunables.pidTalonFX("Motor", motorLeft, KP, KI, KD, KV, KS, KG);

		telemetry = new ClimberTelemetry(this);

		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
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
					new PositionVoltage(currentState.position * ROTATIONS_PER_METER).withFeedForward(ff));
		}

		telemetry.log();
	}

	double getHeightMeters() {
		return motorLeft.getPosition().getValueAsDouble() / ROTATIONS_PER_METER;
	}

	private double getVelocityMetersPerSecond() {
		return motorLeft.getVelocity().getValueAsDouble() / ROTATIONS_PER_METER;
	}

	public boolean isAtGoal() {
		return MathUtil.isNear(goalState.position, getHeightMeters(), POSITION_TOLERANCE.in(Meters));
	}

	boolean isAtTop() {
		return getHeightMeters() >= MAX_HEIGHT.in(Meters);
	}

	boolean isAtBottom() {
		return getHeightMeters() <= MIN_HEIGHT.in(Meters);
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

	public Command goToHeightCommand(double heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters), this)
				.andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	public Command goToHeightCommand(DoubleSupplier heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters.getAsDouble()), this)
				.andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	/** Manual control; holds position when released. */
	public Command manualControlCommand(DoubleSupplier speedInput) {
		return Commands.run(() -> {
			double input = speedInput.getAsDouble();
			goalState.position += input;
		}, this).finallyDo(this::stop);
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

	public Command extendCommand() {
		return goToHeightCommand(MAX_HEIGHT.in(Meters)).withName("Climber Extend");
	}

	public Command retractCommand() {
		return goToHeightCommand(MIN_HEIGHT.in(Meters)).withName("Climber Retract");
	}

	public Command climbBottomCommand() {
		return goToHeightCommand(MIN_HEIGHT.in(Meters)).withName("Climb Bottom");
	}

	public Command climbTopCommand() {
		return goToHeightCommand(MAX_HEIGHT.in(Meters)).withName("Climb Top");
	}

	public Command climbHangCommand() {
		return goToHeightCommand(HANG_HEIGHT.in(Meters)).withName("Climb Hang");
	}

	public Command climbReleaseCommand() {
		return goToHeightCommand(RELEASE_HEIGHT.in(Meters)).withName("Climb Release");
	}

	public Command zeroCommand() {
		return Commands.runOnce(this::resetEncoders, this).withName("Climber Zero");
	}

	public Command testClimberCommand() {
		return goToHeightCommand(() -> testClimberHeight.get()).withName("Test Climber");
	}

	public Command sysIdQuasistatic(Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	public Command sysIdFullCommand(double quasiTimeout, double pauseTimeout, double dynamicTimeout) {
		return sysIdRoutine.quasistatic(Direction.kForward).withTimeout(quasiTimeout)
				.andThen(Commands.waitSeconds(pauseTimeout))
				.andThen(sysIdRoutine.quasistatic(Direction.kReverse).withTimeout(quasiTimeout))
				.andThen(Commands.waitSeconds(pauseTimeout))
				.andThen(sysIdRoutine.dynamic(Direction.kForward).withTimeout(dynamicTimeout))
				.andThen(Commands.waitSeconds(pauseTimeout))
				.andThen(sysIdRoutine.dynamic(Direction.kReverse).withTimeout(dynamicTimeout))
				.withName("Climber SysId");
	}
}
