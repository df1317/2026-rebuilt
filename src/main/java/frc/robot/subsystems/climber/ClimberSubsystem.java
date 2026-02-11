package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.CURRENT_LIMIT;
import static frc.robot.Constants.ClimberConstants.KD;
import static frc.robot.Constants.ClimberConstants.KG;
import static frc.robot.Constants.ClimberConstants.KI;
import static frc.robot.Constants.ClimberConstants.KP;
import static frc.robot.Constants.ClimberConstants.KS;
import static frc.robot.Constants.ClimberConstants.KV;
import static frc.robot.Constants.ClimberConstants.MAX_ACCELERATION;
import static frc.robot.Constants.ClimberConstants.MAX_HEIGHT;
import static frc.robot.Constants.ClimberConstants.MAX_VELOCITY;
import static frc.robot.Constants.ClimberConstants.MIN_HEIGHT;
import static frc.robot.Constants.ClimberConstants.MOTOR_LEFT_ID;
import static frc.robot.Constants.ClimberConstants.MOTOR_RIGHT_ID;
import static frc.robot.Constants.ClimberConstants.POSITION_TOLERANCE;
import static frc.robot.Constants.ClimberConstants.ROTATIONS_PER_METER;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

/**
 * Climber subsystem using an elevator mechanism.
 *
 * <p>
 * Uses trapezoidal motion profiling with feedforward control for smooth, controlled climbing.
 * Implements soft limits based on encoder position.
 */
public class ClimberSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry/visualization)
	// ====================
	// final SparkMax motorLeft;
	// private final SparkMax motorRight;
	// private final SparkClosedLoopController controller;
	// final RelativeEncoder encoder;

	final TalonFX motorLeft;
	final TalonFX motorRight;

	// ==================== Control State (package-private for telemetry/visualization)
	// ====================
	private final TrapezoidProfile profile;
	private final ElevatorFeedforward feedforward;
	TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	TrapezoidProfile.State goalState = new TrapezoidProfile.State();
	private double lastUpdateTimestamp;

	// ==================== SysId ====================
	private final MutVoltage appliedVoltage = Volts.mutable(0);
	private final MutDistance distance = Meters.mutable(0);
	private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
	private final SysIdRoutine sysIdRoutine;

	// ==================== Visualization & Telemetry ====================
	private final ClimberVisualization visualization;
	private final ClimberTelemetry telemetry;

	/**
	 * Creates a new ClimberSubsystem.
	 *
	 * <p>
	 * <b>Important:</b> The encoder position is not initialized on construction and will retain
	 * values from previous power cycles or be zero after a fresh boot. Call {@link #resetEncoders()}
	 * during robot initialization or before first use if the climber is not at the zero position.
	 */
	public ClimberSubsystem() {
		motorLeft = new TalonFX(MOTOR_LEFT_ID);
		motorRight = new TalonFX(MOTOR_RIGHT_ID);

		// Configure the TalonFX for basic use
		TalonFXConfiguration configs = new TalonFXConfiguration();
		// This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on
		// slot 0
		configs.Slot0.kP = KP;
		configs.Slot0.kI = KI;
		configs.Slot0.kD = KD;
		configs.Slot0.kV = KV;
		configs.Slot0.kA = KG;
		configs.Slot0.kS = KS;

		configs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		configs.CurrentLimits.SupplyCurrentLimitEnable = true;

		motorLeft.getConfigurator().apply(configs);
		motorRight.getConfigurator().apply(configs);

		// SparkMaxConfig leaderConfig = new SparkMaxConfig();
		// leaderConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(INVERTED);
		// leaderConfig.closedLoop.p(KP, ClosedLoopSlot.kSlot0).i(KI, ClosedLoopSlot.kSlot0).d(KD,
		// ClosedLoopSlot.kSlot0);
		// leaderConfig.signals.primaryEncoderPositionPeriodMs(10);

		// SparkMaxConfig followerConfig = new SparkMaxConfig();
		// followerConfig.smartCurrentLimit(CURRENT_LIMIT).idleMode(IdleMode.kBrake)
		// // Follower inverts relative to leader. With INVERTED=false on leader,
		// // this causes motors to spin in opposite directions (correct for elevator
		// // with mirrored motor mounting). Verify physical mounting matches this assumption.
		// .follow(motorLeft, true);

		// motorLeft.configure(leaderConfig, ResetMode.kResetSafeParameters,
		// PersistMode.kNoPersistParameters);
		// motorRight.configure(followerConfig, ResetMode.kResetSafeParameters,
		// PersistMode.kNoPersistParameters);

		// controller = motorLeft.getClosedLoopController();
		// encoder = motorLeft.getEncoder();
		// encoder.setPosition(0);

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

		visualization = new ClimberVisualization(this);
		telemetry = new ClimberTelemetry(this);

		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
	}

	public void onEnabled() {
		System.out.println("justed ENABLED!");
		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
	}

	boolean prevEnabled = false;

	// ==================== Periodic ====================
	@Override
	public void periodic() {
		if (DriverStation.isEnabled() && !prevEnabled) {
			onEnabled();
		}
		prevEnabled = DriverStation.isEnabled();

		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		double dt = now - lastUpdateTimestamp;
		lastUpdateTimestamp = now;
		// System.out.println("dt: " + dt);

		double measuredHeight = getHeightMeters();
		// System.out.println("before clamp: " + currentState.position);
		// goalState.position =
		// MathUtil.clamp(goalState.position, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));

		if (MathUtil.isNear(goalState.position, measuredHeight, POSITION_TOLERANCE.in(Meters) * 5)) {
			currentState.position = measuredHeight;
			currentState.velocity = 0.0;
		} else {
			currentState.velocity = MathUtil.clamp(currentState.velocity,
					-MAX_VELOCITY.in(MetersPerSecond), MAX_VELOCITY.in(MetersPerSecond));
		}

		currentState = profile.calculate(dt, currentState, goalState);

		if (canMove(currentState.velocity)) {
			double ff = feedforward.calculate(currentState.velocity);

			// currentState.position += 0.1;

			motorLeft.setControl(
					new PositionVoltage(currentState.position * ROTATIONS_PER_METER).withFeedForward(ff));
			// motorLeft.setControl(new PositionVoltage(currentState.position * ROTATIONS_PER_METER));
			motorRight.setControl(new Follower(MOTOR_LEFT_ID, MotorAlignmentValue.Opposed));
			// System.out.println("position: " + currentState.position);
			// System.out.println("goal pos: " + goalState.position);

			// controller.setSetpoint(currentState.position * ROTATIONS_PER_METER, ControlType.kPosition,
			// ClosedLoopSlot.kSlot0, ff);
		} else {
			// Movement blocked by soft limits; realign profile state with measured position
			currentState.position = measuredHeight;
			currentState.velocity = 0.0;
			motorLeft.stopMotor();
		}

		// visualization.update();
		// telemetry.log();
	}

	// ==================== State Queries ====================

	double getHeightMeters() {
		return motorLeft.getPosition().getValueAsDouble() / ROTATIONS_PER_METER;
	}

	private double getVelocityMetersPerSecond() {
		return motorLeft.getVelocity().getValueAsDouble() / 60.0 / ROTATIONS_PER_METER;
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
		if (isAtGoal()) {
			return Color.kGreen;
		} else if (isAtTop() || isAtBottom()) {
			return Color.kOrange;
		}
		return Color.kYellow;
	}

	private boolean canMove(double requestedVelocity) {
		return true;
		// double height = getHeightMeters();
		// if (height >= MAX_HEIGHT.in(Meters) && requestedVelocity > 0) {
		// return false;
		// }
		// return height > MIN_HEIGHT.in(Meters) || requestedVelocity >= 0;
	}

	// ==================== Control Methods ====================
	private void setGoalHeight(double heightMeters) {
		goalState.position = MathUtil.clamp(heightMeters, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));
		goalState.velocity = 0.0;
	}

	private void stop() {
		goalState.position = currentState.position;
		goalState.velocity = 0.0;
		motorLeft.stopMotor();
	}

	public void resetEncoders() {
		motorLeft.setPosition(0);
		motorRight.setPosition(0);
		currentState = new TrapezoidProfile.State(0, 0);
		goalState = new TrapezoidProfile.State(0, 0);
	}

	// ==================== Commands ====================
	private static final double GO_TO_HEIGHT_TIMEOUT_SECONDS = 5.0;

	public Command goToHeightCommand(double heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters), this)
				.andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	public Command goToHeightCommand(DoubleSupplier heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters.getAsDouble()), this)
				.andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	/**
	 * Creates a command for manual climber control.
	 *
	 * <p>
	 * When the command ends (button released), the climber holds its current position by setting the
	 * goal to the current profile state.
	 *
	 * @param speedInput
	 *          supplier for joystick input (-1 to 1)
	 * @return command for manual control
	 */
	public Command manualControlCommand(DoubleSupplier speedInput) {
		return Commands.run(() -> {
			double input = speedInput.getAsDouble();
			double targetHeight = input > 0 ? MAX_HEIGHT.in(Meters)
					: (input < 0 ? MIN_HEIGHT.in(Meters) : currentState.position);
			// setGoalHeight(targetHeight);
			goalState.position += input;
		}, this).finallyDo(this::stop);
	}

	public Command extendCommand() {
		return goToHeightCommand(MAX_HEIGHT.in(Meters)).withName("Climber Extend");
	}

	public Command retractCommand() {
		return goToHeightCommand(MIN_HEIGHT.in(Meters)).withName("Climber Retract");
	}

	public Command zeroCommand() {
		return Commands.runOnce(this::resetEncoders, this).withName("Climber Zero");
	}

	// ==================== SysId ====================
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
