package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;

/**
 * Climber subsystem using an elevator mechanism.
 *
 * <p>
 * Uses trapezoidal motion profiling with feedforward control for smooth,
 * controlled climbing. Implements soft limits based on encoder position.
 */
public class ClimberSubsystem extends SubsystemBase {

	// ==================== Mechanism2d Visualization ====================
	private static final double VIZ_WIDTH = 0.5;
	private static final double VIZ_HEIGHT = MAX_HEIGHT.in(Meters) * 1.2;
	// ==================== Hardware ====================
	private final SparkMax motorLeft;
	private final SparkMax motorRight;
	private final SparkClosedLoopController controller;
	private final RelativeEncoder encoder;
	// ==================== Control State ====================
	private final TrapezoidProfile profile;
	private final ElevatorFeedforward feedforward;
	// SysId mutable holders
	private final MutVoltage appliedVoltage = Volts.mutable(0);
	private final MutDistance distance = Meters.mutable(0);
	private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
	private final SysIdRoutine sysIdRoutine;
	private final Mechanism2d mechanism2d;
	private final MechanismLigament2d elevatorLigament;
	private final MechanismLigament2d setpointLigament;
	private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
	private double lastUpdateTimestamp;

	/**
	 * Creates a new ClimberSubsystem.
	 *
	 * <p>
	 * <b>Important:</b> The encoder position is not initialized on construction and will retain
	 * values from previous power cycles or be zero after a fresh boot. Call {@link #resetEncoders()}
	 * during robot initialization or before first use if the climber is not at the zero position.
	 */
	public ClimberSubsystem() {
		motorLeft = new SparkMax(MOTOR_LEFT_ID, MotorType.kBrushless);
		motorRight = new SparkMax(MOTOR_RIGHT_ID, MotorType.kBrushless);

		SparkMaxConfig leaderConfig = new SparkMaxConfig();
		leaderConfig
				.smartCurrentLimit(CURRENT_LIMIT)
				.idleMode(IdleMode.kBrake)
				.inverted(INVERTED);
		leaderConfig.closedLoop
				.p(KP, ClosedLoopSlot.kSlot0)
				.i(KI, ClosedLoopSlot.kSlot0)
				.d(KD, ClosedLoopSlot.kSlot0);
		leaderConfig.signals.primaryEncoderPositionPeriodMs(10);

		SparkMaxConfig followerConfig = new SparkMaxConfig();
		followerConfig
				.smartCurrentLimit(CURRENT_LIMIT)
				.idleMode(IdleMode.kBrake)
				// Follower inverts relative to leader. With INVERTED=false on leader,
				// this causes motors to spin in opposite directions (correct for elevator
				// with mirrored motor mounting). Verify physical mounting matches this assumption.
				.follow(motorLeft, true);

		motorLeft.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		motorRight.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		controller = motorLeft.getClosedLoopController();
		encoder = motorLeft.getEncoder();
		encoder.setPosition(0);

		profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				MAX_VELOCITY.in(MetersPerSecond), MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
		feedforward = new ElevatorFeedforward(KS, KG, KV);

		lastUpdateTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

		sysIdRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(null, Voltage.ofBaseUnits(5, Volts), null, null),
				new SysIdRoutine.Mechanism(
						motorLeft::setVoltage,
						log -> log.motor("climber")
								.voltage(appliedVoltage.mut_replace(
										motorLeft.getBusVoltage() * motorLeft.getAppliedOutput(), Volts))
								.linearPosition(distance.mut_replace(getHeightMeters(), Meters))
								.linearVelocity(velocity.mut_replace(getVelocityMetersPerSecond(), MetersPerSecond)),
						this));

		// Set up Mechanism2d visualization (like YAMS)
		mechanism2d = new Mechanism2d(VIZ_WIDTH, VIZ_HEIGHT);

		// Root at the bottom center
		MechanismRoot2d root = mechanism2d.getRoot("ClimberRoot", VIZ_WIDTH / 2, 0.05);

		// Hard limit indicators
		mechanism2d.getRoot("MinLimit", VIZ_WIDTH / 2 - 0.05, 0.05)
				.append(new MechanismLigament2d("Min", MIN_HEIGHT.in(Meters) + 0.01, 90, 2, new Color8Bit(Color.kRed)));
		mechanism2d.getRoot("MaxLimit", VIZ_WIDTH / 2 - 0.05, 0.05)
				.append(new MechanismLigament2d("Max", MAX_HEIGHT.in(Meters), 90, 2, new Color8Bit(Color.kLimeGreen)));

		// Setpoint indicator (thin white line showing target)
		setpointLigament = root.append(
				new MechanismLigament2d("Setpoint", 0, 90, 3, new Color8Bit(Color.kWhite)));

		// Main elevator ligament (thick cyan showing actual position)
		elevatorLigament = root.append(
				new MechanismLigament2d("Elevator", 0, 90, 6, new Color8Bit(Color.kCyan)));

		SmartDashboard.putData("Climber/mechanism", mechanism2d);
	}

	// ==================== Periodic ====================
	@Override
	public void periodic() {
		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		double dt = now - lastUpdateTimestamp;
		lastUpdateTimestamp = now;

		double measuredHeight = getHeightMeters();

		goalState.position = MathUtil.clamp(goalState.position, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));

		if (!MathUtil.isNear(goalState.position, measuredHeight, POSITION_TOLERANCE.in(Meters) * 5)) {
			currentState.position = measuredHeight;
			currentState.velocity = 0.0;
		} else {
			currentState.velocity = MathUtil.clamp(currentState.velocity,
					-MAX_VELOCITY.in(MetersPerSecond), MAX_VELOCITY.in(MetersPerSecond));
		}

		currentState = profile.calculate(dt, currentState, goalState);

		if (canMove(currentState.velocity)) {
			double ff = feedforward.calculate(currentState.velocity);
			controller.setSetpoint(
					currentState.position * ROTATIONS_PER_METER,
					ControlType.kPosition,
					ClosedLoopSlot.kSlot0,
					ff);
		} else {
			// Movement blocked by soft limits; realign profile state with measured position
			currentState.position = measuredHeight;
			currentState.velocity = 0.0;
			motorLeft.stopMotor();
		}

		logTelemetry(measuredHeight);
	}

	// ==================== State Queries ====================
	public double getHeightMeters() {
		return encoder.getPosition() / ROTATIONS_PER_METER;
	}

	public double getVelocityMetersPerSecond() {
		return encoder.getVelocity() / 60.0 / ROTATIONS_PER_METER;
	}

	public boolean isAtGoal() {
		return MathUtil.isNear(goalState.position, getHeightMeters(), POSITION_TOLERANCE.in(Meters));
	}

	public boolean isAtTop() {
		return getHeightMeters() >= MAX_HEIGHT.in(Meters);
	}

	public boolean isAtBottom() {
		return getHeightMeters() <= MIN_HEIGHT.in(Meters);
	}

	private boolean canMove(double requestedVelocity) {
		if (isAtTop() && requestedVelocity > 0) {
			return false;
		}
		return !isAtBottom() || !(requestedVelocity < 0);
	}

	public Color getStatusColor() {
		if (isAtGoal()) {
			return Color.kGreen;
		} else if (isAtTop() || isAtBottom()) {
			return Color.kOrange;
		}
		return Color.kYellow;
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
		encoder.setPosition(0);
		currentState = new TrapezoidProfile.State(0, 0);
		goalState = new TrapezoidProfile.State(0, 0);
	}

	// ==================== Commands ====================
	private static final double GO_TO_HEIGHT_TIMEOUT_SECONDS = 5.0;

	public Command goToHeightCommand(double heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters), this)
				.andThen(Commands.waitUntil(this::isAtGoal))
				.withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	public Command goToHeightCommand(DoubleSupplier heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters.getAsDouble()), this)
				.andThen(Commands.waitUntil(this::isAtGoal))
				.withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	/**
	 * Creates a command for manual climber control.
	 *
	 * <p>
	 * When the command ends (button released), the climber holds its current position
	 * by setting the goal to the current profile state.
	 *
	 * @param speedInput
	 *          supplier for joystick input (-1 to 1)
	 * @return command for manual control
	 */
	public Command manualControlCommand(DoubleSupplier speedInput) {
		return Commands.run(() -> {
			double input = MathUtil.applyDeadband(speedInput.getAsDouble(), 0.1);
			double targetHeight = input > 0
					? MAX_HEIGHT.in(Meters)
					: (input < 0 ? MIN_HEIGHT.in(Meters) : currentState.position);
			setGoalHeight(targetHeight);
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

	// ==================== Telemetry ====================
	private void logTelemetry(double measuredHeight) {
		// Update Mechanism2d visualization
		elevatorLigament.setLength(measuredHeight);
		setpointLigament.setLength(goalState.position);

		// Change color based on state (like YAMS status indicators)
		if (isAtGoal()) {
			elevatorLigament.setColor(new Color8Bit(Color.kGreen));
		} else if (isAtTop() || isAtBottom()) {
			elevatorLigament.setColor(new Color8Bit(Color.kOrange));
		} else {
			elevatorLigament.setColor(new Color8Bit(Color.kCyan));
		}

		// Status for LED strip
		DogLog.forceNt.log("Climber/Status", getStatusColor().toHexString());

		// Position & velocity (always logged)
		DogLog.log("Climber/HeightMeters", measuredHeight);
		DogLog.log("Climber/GoalHeightMeters", goalState.position);
		DogLog.log("Climber/ProfilePositionMeters", currentState.position);
		DogLog.log("Climber/ProfileVelocityMps", currentState.velocity);

		// State flags
		DogLog.log("Climber/AtGoal", isAtGoal());
		DogLog.log("Climber/AtTop", isAtTop());
		DogLog.log("Climber/AtBottom", isAtBottom());

		// Motor data
		DogLog.log("Climber/EncoderRotations", encoder.getPosition());
		DogLog.log("Climber/MotorCurrentAmps", motorLeft.getOutputCurrent());
		DogLog.log("Climber/MotorVoltage", motorLeft.getBusVoltage() * motorLeft.getAppliedOutput());
	}
}
