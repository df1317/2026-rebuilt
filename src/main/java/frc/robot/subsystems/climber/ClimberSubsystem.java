package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
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
import static frc.robot.Constants.ClimberConstants.POSITION_TOLERANCE;
import static frc.robot.Constants.ClimberConstants.ROTATIONS_PER_METER;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class ClimberSubsystem extends SubsystemBase {

	final TalonFX motorLeft;


	private boolean velocityMode = false;
	private AngularVelocity targetVelocity = RPM.of(0);

	private final TrapezoidProfile profile;
	private final ElevatorFeedforward feedforward;
	TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	TrapezoidProfile.State goalState = new TrapezoidProfile.State();
	private double lastUpdateTimestamp;

	private final MutVoltage appliedVoltage = Volts.mutable(0);
	private final MutDistance distance = Meters.mutable(0);
	private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
	private final SysIdRoutine sysIdRoutine;

	private final ClimberVisualization visualization;
	private final ClimberTelemetry telemetry;

	public ClimberSubsystem() {
		motorLeft = new TalonFX(MOTOR_LEFT_ID);

		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.Slot0.kP = KP;
		configs.Slot0.kI = KI;
		configs.Slot0.kD = KD;
		configs.Slot0.kV = KV;
		configs.Slot0.kA = KG;
		configs.Slot0.kS = KS;

		configs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		configs.CurrentLimits.SupplyCurrentLimitEnable = true;

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

		visualization = new ClimberVisualization(this);
		telemetry = new ClimberTelemetry(this);

		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
	}

	public void onEnabled() {
		System.out.println("just ENABLED!");
		goalState.position = getHeightMeters();
		currentState.position = getHeightMeters();
		currentState.velocity = 0.0;
		goalState.velocity = 0.0;
		targetVelocity = RPM.of(0);
	}

	boolean prevEnabled = false;

	@Override
	public void periodic() {
		if (DriverStation.isEnabled() && !prevEnabled) {
			onEnabled();
		}
		prevEnabled = DriverStation.isEnabled();

		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		double dt = now - lastUpdateTimestamp;
		lastUpdateTimestamp = now;

		double measuredHeight = getHeightMeters();

		currentState = profile.calculate(dt, currentState, goalState);
		if (!velocityMode) {
			if (canMove(currentState.velocity)) {
				double ff = feedforward.calculate(currentState.velocity);
				motorLeft.setControl(
						new PositionVoltage(currentState.position * ROTATIONS_PER_METER).withFeedForward(ff));
			} else {
				currentState.position = measuredHeight;
				currentState.velocity = 0.0;
				motorLeft.stopMotor();
			}
		} else {
			motorLeft.setControl(new VelocityVoltage(targetVelocity));
		}

		telemetry.log();
	}

	public boolean isClimberStalled() {
		double climberMotorCurrent = motorLeft.getMotorVoltage().getValueAsDouble();
		double climberMotorRPM = motorLeft.getVelocity().getValueAsDouble(); // RPM
		boolean isPivotStalled = Math.abs(climberMotorRPM) < 2.0 && climberMotorCurrent > CURRENT_LIMIT * 0.5;
		return stallDebouncer.calculate(isPivotStalled);
	}

	private final Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

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
	}

	private void setGoalHeight(double heightMeters) {
		System.out.println("go to height " + heightMeters);
		goalState.position = heightMeters;
		goalState.velocity = 0.0;
		velocityMode = false;
	}

	private void setGoalVelocity(AngularVelocity velo) {
		velocityMode = true;
		targetVelocity = velo;
	}

	private void stop() {
		goalState.position = currentState.position;
		goalState.velocity = 0.0;
		targetVelocity = RPM.of(0);
		motorLeft.stopMotor();
	}

	public void resetEncoders() {
		motorLeft.setPosition(0);
		currentState = new TrapezoidProfile.State(0, 0);
		goalState = new TrapezoidProfile.State(0, 0);
	}

	private static final double GO_TO_HEIGHT_TIMEOUT_SECONDS = 5.0;

	public Command goToHeightCommand(double heightMeters) {
		return Commands.runOnce(() -> {
			System.out.println("called set goal");
			setGoalHeight(heightMeters);
		}).andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	public Command goToHeightCommand(DoubleSupplier heightMeters) {
		return Commands.runOnce(() -> setGoalHeight(heightMeters.getAsDouble()), this)
				.andThen(Commands.waitUntil(this::isAtGoal)).withTimeout(GO_TO_HEIGHT_TIMEOUT_SECONDS);
	}

	public Command goToVelocityCommand(AngularVelocity velo) {
		return Commands.runOnce(() -> this.setGoalVelocity(velo));
	}

	/** Manual control; holds position when released. */
	public Command manualControlCommand(DoubleSupplier speedInput) {
		return Commands.run(() -> {
			double input = speedInput.getAsDouble();
			goalState.position += input;
		}, this).finallyDo(this::stop);
	}

	public Command extendCommand() {
		return goToHeightCommand(MAX_HEIGHT.in(Meters))
				// // Uncomment below to enable limit switch
				// .andThen(idle().until(() -> isClimberStalled() || isAtGoal()))
				// .andThen(runOnce(() -> goToHeightCommand(this::getHeightMeters)))
				.withName("Climber Extend");
	}

	public Command retractCommand() {
		return goToHeightCommand(MIN_HEIGHT.in(Meters))
				// // Uncomment below to enable limit switch
				// .andThen(idle().until(() -> isClimberStalled() || isAtGoal()))
				// .andThen(runOnce(() -> goToHeightCommand(this::getHeightMeters)))
				.withName("Climber Retract");
	}

	public Command zeroCommand() {
		return Commands.runOnce(this::resetEncoders, this).withName("Climber Zero");
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
