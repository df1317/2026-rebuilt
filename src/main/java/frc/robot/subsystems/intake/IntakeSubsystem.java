package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Intake subsystem with a pivoting arm and roller mechanism.
 */
public class IntakeSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry/visualization)
	// ====================
	final SparkMax pivotMotor;
	final SparkFlex rollerMotor;
	final RelativeEncoder pivotEncoder;
	final RelativeEncoder rollerEncoder;
	private final SparkClosedLoopController pivotController;
	private final SparkClosedLoopController rollerController;
	private final Debouncer atPositionDebouncer;
	private final Debouncer stallDebouncer = new Debouncer(0.1, DebounceType.kBoth);
	private final DoubleSubscriber testPivotDeg = DogLog.tunable("Intake/Pivot/Degrees",
			PIVOT_EXTENDED_ANGLE.in(Degrees), Degrees);
	private final DoubleSubscriber testRollerRPM = DogLog.tunable("Intake/Roller/RPM",
			ROLLER_INTAKE_VELOCITY.in(RPM), RPM);
	// ==================== Visualization & Telemetry ====================
	private final IntakeVisualization visualization;
	private final IntakeTelemetry telemetry;
	// Pivot PID tunables
	private final DoubleSubscriber tunePivotKP = DogLog.tunable("Intake/Pivot/kP", PIVOT_KP);
	private final DoubleSubscriber tunePivotKI = DogLog.tunable("Intake/Pivot/kI", PIVOT_KI);
	private final DoubleSubscriber tunePivotKD = DogLog.tunable("Intake/Pivot/kD", PIVOT_KD);
	private final DoubleSubscriber tunePivotKV = DogLog.tunable("Intake/Pivot/kV", 0.0);
	// Roller PID tunables
	private final DoubleSubscriber tuneRollerKP = DogLog.tunable("Intake/Roller/kP", ROLLER_KP);
	private final DoubleSubscriber tuneRollerKI = DogLog.tunable("Intake/Roller/kI", ROLLER_KI);
	private final DoubleSubscriber tuneRollerKD = DogLog.tunable("Intake/Roller/kD", ROLLER_KD);
	private final DoubleSubscriber tuneRollerKV = DogLog.tunable("Intake/Roller/kV", ROLLER_KV);
	// ==================== Control State (package-private for telemetry/visualization)
	// ====================
	Angle targetPivotAngle = PIVOT_RETRACTED_ANGLE;
	AngularVelocity targetRollerVelocity = RPM.of(0);
	private double prevPivotKP = PIVOT_KP, prevPivotKI = PIVOT_KI, prevPivotKD = PIVOT_KD, prevPivotKV = 0.0;
	private double prevRollerKP = ROLLER_KP, prevRollerKI = ROLLER_KI, prevRollerKD = ROLLER_KD, prevRollerKV = ROLLER_KV;
	private final ProfiledPIDController pivotProfiler = new ProfiledPIDController(0, 0, 0,
			new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY_DEG_PER_S, PIVOT_MAX_ACCEL_DEG_PER_S2));

	public IntakeSubsystem() {
		pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
		rollerMotor = new SparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
		pivotController = pivotMotor.getClosedLoopController();
		rollerController = rollerMotor.getClosedLoopController();
		pivotEncoder = pivotMotor.getEncoder();
		rollerEncoder = rollerMotor.getEncoder();

		configurePivotMotor();
		configureRollerMotor();

		atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_TIME, DebounceType.kRising);

		visualization = new IntakeVisualization(this);
		telemetry = new IntakeTelemetry(this);

		pivotProfiler.reset(PIVOT_RETRACTED_ANGLE.in(Degrees));
		setPivotAngle(PIVOT_RETRACTED_ANGLE);
	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit(PIVOT_CURRENT_LIMIT)
				.inverted(PIVOT_INVERTED);
		config.encoder
				// Converts encoder rotations to degrees: (360 deg/rot) / gear_ratio
				.positionConversionFactor(360.0 / PIVOT_GEAR_RATIO);
		config.closedLoop.pid(PIVOT_KP, PIVOT_KI, PIVOT_KD)
				.allowedClosedLoopError(PIVOT_ANGLE_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureRollerMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kCoast).smartCurrentLimit(ROLLER_CURRENT_LIMIT)
				.inverted(ROLLER_INVERTED);
		config.closedLoop.pid(ROLLER_KP, ROLLER_KI, ROLLER_KD).iZone(ROLLER_I_ZONE);
		config.closedLoop.feedForward.kV(ROLLER_KV);

		rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		telemetry.log();
		updatePivotPIDIfChanged();
		updateRollerPIDIfChanged();
		// Step the profiler and feed the intermediate position to the SparkMax
		double profiledSetpoint = pivotProfiler.calculate(pivotEncoder.getPosition());
		pivotController.setSetpoint(pivotProfiler.getSetpoint().position, ControlType.kPosition);
		DogLog.log("Intake/Pivot/ProfiledSetpoint", profiledSetpoint);
		DogLog.log("Intake/Pivot/ProfiledVelocity", pivotProfiler.getSetpoint().velocity);
	}

	private void updatePivotPIDIfChanged() {
		double kP = tunePivotKP.getAsDouble(), kI = tunePivotKI.getAsDouble(),
				kD = tunePivotKD.getAsDouble(), kV = tunePivotKV.getAsDouble();
		if (kP == prevPivotKP && kI == prevPivotKI && kD == prevPivotKD && kV == prevPivotKV)
			return;
		prevPivotKP = kP;
		prevPivotKI = kI;
		prevPivotKD = kD;
		prevPivotKV = kV;
		SparkMaxConfig config = new SparkMaxConfig();
		config.closedLoop.pid(kP, kI, kD);
		config.closedLoop.feedForward.kV(kV);
		pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void updateRollerPIDIfChanged() {
		double kP = tuneRollerKP.getAsDouble(), kI = tuneRollerKI.getAsDouble(),
				kD = tuneRollerKD.getAsDouble(), kV = tuneRollerKV.getAsDouble();
		if (kP == prevRollerKP && kI == prevRollerKI && kD == prevRollerKD && kV == prevRollerKV)
			return;
		prevRollerKP = kP;
		prevRollerKI = kI;
		prevRollerKD = kD;
		prevRollerKV = kV;
		SparkMaxConfig config = new SparkMaxConfig();
		config.closedLoop.pid(kP, kI, kD);
		config.closedLoop.feedForward.kV(kV);
		rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	// ==================== State Query Methods ====================

	public boolean isPivotAtPosition() {
		boolean atPositionRaw = Math.abs(pivotEncoder.getPosition() - targetPivotAngle.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
				.in(Degrees);
		return atPositionDebouncer.calculate(atPositionRaw);
	}

	public boolean isRollerAtSpeed() {
		return Math.abs(rollerEncoder.getVelocity()
				- targetRollerVelocity.in(RPM)) < ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	public boolean isExtended() {
		return isPivotAtPosition()
				&& Math.abs(targetPivotAngle.in(Degrees) - PIVOT_EXTENDED_ANGLE.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
						.in(Degrees);
	}

	public boolean isRetracted() {
		return isPivotAtPosition()
				&& Math.abs(targetPivotAngle.in(Degrees) - PIVOT_RETRACTED_ANGLE.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
						.in(Degrees);
	}

	// ==================== Control Methods ====================

	public void setPivotAngle(Angle angle) {
		targetPivotAngle = angle;
		pivotProfiler.setGoal(angle.in(Degrees));
	}

	public void setRollerVelocity(AngularVelocity velocity) {
		targetRollerVelocity = velocity;
		rollerController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public void stopRoller() {
		setRollerVelocity(RPM.of(0));
	}

	public void stop() {
		stopRoller();
		pivotMotor.stopMotor();
	}

	// ==================== Command Factory Methods ====================

	public Command extendCommand() {
		return runOnce(() -> setPivotAngle(PIVOT_EXTENDED_ANGLE))
				.andThen(idle().until(() -> isPivotStalled() || isPivotAtPosition()))
				.andThen(runOnce(() -> setPivotAngle(Degrees.of(pivotEncoder.getPosition()))))
				.withName("Intake Extend");
	}

	public Command retractCommand() {
		return runOnce(() -> setPivotAngle(PIVOT_RETRACTED_ANGLE))
				.andThen(idle().until(() -> isPivotStalled() || isPivotAtPosition()))
				.andThen(runOnce(() -> setPivotAngle(Degrees.of(pivotEncoder.getPosition()))))
				.withName("Intake Retract");
	}

	public Command runRollerCommand() {
		return run(() -> setRollerVelocity(ROLLER_INTAKE_VELOCITY))
				.finallyDo(() -> setRollerVelocity(RPM.of(0))).withName("Intake Run Roller");
	}

	public Command ejectCommand() {
		return run(() -> setRollerVelocity(ROLLER_EJECT_VELOCITY))
				.finallyDo(() -> setRollerVelocity(RPM.of(0))).withName("Intake Eject");
	}

	public Command stopRollerCommand() {
		return runOnce(this::stopRoller).withName("Intake Stop Roller");
	}

	public Command intakeCommand() {
		return runOnce(() -> setRollerVelocity(ROLLER_INTAKE_VELOCITY))
				.andThen(Commands.idle(this))
				.finallyDo(this::stopRoller)
				.withName("Intake Full Sequence");
	}

	public Command stowCommand() {
		return sequence(stopRollerCommand(), retractCommand()).withName("Intake Stow");
	}

	/** Extends or stows depending on current position. */
	public Command stowToggleCommand() {
		return Commands.either(stowCommand(), extendCommand(), this::isExtended)
				.withName("Intake Stow Toggle");
	}

	/** Extends and runs roller until toggled off, then stows. */
	public Command intakeToggleCommand() {
		return Commands.sequence(extendCommand(), runRollerCommand(), Commands.idle(this))
				.finallyDo(interrupted -> CommandScheduler.getInstance().schedule(stowCommand()))
				.withName("Intake Toggle");
	}

	// ==================== Test Mode ====================

	public Command testPivotCommand() {
		return Commands.run(() -> {
			setPivotAngle(Degrees.of(testPivotDeg.get()));
		}, this)
				.finallyDo(pivotMotor::stopMotor)
				.withName("Test Intake Pivot");
	}

	public Command testRollerCommand() {
		return Commands.run(() -> setRollerVelocity(RPM.of(testRollerRPM.get())), this)
				.finallyDo(this::stopRoller)
				.withName("Test Intake Roller");
	}

	// ==================== Stall Detection ====================

	public boolean isPivotStalled() {
		double pivotMotorCurrent = pivotMotor.getOutputCurrent();
		double pivotMotorRPM = pivotMotor.getEncoder().getVelocity(); // RPM
		boolean isPivotStalled = Math.abs(pivotMotorRPM) < 2.0 && pivotMotorCurrent > PIVOT_CURRENT_LIMIT * 0.5;
		return stallDebouncer.calculate(isPivotStalled);
	}
}
