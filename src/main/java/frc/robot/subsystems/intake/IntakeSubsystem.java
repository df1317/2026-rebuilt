package frc.robot.subsystems.intake;

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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Intake pivot subsystem. The roller is a separate {@link RollerSubsystem} so they can
 * run commands independently (e.g. roller keeps spinning while the pivot retracts).
 */
public class IntakeSubsystem extends SubsystemBase {

	final SparkMax pivotMotor;
	final RelativeEncoder pivotEncoder;
	private final SparkClosedLoopController pivotController;
	private final Debouncer atPositionDebouncer;
	private final Debouncer stallDebouncer = new Debouncer(0.1, DebounceType.kBoth);
	private final DoubleSubscriber testPivotDeg = DogLog.tunable("Intake/Pivot/Degrees",
			PIVOT_EXTENDED_ANGLE.in(Degrees), Degrees);
	private final IntakeTelemetry telemetry;
	// Pivot PID tunables
	private final DoubleSubscriber tunePivotKP = DogLog.tunable("Intake/Pivot/kP", PIVOT_KP);
	private final DoubleSubscriber tunePivotKI = DogLog.tunable("Intake/Pivot/kI", PIVOT_KI);
	private final DoubleSubscriber tunePivotKD = DogLog.tunable("Intake/Pivot/kD", PIVOT_KD);
	private final DoubleSubscriber tunePivotKV = DogLog.tunable("Intake/Pivot/kV", 0.0);
	Angle targetPivotAngle = PIVOT_RETRACTED_ANGLE;
	boolean homed = false;
	private double prevPivotKP = PIVOT_KP, prevPivotKI = PIVOT_KI, prevPivotKD = PIVOT_KD, prevPivotKV = 0.0;
	private final ProfiledPIDController pivotProfiler = new ProfiledPIDController(0, 0, 0,
			new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY_DEG_PER_S, PIVOT_MAX_ACCEL_DEG_PER_S2));

	private final RollerSubsystem roller;

	public IntakeSubsystem(RollerSubsystem roller) {
		this.roller = roller;
		pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
		pivotController = pivotMotor.getClosedLoopController();
		pivotEncoder = pivotMotor.getEncoder();

		configurePivotMotor();

		atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_TIME, DebounceType.kRising);

		telemetry = new IntakeTelemetry(this, roller);

		double initialAngle = pivotEncoder.getPosition();
		pivotProfiler.reset(initialAngle);
		setPivotAngle(Degrees.of(initialAngle));
	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit(PIVOT_CURRENT_LIMIT)
				.inverted(PIVOT_INVERTED);
		config.encoder
				.positionConversionFactor(360.0 / PIVOT_GEAR_RATIO);
		config.closedLoop.pid(PIVOT_KP, PIVOT_KI, PIVOT_KD)
				.allowedClosedLoopError(PIVOT_ANGLE_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		telemetry.log();
		updatePivotPIDIfChanged();
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

	// ==================== State Query Methods ====================

	public boolean isPivotAtPosition() {
		boolean atPositionRaw = Math.abs(pivotEncoder.getPosition() - targetPivotAngle.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
				.in(Degrees);
		return atPositionDebouncer.calculate(atPositionRaw);
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

	public void stop() {
		pivotMotor.stopMotor();
	}

	// ==================== Command Factory Methods ====================

	public Command extendCommand() {
		return runOnce(() -> {
			pivotProfiler.setConstraints(new TrapezoidProfile.Constraints(
					PIVOT_EXTEND_MAX_VELOCITY_DEG_PER_S, PIVOT_EXTEND_MAX_ACCEL_DEG_PER_S2));
			setPivotAngle(PIVOT_EXTENDED_ANGLE);
		})
				.andThen(idle().until(() -> isPivotStalled() || isPivotAtPosition()))
				.andThen(runOnce(() -> setPivotAngle(Degrees.of(pivotEncoder.getPosition()))))
				.finallyDo(() -> pivotProfiler.setConstraints(new TrapezoidProfile.Constraints(
						PIVOT_MAX_VELOCITY_DEG_PER_S, PIVOT_MAX_ACCEL_DEG_PER_S2)))
				.withName("Intake Extend");
	}

	public Command retractCommand() {
		return runOnce(() -> setPivotAngle(PIVOT_RETRACTED_ANGLE))
				.andThen(idle().until(() -> isPivotStalled() || isPivotAtPosition()))
				.andThen(runOnce(() -> setPivotAngle(Degrees.of(pivotEncoder.getPosition()))))
				.withName("Intake Retract");
	}

	public Command stowCommand() {
		return retractCommand().withName("Intake Stow");
	}

	/** Extends or stows depending on current position. */
	public Command stowToggleCommand() {
		return Commands.either(stowCommand(), extendCommand(), this::isExtended)
				.withName("Intake Stow Toggle");
	}

	public Command holdExtendedCommand() {
		return run(() -> setPivotAngle(PIVOT_EXTENDED_ANGLE))
				.withName("Hold Extended");
	}

	// ==================== Test Mode ====================

	public Command homeCommand() {
		return runOnce(() -> {
			pivotProfiler.setConstraints(new TrapezoidProfile.Constraints(
					PIVOT_MAX_VELOCITY_DEG_PER_S, PIVOT_MAX_ACCEL_DEG_PER_S2));
			setPivotAngle(Degrees.of(pivotEncoder.getPosition() + PIVOT_HOMING_OFFSET_DEG));
		})
				.andThen(idle().until(this::isPivotStalled).withTimeout(5.0))
				.finallyDo(() -> {
					pivotMotor.stopMotor();
					pivotEncoder.setPosition(0);
					pivotProfiler.reset(0);
					setPivotAngle(Degrees.of(0));
					homed = true;
				})
				.withName("Home Intake");
	}

	public Command zeroCommand() {
		return runOnce(() -> {
			pivotEncoder.setPosition(0);
			pivotProfiler.reset(0);
			setPivotAngle(Degrees.of(0));
		}).withName("Intake Zero");
	}

	public Command testPivotCommand() {
		return Commands.run(() -> {
			setPivotAngle(Degrees.of(testPivotDeg.get()));
		}, this)
				.finallyDo(pivotMotor::stopMotor)
				.withName("Test Intake Pivot");
	}

	// ==================== Stall Detection ====================

	public boolean isPivotStalled() {
		double pivotMotorCurrent = pivotMotor.getOutputCurrent();
		double pivotMotorRPM = pivotMotor.getEncoder().getVelocity();
		boolean isPivotStalled = Math.abs(pivotMotorRPM) < 2.0 && pivotMotorCurrent > PIVOT_CURRENT_LIMIT * 0.5;
		return stallDebouncer.calculate(isPivotStalled);
	}
}
