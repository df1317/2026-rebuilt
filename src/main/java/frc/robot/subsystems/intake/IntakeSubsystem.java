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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandBuilder;
import frc.robot.util.Mutable;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Intake pivot subsystem. The roller is a separate {@link RollerSubsystem} so they can run commands independently (e.g.
 * roller keeps spinning while the pivot retracts).
 */
public class IntakeSubsystem extends SubsystemBase {

	// ==================== Hardware ====================
	final SparkMax pivotMotor;
	final RelativeEncoder pivotEncoder;
	private final SparkClosedLoopController pivotController;
	private final Debouncer atPositionDebouncer;
	private final Debouncer stallDebouncer = new Debouncer(1.5, DebounceType.kBoth);

	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Intake");
	private static final TunableTable pivotTunables = tunables.getNested("Pivot");
	private final TunableDouble testPivotDeg = pivotTunables.value("Degrees", PIVOT_EXTENDED_ANGLE.in(Degrees), Degrees);
	private final TunableDouble retractDelta = pivotTunables.value("RetractDelta", PIVOT_RETRACTED_DELTA.in(Degrees),
			Degrees);
	private final TunableDouble fastVelocity = pivotTunables.value("FastVelDegPerS", PIVOT_MAX_VELOCITY_DEG_PER_S);
	private final TunableDouble fastAccel = pivotTunables.value("FastAccelDegPerS2", PIVOT_MAX_ACCEL_DEG_PER_S2);
	private final TunableDouble slowVelocity = pivotTunables.value("SlowVelDegPerS", PIVOT_EXTEND_MAX_VELOCITY_DEG_PER_S);
	private final TunableDouble slowAccel = pivotTunables.value("SlowAccelDegPerS2", PIVOT_EXTEND_MAX_ACCEL_DEG_PER_S2);
	private final TunableDouble kickDurationS = pivotTunables.value("KickDurationS", 0.5);
	private final TunableDouble homingOffset = pivotTunables.value("HomingOffsetDeg", PIVOT_HOMING_OFFSET_DEG);

	// ==================== Telemetry ====================
	private final IntakeTelemetry telemetry;

	// ==================== Control State ====================
	private final ProfiledPIDController pivotProfiler = new ProfiledPIDController(0, 0, 0,
			new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY_DEG_PER_S, PIVOT_MAX_ACCEL_DEG_PER_S2));
	private final RollerSubsystem roller;
	boolean homed = false;
	private boolean wantToExtend = false;
	private Angle extendedPivotAngle = PIVOT_EXTENDED_ANGLE;
	Angle targetPivotAngle = extendedPivotAngle.plus(PIVOT_RETRACTED_DELTA);

	public IntakeSubsystem(RollerSubsystem roller) {
		this.roller = roller;
		pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
		pivotController = pivotMotor.getClosedLoopController();
		pivotEncoder = pivotMotor.getEncoder();

		configurePivotMotor();

		atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_TIME, DebounceType.kRising);

		telemetry = new IntakeTelemetry(this, roller);

		tunables.pidSpark("Pivot", pivotMotor, PIVOT_KP, PIVOT_KI, PIVOT_KD, 0.0);

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

	private boolean wasEnabled = false;

	@Override
	public void periodic() {
		boolean enabled = edu.wpi.first.wpilibj.DriverStation.isEnabled();
		if (enabled && !wasEnabled) {
			double pos = pivotEncoder.getPosition();
			pivotProfiler.reset(pos);
			setPivotAngle(Degrees.of(pos));
			double distToExtended = Math.abs(pos - extendedPivotAngle.in(Degrees));
			double distToRetracted = Math.abs(pos - retractedAngleDeg());
			wantToExtend = distToExtended < distToRetracted;
		}
		wasEnabled = enabled;

		telemetry.log();
		double profiledSetpoint = pivotProfiler.calculate(pivotEncoder.getPosition());
		pivotController.setSetpoint(pivotProfiler.getSetpoint().position, ControlType.kPosition);
		DogLog.log("Intake/Pivot/ProfiledSetpoint", profiledSetpoint);
		DogLog.log("Intake/Pivot/ProfiledVelocity", pivotProfiler.getSetpoint().velocity);
	}

	// ==================== Helpers ====================

	private TrapezoidProfile.Constraints fastConstraints() {
		return new TrapezoidProfile.Constraints(fastVelocity.get(), fastAccel.get());
	}

	private TrapezoidProfile.Constraints slowConstraints() {
		return new TrapezoidProfile.Constraints(slowVelocity.get(), slowAccel.get());
	}

	private double retractedAngleDeg() {
		return extendedPivotAngle.in(Degrees) + retractDelta.get();
	}

	// ==================== State Query Methods ====================

	public boolean isPivotAtPosition() {
		boolean atPositionRaw = Math.abs(pivotEncoder.getPosition() - targetPivotAngle.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
				.in(Degrees);
		return atPositionDebouncer.calculate(atPositionRaw);
	}

	public boolean isExtended() {
		return Math.abs(pivotEncoder.getPosition() - extendedPivotAngle.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
				.in(Degrees);
	}

	public boolean isRetracted() {
		return Math.abs(pivotEncoder.getPosition() - retractedAngleDeg()) < PIVOT_ANGLE_TOLERANCE.in(Degrees);
	}

	// ==================== Control Methods ====================

	public void setPivotAngle(Angle angle) {
		targetPivotAngle = angle;
		pivotProfiler.setGoal(angle.in(Degrees));
	}

	// ==================== Command Factory Methods ====================

	/**
	 * Extends the intake using a two-phase motion profile:
	 * KICK phase pushes through the panel at full speed, then GENTLE phase
	 * slows down for a controlled landing.
	 */
	public Command extendCommand() {
		enum Phase{KICK,GENTLE}
		Mutable<Phase> phase = new Mutable<>(Phase.KICK);
		Timer kickTimer = new Timer();

		return new CommandBuilder("Intake.extend", this)
				.onInitialize(() -> {
					phase.value = Phase.KICK;
					kickTimer.restart();
					pivotProfiler.setConstraints(fastConstraints());
					setPivotAngle(extendedPivotAngle);
					wantToExtend = true;
				})
				.onExecute(() -> {
					if (phase.value == Phase.KICK && kickTimer.hasElapsed(kickDurationS.get())) {
						pivotProfiler.setConstraints(slowConstraints());
						phase.value = Phase.GENTLE;
					}
				})
				.isFinished(() -> phase.value == Phase.GENTLE && (isPivotStalled() || isPivotAtPosition()))
				.onEnd(() -> {
					setPivotAngle(Degrees.of(pivotEncoder.getPosition()));
					pivotProfiler.setConstraints(fastConstraints());
				});
	}

	public Command retractCommand() {
		return new CommandBuilder("Intake.retract", this)
				.onInitialize(() -> {
					setPivotAngle(Degrees.of(retractedAngleDeg()));
					wantToExtend = false;
				})
				.isFinished(() -> isPivotStalled() || isPivotAtPosition())
				.onEnd(() -> setPivotAngle(Degrees.of(pivotEncoder.getPosition())));
	}

	public Command stowCommand() {
		return retractCommand().withName("Intake.stow");
	}

	public Command jogDownCommand() {
		return new CommandBuilder("Intake.jogDown", this)
				.onExecute(() -> pivotMotor.set(-0.1))
				.onEnd(() -> {
					pivotMotor.stopMotor();
					double pos = pivotEncoder.getPosition();
					pivotProfiler.reset(pos);
					setPivotAngle(Degrees.of(pos));
				});
	}

	public Command zeroIntakeCommand() {
		return runOnce(() -> {
			double currentAngle = pivotEncoder.getPosition();
			extendedPivotAngle = Degrees.of(currentAngle);
			pivotProfiler.reset(currentAngle);
			setPivotAngle(extendedPivotAngle);
			homed = true;
		}).withName("Intake.zero");
	}

	/** Extends or stows depending on current position. */
	public Command stowToggleCommand() {
		return Commands.either(stowCommand(), extendCommand(), () -> wantToExtend)
				.withName("Intake.toggle");
	}

	public Command holdExtendedCommand() {
		return new CommandBuilder("Intake.holdExtended", this)
				.onExecute(() -> {
					if (!isRetracted()) {
						setPivotAngle(extendedPivotAngle);
					}
				});
	}

	// ==================== Homing & Test ====================

	/** Drives toward the hard stop, zeroes on stall. */
	public Command homeCommand() {
		return new CommandBuilder("Intake.home", this)
				.onInitialize(() -> {
					pivotProfiler.setConstraints(fastConstraints());
					setPivotAngle(Degrees.of(pivotEncoder.getPosition() + homingOffset.get()));
				})
				.isFinished(this::isPivotStalled)
				.onEnd(() -> {
					pivotMotor.stopMotor();
					pivotEncoder.setPosition(0);
					pivotProfiler.reset(0);
					setPivotAngle(Degrees.of(0));
					homed = true;
				});
	}

	public Command testPivotCommand() {
		return new CommandBuilder("Intake.testPivot", this)
				.onExecute(() -> setPivotAngle(Degrees.of(testPivotDeg.get())))
				.onEnd(() -> pivotMotor.stopMotor());
	}

	// ==================== Stall Detection ====================

	public boolean isPivotStalled() {
		double pivotMotorCurrent = pivotMotor.getOutputCurrent();
		double pivotMotorRPM = pivotMotor.getEncoder().getVelocity();
		boolean isPivotStalled = Math.abs(pivotMotorRPM) < 2.0 && pivotMotorCurrent > PIVOT_CURRENT_LIMIT * 0.75;
		return stallDebouncer.calculate(isPivotStalled);
	}
}
