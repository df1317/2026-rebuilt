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

/**
 * Intake pivot subsystem. The roller is a separate {@link RollerSubsystem} so they can run commands independently (e.g.
 * roller keeps spinning while the pivot retracts).
 */
public class IntakeSubsystem extends SubsystemBase {

	// ==================== Hardware Config ====================
	private static final int PIVOT_MOTOR_ID = 25;
	private static final int PIVOT_CURRENT_LIMIT = 35;
	private static final boolean PIVOT_INVERTED = false;
	private static final double PIVOT_GEAR_RATIO = (48.0 * 22.0) / 14.0;
	private static final Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(8);
	private static final double STALL_DEBOUNCE_S = 1.5;
	private static final double AT_POSITION_DEBOUNCE_S = 0.1;
	private static final double STALL_RPM_THRESHOLD = 2.0;
	private static final double STALL_CURRENT_RATIO = 0.75;
	private static final double JOG_DOWN_SPEED = -0.1;

	// ==================== PID Gains ====================
	private static final double PIVOT_KP = 0.05;
	private static final double PIVOT_KI = 0.0;
	private static final double PIVOT_KD = 0.0;

	// ==================== Default Tunable Values ====================
	private static final double DEFAULT_EXTENDED_ANGLE_DEG = 14.0;
	private static final double DEFAULT_RETRACT_DELTA_DEG = 90.0;
	private static final double DEFAULT_FAST_VELOCITY = 240.0;
	private static final double DEFAULT_FAST_ACCEL = 240.0;
	private static final double DEFAULT_SLOW_VELOCITY = 60.0;
	private static final double DEFAULT_SLOW_ACCEL = 180.0;
	private static final double DEFAULT_KICK_DURATION_S = 0.5;
	private static final double DEFAULT_HOMING_OFFSET_DEG = -20.0;
	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Intake");
	private static final TunableTable pivotTunables = tunables.getNested("Pivot");
	// ==================== Hardware ====================
	final SparkMax pivotMotor;
	final RelativeEncoder pivotEncoder;
	private final SparkClosedLoopController pivotController;
	private final Debouncer atPositionDebouncer;
	private final Debouncer stallDebouncer = new Debouncer(STALL_DEBOUNCE_S, DebounceType.kBoth);
	private final TunableDouble testPivotDeg = pivotTunables.value("Degrees", DEFAULT_EXTENDED_ANGLE_DEG, Degrees);
	private final TunableDouble retractDelta = pivotTunables.value("RetractDelta", DEFAULT_RETRACT_DELTA_DEG, Degrees);
	private final TunableDouble fastVelocity = pivotTunables.value("FastVelDegPerS", DEFAULT_FAST_VELOCITY);
	private final TunableDouble fastAccel = pivotTunables.value("FastAccelDegPerS2", DEFAULT_FAST_ACCEL);
	private final TunableDouble slowVelocity = pivotTunables.value("SlowVelDegPerS", DEFAULT_SLOW_VELOCITY);
	private final TunableDouble slowAccel = pivotTunables.value("SlowAccelDegPerS2", DEFAULT_SLOW_ACCEL);
	private final TunableDouble kickDurationS = pivotTunables.value("KickDurationS", DEFAULT_KICK_DURATION_S);
	private final TunableDouble homingOffset = pivotTunables.value("HomingOffsetDeg", DEFAULT_HOMING_OFFSET_DEG);

	// ==================== Telemetry ====================
	private final IntakeTelemetry telemetry;

	public enum IntakeState {
		STOWED, EXTENDING_KICK, EXTENDING_GENTLE, EXTENDED, JOGGING_UP, JOGGING_DOWN, HOMING, TEST, UNKNOWN
	}

	private IntakeState currentState = IntakeState.UNKNOWN;
	private final Timer stateTimer = new Timer();

	public void setState(IntakeState newState) {
		if (currentState != newState) {
			currentState = newState;
			stateTimer.restart();
		}
	}
	private final ProfiledPIDController pivotProfiler = new ProfiledPIDController(0, 0, 0,
			new TrapezoidProfile.Constraints(DEFAULT_FAST_VELOCITY, DEFAULT_FAST_ACCEL));
	private final RollerSubsystem roller;
	boolean homed = false;
	private boolean wantToExtend = false;
	private Angle extendedPivotAngle = Degrees.of(DEFAULT_EXTENDED_ANGLE_DEG);
	Angle targetPivotAngle = extendedPivotAngle.plus(Degrees.of(DEFAULT_RETRACT_DELTA_DEG));
	private boolean wasEnabled = false;

	public IntakeSubsystem(RollerSubsystem roller) {
		this.roller = roller;
		pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
		pivotController = pivotMotor.getClosedLoopController();
		pivotEncoder = pivotMotor.getEncoder();

		configurePivotMotor();

		atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_S, DebounceType.kRising);

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
			if (wantToExtend) {
				setState(IntakeState.EXTENDED);
			} else {
				setState(IntakeState.STOWED);
			}
		}
		wasEnabled = enabled;

		switch (currentState) {
			case EXTENDING_KICK:
				pivotProfiler.setConstraints(fastConstraints());
				setPivotAngle(extendedPivotAngle);
				if (stateTimer.hasElapsed(kickDurationS.get())) {
					setState(IntakeState.EXTENDING_GENTLE);
				}
				break;
			case EXTENDING_GENTLE:
				pivotProfiler.setConstraints(slowConstraints());
				setPivotAngle(extendedPivotAngle);
				if (isPivotStalled() || isPivotAtPosition()) {
					setState(IntakeState.EXTENDED);
				}
				break;
			case EXTENDED:
				pivotProfiler.setConstraints(fastConstraints());
				setPivotAngle(extendedPivotAngle);
				break;
			case STOWED:
				pivotProfiler.setConstraints(fastConstraints());
				setPivotAngle(Degrees.of(retractedAngleDeg()));
				break;
			case JOGGING_UP:
				pivotProfiler.setConstraints(slowConstraints());
				setPivotAngle(Degrees.of(pivotEncoder.getPosition() + 360.0));
				break;
			case JOGGING_DOWN:
				pivotProfiler.setConstraints(slowConstraints());
				setPivotAngle(Degrees.of(pivotEncoder.getPosition() - 360.0));
				break;
			case HOMING:
				pivotProfiler.setConstraints(fastConstraints());
				setPivotAngle(Degrees.of(pivotEncoder.getPosition() + homingOffset.get()));
				break;
			case TEST:
				pivotProfiler.setConstraints(fastConstraints());
				setPivotAngle(Degrees.of(testPivotDeg.get()));
				break;
			case UNKNOWN:
				break;
		}

		telemetry.log();
		DogLog.log("Intake/State", currentState.name());
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
	 * Extends the intake using a two-phase motion profile: KICK phase pushes through the panel at full speed, then GENTLE
	 * phase slows down for a controlled landing.
	 */
	public Command extendCommand() {
		return new CommandBuilder("Intake.extend", this)
				.onInitialize(() -> {
					wantToExtend = true;
					setState(IntakeState.EXTENDING_KICK);
				})
				.isFinished(() -> currentState == IntakeState.EXTENDED);
	}

	public Command retractCommand() {
		return new CommandBuilder("Intake.retract", this)
				.onInitialize(() -> {
					wantToExtend = false;
					setState(IntakeState.STOWED);
				})
				.isFinished(() -> isPivotStalled() || isPivotAtPosition());
	}

	public Command stowCommand() {
		return retractCommand().withName("Intake.stow");
	}

	public Command jogDownCommand() {
		return new CommandBuilder("Intake.jogDown", this)
				.onInitialize(() -> setState(IntakeState.JOGGING_DOWN))
				.onEnd(() -> {
					double pos = pivotEncoder.getPosition();
					pivotProfiler.reset(pos);
					setPivotAngle(Degrees.of(pos));
					setState(IntakeState.UNKNOWN);
				});
	}

	public Command jogUpCommand() {
		return new CommandBuilder("Intake.jogUp", this)
				.onInitialize(() -> setState(IntakeState.JOGGING_UP))
				.onEnd(() -> {
					double pos = pivotEncoder.getPosition();
					pivotProfiler.reset(pos);
					setPivotAngle(Degrees.of(pos));
					setState(IntakeState.UNKNOWN);
				});
	}

	public Command zeroIntakeCommand() {
		return runOnce(() -> {
			double currentAngle = pivotEncoder.getPosition();
			extendedPivotAngle = Degrees.of(currentAngle - retractDelta.get());
			pivotProfiler.reset(currentAngle);
			setPivotAngle(Degrees.of(currentAngle));
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
				.onInitialize(() -> setState(IntakeState.HOMING))
				.isFinished(this::isPivotStalled)
				.onEnd(() -> {
					pivotMotor.stopMotor();
					pivotEncoder.setPosition(0);
					pivotProfiler.reset(0);
					setPivotAngle(Degrees.of(0));
					homed = true;
					setState(IntakeState.UNKNOWN);
				});
	}

	public Command testPivotCommand() {
		return new CommandBuilder("Intake.testPivot", this)
				.onInitialize(() -> setState(IntakeState.TEST))
				.onEnd(() -> {
					pivotMotor.stopMotor();
					setState(IntakeState.UNKNOWN);
				});
	}

	// ==================== Stall Detection ====================

	public boolean isPivotStalled() {
		double pivotMotorCurrent = pivotMotor.getOutputCurrent();
		double pivotMotorRPM = pivotMotor.getEncoder().getVelocity();
		boolean isPivotStalled = Math.abs(pivotMotorRPM) < STALL_RPM_THRESHOLD
				&& pivotMotorCurrent > PIVOT_CURRENT_LIMIT * STALL_CURRENT_RATIO;
		return stallDebouncer.calculate(isPivotStalled);
	}
}
