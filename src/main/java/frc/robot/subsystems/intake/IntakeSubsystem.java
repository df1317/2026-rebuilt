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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Intake subsystem with a pivoting arm and roller mechanism.
 */
public class IntakeSubsystem extends SubsystemBase {

	// ==================== Hardware ====================
	private final SparkMax pivotMotor;
	private final SparkMax rollerMotor;
	private final SparkClosedLoopController pivotController;
	private final SparkClosedLoopController rollerController;
	private final RelativeEncoder pivotEncoder;
	private final RelativeEncoder rollerEncoder;

	// ==================== Control State ====================
	private Angle targetPivotAngle = PIVOT_RETRACTED_ANGLE;
	private AngularVelocity targetRollerVelocity = RPM.of(0);
	private final Debouncer atPositionDebouncer;

	// ==================== Visualization & Telemetry ====================
	private final IntakeVisualization visualization;
	private final IntakeTelemetry telemetry;

	public IntakeSubsystem() {
		pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
		rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
		pivotController = pivotMotor.getClosedLoopController();
		rollerController = rollerMotor.getClosedLoopController();
		pivotEncoder = pivotMotor.getEncoder();
		rollerEncoder = rollerMotor.getEncoder();

		configurePivotMotor();
		configureRollerMotor();

		atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_TIME, DebounceType.kRising);

		visualization = new IntakeVisualization();
		telemetry = new IntakeTelemetry(pivotMotor, rollerMotor);
	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(PIVOT_CURRENT_LIMIT)
				.inverted(PIVOT_INVERTED);
		config.encoder
				// Converts encoder rotations to degrees: (360 deg/rot) / gear_ratio
				.positionConversionFactor(360.0 / PIVOT_GEAR_RATIO);
		config.closedLoop
				.pid(PIVOT_KP, PIVOT_KI, PIVOT_KD)
				.allowedClosedLoopError(PIVOT_ANGLE_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureRollerMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kCoast)
				.smartCurrentLimit(ROLLER_CURRENT_LIMIT)
				.inverted(ROLLER_INVERTED);
		config.closedLoop
				.pid(ROLLER_KP, ROLLER_KI, ROLLER_KD);
		config.closedLoop.feedForward
				.kV(ROLLER_KV);

		rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		visualization.update(
				getPivotAngle().in(Degrees),
				targetPivotAngle.in(Degrees),
				isPivotAtPosition(),
				targetRollerVelocity,
				isRollerAtSpeed());

		telemetry.log(
				getPivotAngle().in(Degrees),
				targetPivotAngle.in(Degrees),
				isPivotAtPosition(),
				isExtended(),
				isRetracted(),
				getRollerVelocity().in(RPM),
				targetRollerVelocity.in(RPM),
				isRollerRunning(),
				isRollerAtSpeed(),
				getStatusColor());
	}

	// ==================== State Query Methods ====================

	public Angle getPivotAngle() {
		return Degrees.of(pivotEncoder.getPosition());
	}

	public AngularVelocity getRollerVelocity() {
		return RPM.of(rollerEncoder.getVelocity());
	}

	public boolean isPivotAtPositionRaw() {
		return Math.abs(getPivotAngle().in(Degrees) - targetPivotAngle.in(Degrees)) < PIVOT_ANGLE_TOLERANCE.in(Degrees);
	}

	public boolean isPivotAtPosition() {
		return atPositionDebouncer.calculate(isPivotAtPositionRaw());
	}

	public boolean isExtended() {
		return isPivotAtPosition()
				&& Math.abs(targetPivotAngle.in(Degrees) - PIVOT_EXTENDED_ANGLE.in(Degrees)) < 1.0;
	}

	public boolean isRetracted() {
		return isPivotAtPosition()
				&& Math.abs(targetPivotAngle.in(Degrees) - PIVOT_RETRACTED_ANGLE.in(Degrees)) < 1.0;
	}

	public boolean isRollerRunning() {
		return Math.abs(targetRollerVelocity.in(RPM)) > ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	public boolean isRollerAtSpeed() {
		return Math.abs(getRollerVelocity().in(RPM) - targetRollerVelocity.in(RPM)) < ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	public Color getStatusColor() {
		if (targetRollerVelocity.in(RPM) > 0 && isExtended()) {
			return isRollerAtSpeed() ? Color.kGreen : Color.kYellow;
		} else if (targetRollerVelocity.in(RPM) < 0) {
			return Color.kOrange;
		} else if (!isRetracted()) {
			return Color.kYellow;
		}
		return Color.kRed;
	}

	// ==================== Control Methods ====================

	public void setPivotAngle(Angle angle) {
		targetPivotAngle = angle;
		pivotController.setSetpoint(angle.in(Degrees), ControlType.kPosition);
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
				.withName("Intake Extend");
	}

	public Command retractCommand() {
		return runOnce(() -> setPivotAngle(PIVOT_RETRACTED_ANGLE))
				.withName("Intake Retract");
	}

	public Command runRollerCommand() {
		return runOnce(() -> setRollerVelocity(ROLLER_INTAKE_VELOCITY))
				.withName("Intake Run Roller");
	}

	public Command ejectCommand() {
		return runOnce(() -> setRollerVelocity(ROLLER_EJECT_VELOCITY))
				.withName("Intake Eject");
	}

	public Command stopRollerCommand() {
		return runOnce(this::stopRoller)
				.withName("Intake Stop Roller");
	}

	public Command intakeCommand() {
		return Commands.sequence(
				extendCommand(),
				Commands.waitUntil(this::isPivotAtPosition),
				runRollerCommand())
				.withName("Intake Full Sequence");
	}

	public Command stowCommand() {
		return Commands.sequence(
				stopRollerCommand(),
				retractCommand())
				.withName("Intake Stow");
	}
}
