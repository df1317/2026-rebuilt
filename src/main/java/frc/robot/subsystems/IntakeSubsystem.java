package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

	// Hardware
	private final SparkMax pivotMotor;
	private final SparkMax rollerMotor;
	private final SparkClosedLoopController pivotController;
	private final SparkClosedLoopController rollerController;
	private final RelativeEncoder pivotEncoder;
	private final RelativeEncoder rollerEncoder;

	// Control state
	private Angle targetPivotAngle = PIVOT_RETRACTED_ANGLE;
	private AngularVelocity targetRollerVelocity = RPM.of(0);
	private final Debouncer atPositionDebouncer;

	// Mechanism2d visualization
	private static final double VIZ_WIDTH = 0.6;
	private static final double VIZ_HEIGHT = 0.6;
	private final Mechanism2d mechanism2d;
	private final MechanismLigament2d pivotLigament;
	private final MechanismLigament2d setpointLigament;
	private final MechanismLigament2d rollerLigament;

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

		// Setup Mechanism2d visualization
		mechanism2d = new Mechanism2d(VIZ_WIDTH, VIZ_HEIGHT);

		MechanismRoot2d root = mechanism2d.getRoot("IntakeRoot", VIZ_WIDTH / 2, 0.1);

		// Setpoint indicator (thin white line showing target angle)
		setpointLigament = root.append(
				new MechanismLigament2d("Setpoint", PIVOT_ARM_LENGTH.in(Meters), 0, 2, new Color8Bit(Color.kWhite)));

		// Main pivot arm (thick, color changes based on state)
		pivotLigament = root.append(
				new MechanismLigament2d("Pivot", PIVOT_ARM_LENGTH.in(Meters), 0, 6, new Color8Bit(Color.kCyan)));

		// Roller indicator at end of pivot (shows roller activity)
		rollerLigament = pivotLigament.append(
				new MechanismLigament2d("Roller", 0.05, 90, 8, new Color8Bit(Color.kGray)));

		SmartDashboard.putData("Intake/Mechanism", mechanism2d);
	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(PIVOT_CURRENT_LIMIT)
				.inverted(PIVOT_INVERTED);
		config.encoder
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
		updateVisualization();

		// Status for LED strip
		DogLog.forceNt.log("Intake/Status", getStatusColor().toHexString());

		// Pivot tracking
		DogLog.log("Intake/PivotAngleDeg", getPivotAngle().in(Degrees));
		DogLog.log("Intake/PivotTargetAngleDeg", targetPivotAngle.in(Degrees));
		DogLog.log("Intake/PivotErrorDeg", targetPivotAngle.in(Degrees) - getPivotAngle().in(Degrees));
		DogLog.log("Intake/PivotAtPosition", isPivotAtPosition());
		DogLog.log("Intake/IsExtended", isExtended());
		DogLog.log("Intake/IsRetracted", isRetracted());

		// Roller tracking
		DogLog.log("Intake/RollerVelocityRPM", getRollerVelocity().in(RPM));
		DogLog.log("Intake/RollerTargetRPM", targetRollerVelocity.in(RPM));
		DogLog.log("Intake/RollerErrorRPM", targetRollerVelocity.in(RPM) - getRollerVelocity().in(RPM));
		DogLog.log("Intake/RollerRunning", isRollerRunning());
		DogLog.log("Intake/RollerAtSpeed", isRollerAtSpeed());

		// Motor data
		DogLog.log("Intake/PivotCurrentAmps", pivotMotor.getOutputCurrent());
		DogLog.log("Intake/PivotVoltage", pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
		DogLog.log("Intake/RollerCurrentAmps", rollerMotor.getOutputCurrent());
		DogLog.log("Intake/RollerVoltage", rollerMotor.getBusVoltage() * rollerMotor.getAppliedOutput());
	}

	private void updateVisualization() {
		double currentAngle = getPivotAngle().in(Degrees);
		double targetAngle = targetPivotAngle.in(Degrees);

		// Update ligament angles
		pivotLigament.setAngle(currentAngle);
		setpointLigament.setAngle(targetAngle);

		// Pivot color based on state
		if (isPivotAtPosition()) {
			pivotLigament.setColor(new Color8Bit(Color.kGreen));
		} else {
			pivotLigament.setColor(new Color8Bit(Color.kYellow));
		}

		// Roller color based on activity
		if (targetRollerVelocity.in(RPM) > 0) {
			rollerLigament.setColor(new Color8Bit(isRollerAtSpeed() ? Color.kLime : Color.kOrange));
		} else if (targetRollerVelocity.in(RPM) < 0) {
			rollerLigament.setColor(new Color8Bit(Color.kRed));
		} else {
			rollerLigament.setColor(new Color8Bit(Color.kGray));
		}
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
