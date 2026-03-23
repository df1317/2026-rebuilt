package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandBuilder;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;

public class RollerSubsystem extends SubsystemBase {

	// ==================== Hardware Config ====================
	private static final int MOTOR_ID = 30;
	private static final int CURRENT_LIMIT = 40;
	private static final boolean INVERTED = true;
	private static final double SPEED_SCALE_MAX_RPM = 3500;
	private static final double SPEED_SCALE_MAX_ROBOT_MPS = 3.0;
	static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);

	// ==================== State Enum ====================

	private enum State {
		INTAKE(2500.0), EJECT(-1500.0);

		public final TunableDouble rpm;

		State(double rpm) {
			this.rpm = tunables.value("speeds/" + name(), rpm, RPM);
		}
	}

	// ==================== Hardware ====================
	final SparkFlex rollerMotor;
	final RelativeEncoder rollerEncoder;
	private final SparkClosedLoopController rollerController;

	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Intake/Roller");
	// ==================== Control State ====================
	AngularVelocity targetRollerVelocity = RPM.of(0);
	private DoubleSupplier robotSpeedSupplier = () -> 0.0;

	public RollerSubsystem() {
		rollerMotor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
		rollerController = rollerMotor.getClosedLoopController();
		rollerEncoder = rollerMotor.getEncoder();
		configureRollerMotor();

		tunables.pidSpark("Motor", rollerMotor, 2E-4, 1.3E-4, 0.0, 1.5E-4);

		// Enum warmup
		State.INTAKE.rpm.get();
	}

	private void configureRollerMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kCoast).smartCurrentLimit(CURRENT_LIMIT)
				.inverted(INVERTED);
		config.closedLoop.pid(2E-4, 1.3E-4, 0.0).iZone(1E-3);
		config.closedLoop.feedForward.kV(1.5E-4);
		rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
	}

	// ==================== Control Methods ====================

	public void setRollerVelocity(AngularVelocity velocity) {
		targetRollerVelocity = velocity;
		rollerController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public void setRobotSpeedSupplier(DoubleSupplier supplier) {
		this.robotSpeedSupplier = supplier;
	}

	public AngularVelocity getSpeedScaledRollerVelocity() {
		double t = MathUtil.clamp(robotSpeedSupplier.getAsDouble() / SPEED_SCALE_MAX_ROBOT_MPS, 0.0, 1.0);
		double rpm = MathUtil.interpolate(State.INTAKE.rpm.get(), SPEED_SCALE_MAX_RPM, t);
		return RPM.of(rpm);
	}

	public boolean isRollerAtSpeed() {
		return Math.abs(rollerEncoder.getVelocity()
				- targetRollerVelocity.in(RPM)) < VELOCITY_TOLERANCE.in(RPM);
	}

	public void stopRoller() {
		setRollerVelocity(RPM.of(0));
	}

	// ==================== Command Factory Methods ====================

	private Command runState(State state) {
		return new CommandBuilder("Roller." + state.name().toLowerCase(), this)
				.onExecute(() -> setRollerVelocity(RPM.of(state.rpm.get())))
				.onEnd(this::stopRoller);
	}

	public Command runRollerCommand() {
		return runState(State.INTAKE);
	}

	public Command ejectCommand() {
		return runState(State.EJECT);
	}

	/** Intake with speed scaling based on robot velocity. */
	public Command intakeCommand() {
		return new CommandBuilder("Roller.intake", this)
				.onExecute(() -> setRollerVelocity(getSpeedScaledRollerVelocity()))
				.onEnd(this::stopRoller);
	}

}
