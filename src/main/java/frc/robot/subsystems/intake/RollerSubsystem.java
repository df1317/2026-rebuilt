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
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.*;

public class RollerSubsystem extends SubsystemBase {

	final SparkFlex rollerMotor;
	final RelativeEncoder rollerEncoder;
	private final SparkClosedLoopController rollerController;
	// Roller PID tunables
	private final DoubleSubscriber tuneRollerKP = DogLog.tunable("Intake/Roller/kP", ROLLER_KP);
	private final DoubleSubscriber tuneRollerKI = DogLog.tunable("Intake/Roller/kI", ROLLER_KI);
	private final DoubleSubscriber tuneRollerKD = DogLog.tunable("Intake/Roller/kD", ROLLER_KD);
	private final DoubleSubscriber tuneRollerKV = DogLog.tunable("Intake/Roller/kV", ROLLER_KV);
	private final DoubleSubscriber testRollerRPM = DogLog.tunable("Intake/Roller/RPM",
			ROLLER_INTAKE_VELOCITY.in(RPM), RPM);
	AngularVelocity targetRollerVelocity = RPM.of(0);
	private double prevRollerKP = ROLLER_KP, prevRollerKI = ROLLER_KI, prevRollerKD = ROLLER_KD,
			prevRollerKV = ROLLER_KV;
	private DoubleSupplier robotSpeedSupplier = () -> 0.0;

	public RollerSubsystem() {
		rollerMotor = new SparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
		rollerController = rollerMotor.getClosedLoopController();
		rollerEncoder = rollerMotor.getEncoder();
		configureRollerMotor();
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
		updateRollerPIDIfChanged();
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

	public void setRollerVelocity(AngularVelocity velocity) {
		targetRollerVelocity = velocity;
		rollerController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public void setRobotSpeedSupplier(DoubleSupplier supplier) {
		this.robotSpeedSupplier = supplier;
	}

	public AngularVelocity getSpeedScaledRollerVelocity() {
		double t = MathUtil.clamp(robotSpeedSupplier.getAsDouble() / ROLLER_SPEED_SCALE_MAX_ROBOT_MPS, 0.0, 1.0);
		double rpm = MathUtil.interpolate(ROLLER_INTAKE_VELOCITY.in(RPM), ROLLER_SPEED_SCALE_MAX_RPM, t);
		return RPM.of(rpm);
	}

	public boolean isRollerAtSpeed() {
		return Math.abs(rollerEncoder.getVelocity()
				- targetRollerVelocity.in(RPM)) < ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	public void stopRoller() {
		setRollerVelocity(RPM.of(0));
	}

	// ==================== Command Factory Methods ====================

	public Command runRollerCommand() {
		return run(() -> setRollerVelocity(ROLLER_INTAKE_VELOCITY))
				.finallyDo(this::stopRoller).withName("Run Roller");
	}

	public Command ejectCommand() {
		return run(() -> setRollerVelocity(ROLLER_EJECT_VELOCITY))
				.finallyDo(this::stopRoller).withName("Eject Roller");
	}

	public Command intakeCommand() {
		return run(() -> setRollerVelocity(getSpeedScaledRollerVelocity()))
				.finallyDo(this::stopRoller)
				.withName("Intake Roller");
	}

	public Command stopRollerCommand() {
		return runOnce(this::stopRoller).withName("Stop Roller");
	}

	public Command testRollerCommand() {
		return Commands.run(() -> setRollerVelocity(RPM.of(testRollerRPM.get())), this)
				.finallyDo(this::stopRoller)
				.withName("Test Roller");
	}
}
