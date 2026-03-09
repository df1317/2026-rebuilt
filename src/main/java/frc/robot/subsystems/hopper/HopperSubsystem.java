package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.HopperConstants.*;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hopper subsystem
 */
public class HopperSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry/visualization)
	// ====================
	final SparkMax hopperMotor;
	private final SparkClosedLoopController hopperController;
	final RelativeEncoder hopperEncoder;

	// ==================== Control State (package-private for telemetry/visualization)
	// ====================
	AngularVelocity targetHopperVelocity = RPM.of(0);

	// ==================== Visualization & Telemetry ====================
	private final HopperTelemetry telemetry;

	// ==================== Test Mode ====================
	private final DoubleSubscriber testHopperRPM = DogLog.tunable("Test/HopperRPM", 0.5);

	public HopperSubsystem() {
		hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
		hopperController = hopperMotor.getClosedLoopController();
		hopperEncoder = hopperMotor.getEncoder();

		configureHopperMotor();

		telemetry = new HopperTelemetry(this);
	}

	private void configureHopperMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kCoast).smartCurrentLimit(HOPPER_CURRENT_LIMIT)
				.inverted(INVERTED);
		config.closedLoop.pid(HOPPER_KP, HOPPER_KI, HOPPER_KD).iZone(HOPPER_I_ZONE);
		config.closedLoop.feedForward.kV(HOPPER_KV);

		hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		telemetry.log();
	}

	// ==================== State Query Methods ====================
	public boolean isHopperAtSpeed() {
		return Math.abs(hopperEncoder.getVelocity()
				- targetHopperVelocity.in(RPM)) < HOPPER_VELOCITY_TOLERANCE.in(RPM);
	}

	// ==================== Control Methods ====================

	public void setHopperVelocity(AngularVelocity velocity) {
		targetHopperVelocity = velocity;
		hopperController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public void stopHopper() {
		setHopperVelocity(RPM.of(0));
	}

	// ==================== Command Factory Methods ====================

	public Command forwardCommand() {
		return runOnce(() -> setHopperVelocity(REVERSE_SPEED)).withName("Hopper Forward");
	}

	public Command reverseCommand() {
		return runOnce(() -> setHopperVelocity(FEED_SPEED)).withName("Hopper Back");
	}

	public Command stopCommand() {
		return runOnce(this::stopHopper).withName("Hopper Stop");
	}

	public Command testHopperCommand() {
		return Commands.run(() -> {
			setHopperVelocity(RPM.of(testHopperRPM.get()));
		}, this).finallyDo(this::stopHopper).withName("Test Hopper");
	}
}
