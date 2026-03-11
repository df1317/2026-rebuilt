package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.HopperConstants.FEED_SPEED;
import static frc.robot.Constants.HopperConstants.GEAR_RATIO;
import static frc.robot.Constants.HopperConstants.HOPPER_CURRENT_LIMIT;
import static frc.robot.Constants.HopperConstants.HOPPER_I_ZONE;
import static frc.robot.Constants.HopperConstants.HOPPER_KD;
import static frc.robot.Constants.HopperConstants.HOPPER_KI;
import static frc.robot.Constants.HopperConstants.HOPPER_KP;
import static frc.robot.Constants.HopperConstants.HOPPER_KV;
import static frc.robot.Constants.HopperConstants.HOPPER_MOTOR_ID;
import static frc.robot.Constants.HopperConstants.HOPPER_VELOCITY_TOLERANCE;
import static frc.robot.Constants.HopperConstants.INVERTED;
import static frc.robot.Constants.HopperConstants.REVERSE_SPEED;

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
import frc.robot.Constants.HopperConstants;

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
	private final DoubleSubscriber testHopperRPM = DogLog.tunable("Test/HopperRPM", 2000.0);

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
		config.encoder
				// Converts encoder rotations to degrees: (360 deg/rot) / gear_ratio
				.positionConversionFactor(360.0 / GEAR_RATIO);
		config.closedLoop.pid(HOPPER_KP, HOPPER_KI, HOPPER_KD).iZone(HOPPER_I_ZONE);
		config.closedLoop.feedForward.kV(HOPPER_KV);

		hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private final DoubleSubscriber KP = DogLog.tunable("Test/KP", HopperConstants.HOPPER_KP);
	private final DoubleSubscriber KI = DogLog.tunable("Test/KI", HopperConstants.HOPPER_KI);
	private final DoubleSubscriber KD = DogLog.tunable("Test/KP", HopperConstants.HOPPER_KD);
	private final DoubleSubscriber KV = DogLog.tunable("Test/KV", HopperConstants.HOPPER_KV);
	private final DoubleSubscriber KS = DogLog.tunable("Test/KS", HopperConstants.HOPPER_KS);

	double prevKP = KP.getAsDouble();
	double prevKI = KI.getAsDouble();
	double prevKD = KD.getAsDouble();
	double prevKV = KV.getAsDouble();
	double prevKS = KS.getAsDouble();

	@Override
	public void periodic() {
		telemetry.log();
		if (prevKP != KP.getAsDouble() || prevKI != KI.getAsDouble() || prevKD != KD.getAsDouble()
				|| prevKV != KV.getAsDouble()) {

			prevKP = KP.getAsDouble();
			prevKI = KI.getAsDouble();
			prevKD = KD.getAsDouble();
			prevKV = KV.getAsDouble();

			SparkMaxConfig config = new SparkMaxConfig();
			config.idleMode(IdleMode.kCoast).smartCurrentLimit(HOPPER_CURRENT_LIMIT)
					.inverted(INVERTED);
			config.closedLoop.pid(KP.getAsDouble(), KI.getAsDouble(), KD.getAsDouble()).iZone(HOPPER_I_ZONE);
			config.closedLoop.feedForward.kV(KV.getAsDouble());

			hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		}
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
