package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.HopperConstants.*;

/**
 * Hopper subsystem
 */
public class HopperSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry/visualization)
	// ====================
	final SparkMax hopperMotor;
	final RelativeEncoder hopperEncoder;
	private final SparkClosedLoopController hopperController;
	// ==================== Visualization & Telemetry ====================
	private final HopperTelemetry telemetry;
	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Hopper");
	private final TunableDouble testHopperRPM = tunables.value("RPM", FEED_SPEED.in(RPM), RPM);
	// ==================== Control State (package-private for telemetry/visualization)
	// ====================
	AngularVelocity targetHopperVelocity = RPM.of(0);

	public HopperSubsystem() {
		hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
		hopperController = hopperMotor.getClosedLoopController();
		hopperEncoder = hopperMotor.getEncoder();

		configureHopperMotor();

		tunables.pidSpark("Motor", hopperMotor, HOPPER_KP, HOPPER_KI, HOPPER_KD, HOPPER_KV);

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

	/** Runs the hopper at feed speed while held, stops on release. */
	public Command feedCommand() {
		return Commands.run(() -> setHopperVelocity(FEED_SPEED), this)
				.finallyDo(this::stopHopper)
				.withName("Hopper Feed");
	}

	public Command reverseCommand() {
		return Commands.run(() -> setHopperVelocity(REVERSE_SPEED), this)
				.finallyDo(this::stopHopper)
				.withName("Hopper Reverse");
	}

	public Command testHopperCommand() {
		return Commands.run(() -> {
			setHopperVelocity(RPM.of(testHopperRPM.get()));
		}, this).finallyDo(this::stopHopper).withName("Test Hopper");
	}

	public Command setHopperVelocityCommand(Supplier<AngularVelocity> velocity) {
		return Commands.run(() -> setHopperVelocity(velocity.get()), this)
				.finallyDo(this::stopHopper);
	}

	public AngularVelocity getHopperTestRPM() {
		return RPM.of(testHopperRPM.get());
	}
}
