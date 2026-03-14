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
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
	// ==================== Test Mode ====================
	private final DoubleSubscriber testHopperRPM = DogLog.tunable("Hopper/RPM", 2000.0);
	private final DoubleSubscriber KP = DogLog.tunable("Hopper/kP", HOPPER_KP);
	private final DoubleSubscriber KI = DogLog.tunable("Hopper/kI", HOPPER_KI);
	private final DoubleSubscriber KD = DogLog.tunable("Hopper/kD", HOPPER_KD);
	private final DoubleSubscriber KV = DogLog.tunable("Hopper/kV", HOPPER_KV);
	private final DoubleSubscriber KS = DogLog.tunable("Hopper/kS", HOPPER_KS);
	// ==================== Control State (package-private for telemetry/visualization)
	// ====================
	AngularVelocity targetHopperVelocity = RPM.of(0);
	double prevKP = KP.getAsDouble();
	double prevKI = KI.getAsDouble();
	double prevKD = KD.getAsDouble();
	double prevKV = KV.getAsDouble();
	double prevKS = KS.getAsDouble();

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
