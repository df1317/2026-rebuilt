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
import frc.robot.util.CommandBuilder;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RPM;

/**
 * Hopper subsystem using enum-as-config state pattern.
 */
public class HopperSubsystem extends SubsystemBase {

	// ==================== Hardware Config ====================
	private static final int MOTOR_ID = 26;
	private static final int CURRENT_LIMIT = 20;
	private static final boolean INVERTED = true;
	private static final double GEAR_RATIO = 24.0;
	static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);

	// ==================== State Enum ====================

	private enum State {
		FEED(2000.0), REVERSE(-2000.0);

		public final TunableDouble rpm;

		State(double rpm) {
			this.rpm = tunables.value("speeds/" + name(), rpm, RPM);
		}
	}

	// ==================== Hardware (package-private for telemetry) ====================
	final SparkMax hopperMotor;
	final RelativeEncoder hopperEncoder;
	private final SparkClosedLoopController hopperController;

	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Hopper");
	private final TunableDouble testHopperRPM = tunables.value("RPM", 2000.0, RPM);

	// ==================== Telemetry ====================
	private final HopperTelemetry telemetry;

	// ==================== Control State (package-private for telemetry) ====================
	AngularVelocity targetHopperVelocity = RPM.of(0);

	public HopperSubsystem() {
		hopperMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
		hopperController = hopperMotor.getClosedLoopController();
		hopperEncoder = hopperMotor.getEncoder();

		configureHopperMotor();

		tunables.pidSpark("Motor", hopperMotor, 2E-4, 1E-5, 0.0, 1.8E-4);

		telemetry = new HopperTelemetry(this);

		// Enum warmup — forces lazy static enum initialization
		State.FEED.rpm.get();
	}

	private void configureHopperMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kCoast).smartCurrentLimit(CURRENT_LIMIT)
				.inverted(INVERTED);
		config.encoder
				.positionConversionFactor(360.0 / GEAR_RATIO);
		config.closedLoop.pid(2E-4, 1E-5, 0.0).iZone(1E-3);
		config.closedLoop.feedForward.kV(1.8E-4);

		hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		telemetry.log();
	}

	// ==================== State Query Methods ====================

	public boolean isHopperAtSpeed() {
		return Math.abs(hopperEncoder.getVelocity()
				- targetHopperVelocity.in(RPM)) < VELOCITY_TOLERANCE.in(RPM);
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

	/** Runs the hopper at the given state's speed while held, stops on release. */
	private Command runState(State state) {
		return new CommandBuilder("Hopper." + state.name().toLowerCase(), this)
				.onExecute(() -> setHopperVelocity(RPM.of(state.rpm.get())))
				.onEnd(this::stopHopper);
	}

	public Command feedCommand() {
		return runState(State.FEED);
	}

	public Command reverseCommand() {
		return runState(State.REVERSE);
	}

	public Command testHopperCommand() {
		return new CommandBuilder("Hopper.test", this)
				.onExecute(() -> setHopperVelocity(RPM.of(testHopperRPM.get())))
				.onEnd(this::stopHopper);
	}

	public Command setHopperVelocityCommand(Supplier<AngularVelocity> velocity) {
		return Commands.run(() -> setHopperVelocity(velocity.get()), this)
				.finallyDo(this::stopHopper);
	}

	public AngularVelocity getHopperTestRPM() {
		return RPM.of(testHopperRPM.get());
	}
}
