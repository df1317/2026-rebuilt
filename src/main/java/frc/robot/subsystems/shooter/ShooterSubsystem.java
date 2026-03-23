package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.CommandBuilder;
import frc.robot.util.Mutable;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

/**
 * Subsystem controlling a single-motor flywheel shooter with feeder and hood.
 */
public class ShooterSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry) ====================
	final TalonFX motor;
	final SparkMax feeder;
	final SparkMax hood;
	final RelativeEncoder feederEncoder;
	final RelativeEncoder hoodEncoder;
	private final SparkClosedLoopController feedController;
	private final SparkClosedLoopController hoodController;
	private final Debouncer atSpeedDebouncer;
	private final Debouncer atFeederSpeedDebouncer;
	private final Debouncer stallDebouncer;
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

	// ==================== Lookup Tables ====================
	private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap distanceToHoodPercent = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap rpmToBallSpeed = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap hoodPercentToLaunchAngle = new InterpolatingDoubleTreeMap();

	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Shooter");
	private final TunableDouble testShooterRPM = tunables.value("RPM", 3000.0, RPM);
	private final TunableDouble testFeederRPM = tunables.getNested("Feeder").value("RPM", 3000.0, RPM);
	private final TunableDouble testHoodPercent = tunables.getNested("Hood").value("Percent", 0.5);
	private final TunableDouble feederFeedRPM = tunables.getNested("Feeder").value("FeedRPM", FEEDER_RPM, RPM);
	private final TunableDouble homingVoltage = tunables.getNested("Hood").value("HomingVoltage", HOOD_HOMING_VOLTAGE);
	private final TunableDouble distanceStepM = tunables.value("DistanceStepM", 0.5);

	// ==================== Telemetry ====================
	private final ShooterTelemetry telemetry;

	// ==================== Control State (package-private for telemetry) ====================
	AngularVelocity targetVelocity = RPM.of(0);
	AngularVelocity targetFeederVelocity = RPM.of(0);
	Angle targetHoodAngle = Degrees.of(0);
	double hoodMaxDeg = Double.NaN;
	boolean manualDistanceEnabled = false;
	private static final double DISTANCE_MIN_M = 0.5;
	private static final double DISTANCE_MAX_M = 9.0;
	private double manualDistanceM = 1.0;
	private Supplier<Distance> autoDistanceSupplier = () -> Meters.of(0);

	public ShooterSubsystem() {
		feeder = new SparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless);
		motor = new TalonFX(MOTOR_ID);
		hood = new SparkMax(ShooterConstants.HOOD_ID, MotorType.kBrushless);
		feederEncoder = feeder.getEncoder();
		hoodEncoder = hood.getEncoder();
		feedController = feeder.getClosedLoopController();
		hoodController = hood.getClosedLoopController();
		atSpeedDebouncer = new Debouncer(AT_SPEED_DEBOUNCE_TIME, DebounceType.kRising);
		atFeederSpeedDebouncer = new Debouncer(AT_SPEED_DEBOUNCE_TIME, DebounceType.kRising);
		stallDebouncer = new Debouncer(CURRENT_DEBOUNCE_TIME, DebounceType.kRising);

		configureFlywheelAndFeeder();
		configureHood();
		populateLookupTables();

		tunables.pidTalonFX("Shooter", motor, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KV, SHOOTER_KS);
		tunables.pidSpark("Feeder", feeder, FEEDER_KP, FEEDER_KI, FEEDER_KD, FEEDER_KV);
		tunables.pidSpark("Hood", hood, HOOD_KP, HOOD_KI, HOOD_KD);

		telemetry = new ShooterTelemetry(this);
	}

	// ==================== Motor Configuration ====================

	private void configureFlywheelAndFeeder() {
		SparkMaxConfig feederConfig = new SparkMaxConfig();
		feederConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.FEEDER_CURRENT_LIMIT)
				.inverted(ShooterConstants.FEEDER_INVERTED);
		feederConfig.closedLoop.pid(FEEDER_KP, FEEDER_KI, FEEDER_KD);
		feederConfig.closedLoop.feedForward.kV(FEEDER_KV);
		feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.Slot0.kP = SHOOTER_KP;
		flywheelConfig.Slot0.kI = SHOOTER_KI;
		flywheelConfig.Slot0.kD = SHOOTER_KD;
		flywheelConfig.Slot0.kV = SHOOTER_KV;
		flywheelConfig.Slot0.kA = SHOOTER_KG;
		flywheelConfig.Slot0.kS = SHOOTER_KS;
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motor.getConfigurator().apply(flywheelConfig);
	}

	private void configureHood() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit(HOOD_CURRENT_LIMIT)
				.inverted(HOOD_INVERTED);
		config.encoder.positionConversionFactor(360.0 / HOOD_GEAR_RATIO);
		config.closedLoop
				.pid(HOOD_KP, HOOD_KI, HOOD_KD)
				.allowedClosedLoopError(HOOD_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);
		hood.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	// ==================== Lookup Tables ====================

	private void populateLookupTables() {
		distanceToRPM.put(1.0, 2993.0);
		distanceToRPM.put(1.5, 2655.0);
		distanceToRPM.put(2.0, 2613.0);
		distanceToRPM.put(2.5, 2717.0);
		distanceToRPM.put(3.0, 2777.0);
		distanceToRPM.put(3.5, 2885.0);
		distanceToRPM.put(4.0, 3048.0);
		distanceToRPM.put(4.5, 3265.0);
		distanceToRPM.put(5.0, 3538.0);
		distanceToRPM.put(5.5, 3865.0);
		distanceToRPM.put(6.0, 4247.0);
		distanceToRPM.put(6.5, 4684.0);
		distanceToRPM.put(7.0, 5175.0);

		distanceToHoodPercent.put(1.0, 0.00);
		distanceToHoodPercent.put(1.5, 0.00);
		distanceToHoodPercent.put(2.0, 0.00);
		distanceToHoodPercent.put(2.5, 0.04);
		distanceToHoodPercent.put(3.0, 0.11);
		distanceToHoodPercent.put(3.5, 0.21);
		distanceToHoodPercent.put(4.0, 0.34);
		distanceToHoodPercent.put(4.5, 0.50);
		distanceToHoodPercent.put(5.0, 0.68);
		distanceToHoodPercent.put(5.5, 0.90);
		distanceToHoodPercent.put(6.0, 1.00);
		distanceToHoodPercent.put(6.5, 1.00);
		distanceToHoodPercent.put(7.0, 1.00);

		rpmToBallSpeed.put(2555.0, BALL_SPEED_LOW_M_S);
		rpmToBallSpeed.put(3250.0, BALL_SPEED_HIGH_M_S);

		hoodPercentToLaunchAngle.put(0.00, 20.0);
		hoodPercentToLaunchAngle.put(0.17, 25.0);
		hoodPercentToLaunchAngle.put(0.36, 32.0);
		hoodPercentToLaunchAngle.put(0.51, 38.0);
	}

	@Override
	public void periodic() {
		telemetry.log();
	}

	// ==================== State Queries ====================

	public boolean isAtSpeed() {
		double error = Math.abs(targetVelocity.in(RPM) - (motor.getVelocity().getValueAsDouble() * 60));
		boolean withinTolerance = error < VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

	public boolean isFeederAtSpeed() {
		double error = Math.abs(targetFeederVelocity.in(RPM) - feederEncoder.getVelocity());
		boolean withinTolerance = error < VELOCITY_TOLERANCE.in(RPM) && targetFeederVelocity.in(RPM) > 0;
		return atFeederSpeedDebouncer.calculate(withinTolerance);
	}

	public boolean isHoodAtPosition() {
		return Math.abs(hoodEncoder.getPosition() - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees);
	}

	public boolean isHoodStalled() {
		double current = hood.getOutputCurrent();
		double rpm = hood.getEncoder().getVelocity();
		return stallDebouncer.calculate(Math.abs(rpm) < HOOD_STALL_RPM && current > HOOD_CURRENT_LIMIT * 0.5);
	}

	public boolean isHoodHomed() {
		return !Double.isNaN(hoodMaxDeg);
	}

	// ==================== Control Methods ====================

	public void setVelocity(AngularVelocity velocity) {
		targetVelocity = velocity;
		motor.setControl(velocityVoltageRequest.withVelocity(velocity));
	}

	public void setFeederVelocity(AngularVelocity velocity) {
		targetFeederVelocity = velocity;
		feedController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public void setHoodAngle(Angle angle) {
		targetHoodAngle = angle;
		hoodController.setSetpoint(angle.in(Degrees), ControlType.kPosition);
	}

	public void setHoodPercent(double percent) {
		if (Double.isNaN(hoodMaxDeg))
			return;
		setHoodAngle(Degrees.of(percent * hoodMaxDeg));
	}

	public void setTestHoodPercent() {
		setHoodAngle(Degrees.of(testHoodPercent.get() * hoodMaxDeg));
	}

	public void setForDistance(Supplier<Distance> distance) {
		Distance d = distance.get();
		setVelocity(getRPMForDistance(d));
		setFeederVelocity(RPM.of(feederFeedRPM.get()));
		setHoodPercent(getHoodPercentForDistance(d));
	}

	public void stop() {
		targetVelocity = RPM.of(0);
		targetFeederVelocity = RPM.of(0);
		setHoodPercent(0);
		motor.stopMotor();
		feeder.stopMotor();
	}

	// ==================== Distance Lookup ====================

	public AngularVelocity getRPMForDistance(Distance distance) {
		double d = Math.max(1.0, Math.min(7.0, distance.in(Meters)));
		return RPM.of(distanceToRPM.get(d));
	}

	public double getHoodPercentForDistance(Distance distance) {
		double d = Math.max(1.0, Math.min(7.0, distance.in(Meters)));
		return distanceToHoodPercent.get(d);
	}

	public double getHorizontalBallSpeedMPS(Distance distance) {
		double exitSpeed = rpmToBallSpeed.get(targetVelocity.in(RPM));
		double hoodPercent = getHoodPercentForDistance(distance);
		double launchAngleDeg = hoodPercentToLaunchAngle.get(hoodPercent);
		return exitSpeed * Math.cos(Math.toRadians(launchAngleDeg));
	}

	public double getTargetHoodPercent() {
		if (Double.isNaN(hoodMaxDeg) || hoodMaxDeg == 0)
			return 0;
		return targetHoodAngle.in(Degrees) / hoodMaxDeg;
	}

	// ==================== Manual Distance Override ====================

	public void setAutoDistanceSupplier(Supplier<Distance> supplier) {
		autoDistanceSupplier = supplier;
	}

	public void clearManualDistanceOverride() {
		manualDistanceEnabled = false;
	}

	public double getActiveDistanceM() {
		return manualDistanceEnabled ? manualDistanceM : autoDistanceSupplier.get().in(Meters);
	}

	public Command advanceDistanceCommand() {
		return runOnce(() -> {
			manualDistanceM = Math.min(manualDistanceM + distanceStepM.get(), DISTANCE_MAX_M);
			manualDistanceEnabled = true;
		}).withName("Shooter.advanceDistance");
	}

	public Command reduceDistanceCommand() {
		return runOnce(() -> {
			manualDistanceM = Math.max(manualDistanceM - distanceStepM.get(), DISTANCE_MIN_M);
			manualDistanceEnabled = true;
		}).withName("Shooter.reduceDistance");
	}

	// ==================== Shooting Commands ====================

	public Command shootForDistanceCommand(Supplier<Distance> distance) {
		return new CommandBuilder("Shooter.shootForDistance", this)
				.onExecute(() -> setForDistance(distance))
				.onEnd(this::stop);
	}

	/**
	 * Spins up the flywheel while reversing the feeder to prevent premature feeding,
	 * then switches to forward feed once at speed.
	 */
	public Command spinUpReverseFeederThenShootCommand(Supplier<Distance> distance) {
		enum Phase{REVERSE,SHOOT}
		Mutable<Phase> phase = new Mutable<>(Phase.REVERSE);

		return new CommandBuilder("Shooter.reverseAndShoot", this)
				.onInitialize(() -> phase.value = Phase.REVERSE)
				.onExecute(() -> {
					Distance d = distance.get();
					switch (phase.value) {
						case REVERSE:
							setVelocity(getRPMForDistance(d));
							setFeederVelocity(RPM.of(-feederFeedRPM.get()));
							setHoodPercent(getHoodPercentForDistance(d));
							if (isAtSpeed()) {
								phase.value = Phase.SHOOT;
							}
							break;
						case SHOOT:
							setForDistance(distance);
							break;
					}
				})
				.onEnd(this::stop);
	}

	/** Spins up flywheel and feeder, then waits until both are at speed. */
	public Command spinUpAndWaitCommand(Supplier<AngularVelocity> shooterVelocity,
			Supplier<AngularVelocity> feederVelocity) {
		return Commands.sequence(
				Commands.runOnce(() -> {
					setVelocity(shooterVelocity.get());
					setFeederVelocity(feederVelocity.get());
				}, this),
				Commands.waitUntil(() -> isFeederAtSpeed() && isAtSpeed()));
	}

	public Command reverseFeederCommand() {
		return new CommandBuilder("Shooter.reverseFeeder", this)
				.onExecute(() -> setFeederVelocity(RPM.of(-testFeederRPM.get())))
				.onEnd(() -> {
					feeder.stopMotor();
					targetFeederVelocity = RPM.of(0);
				});
	}

	// ==================== Test Mode ====================

	public Command testShooterMotorCommand() {
		return new CommandBuilder("Shooter.testMotor", this)
				.onExecute(() -> setVelocity(RPM.of(testShooterRPM.get())))
				.onEnd(this::stop);
	}

	public Command testFeederCommand() {
		return new CommandBuilder("Shooter.testFeeder", this)
				.onExecute(() -> setFeederVelocity(RPM.of(testFeederRPM.get())))
				.onEnd(() -> {
					feeder.stopMotor();
					targetFeederVelocity = RPM.of(0);
				});
	}

	public Command testHoodCommand() {
		return new CommandBuilder("Shooter.testHood", this)
				.onExecute(() -> setHoodPercent(testHoodPercent.get()))
				.onEnd(() -> hood.stopMotor());
	}

	public AngularVelocity getShooterTestRPM() {
		return RPM.of(testShooterRPM.get());
	}

	public AngularVelocity getFeederTestRPM() {
		return RPM.of(testFeederRPM.get());
	}

	// ==================== Hood Homing ====================

	public Command homeHoodCommand() {
		return Commands.sequence(
				Commands.runOnce(() -> {
					SparkMaxConfig config = new SparkMaxConfig();
					config.softLimit.forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false);
					hood.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
				}, this),
				// Find min stop
				Commands.runOnce(() -> {
					stallDebouncer.calculate(false);
					hood.setVoltage(-homingVoltage.get());
				}),
				Commands.waitUntil(this::isHoodStalled).withTimeout(2.0),
				Commands.runOnce(() -> {
					hood.stopMotor();
					hoodEncoder.setPosition(0.0);
					stallDebouncer.calculate(false);
				}),
				// Find max stop
				Commands.runOnce(() -> hood.setVoltage(homingVoltage.get())),
				Commands.waitUntil(this::isHoodStalled).withTimeout(2.0),
				Commands.runOnce(() -> {
					hood.stopMotor();
					hoodMaxDeg = hoodEncoder.getPosition();
					DogLog.log("Shooter/HoodMaxDeg", hoodMaxDeg);
					SparkMaxConfig config = new SparkMaxConfig();
					config.softLimit
							.forwardSoftLimit((float) hoodMaxDeg).forwardSoftLimitEnabled(true)
							.reverseSoftLimit(0.0f).reverseSoftLimitEnabled(true);
					hood.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
				}),
				// Zero to min position
				runOnce(() -> setHoodPercent(0)),
				Commands.waitUntil(this::isHoodAtPosition))
				.finallyDo(hood::stopMotor)
				.withName("Shooter.homeHood");
	}
}
