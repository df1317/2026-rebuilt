package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.*;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Subsystem controlling a dual-motor flywheel shooter with feeder and hood. The two flywheel Krakens are physically
 * opposed on the same axle, so the second motor follows the first with opposite direction.
 */
public class ShooterSubsystem extends SubsystemBase {

	// ==================== Hardware Config ====================
	private static final int MOTOR_ID = 40;
	private static final int MOTOR2_ID = 41;
	private static final int FEEDER_ID = 28;
	private static final int HOOD_ID = 24;
	private static final boolean FLYWHEEL_INVERTED = false;
	private static final boolean FEEDER_INVERTED = true;
	private static final boolean HOOD_INVERTED = true;
	private static final int FLYWHEEL_CURRENT_LIMIT = 25;
	private static final int FEEDER_CURRENT_LIMIT = 35;
	private static final int HOOD_CURRENT_LIMIT = 20;
	private static final double HOOD_GEAR_RATIO = 24.0;
	private static final Angle HOOD_TOLERANCE = Degrees.of(3);
	private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
	private static final double HOOD_STALL_RPM = 2.0;
	private static final double HOOD_STALL_CURRENT_RATIO = 0.5;
	private static final double AT_SPEED_DEBOUNCE_S = 0.1;
	private static final double STALL_DEBOUNCE_S = 0.1;
	private static final double HOMING_TIMEOUT_S = 2.0;

	// ==================== Flywheel PID ====================
	private static final double FLYWHEEL_KP = 0.17;
	private static final double FLYWHEEL_KI = 0.001;
	private static final double FLYWHEEL_KD = 0.0;
	private static final double FLYWHEEL_KV = 0.115;
	private static final double FLYWHEEL_KA = 0.0;
	private static final double FLYWHEEL_KS = 0.0;

	// ==================== Feeder PID ====================
	private static final double FEEDER_KP = 0.0002;
	private static final double FEEDER_KI = 0.0;
	private static final double FEEDER_KD = 0.0;
	private static final double FEEDER_KV = 0.000175;

	// ==================== Hood PID ====================
	private static final double HOOD_KP = 0.013;
	private static final double HOOD_KI = 0.0;
	private static final double HOOD_KD = 0.0;

	// ==================== Default Tunable Values ====================
	private static final double DEFAULT_FEEDER_FEED_RPM = 3000.0;
	private static final double DEFAULT_HOMING_VOLTAGE = 5.0;

	// ==================== Ball Physics ====================
	private static final double BALL_SPEED_LOW_M_S = 4.97;
	private static final double BALL_SPEED_HIGH_M_S = 6.15;
	private static final double BALL_SPEED_LOW_RPM = 2555.0;
	private static final double BALL_SPEED_HIGH_RPM = 3250.0;

	// ==================== Distance Bounds ====================
	private static final double DISTANCE_MIN_M = 0.5;
	private static final double DISTANCE_MAX_M = 9.0;
	// ==================== Tunables ====================
	private static final TunableTable tunables = new TunableTable("Shooter");
	// ==================== Hardware (package-private for telemetry) ====================
	final TalonFX motor;
	final TalonFX motor2;
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
	private final TunableDouble testShooterRPM = tunables.value("RPM", 3000.0, RPM);
	private final TunableDouble testFeederRPM = tunables.getNested("Feeder").value("RPM", 3000.0, RPM);
	private final TunableDouble testHoodPercent = tunables.getNested("Hood").value("Percent", 0.5);
	private final TunableDouble feederFeedRPM = tunables.getNested("Feeder").value("FeedRPM", DEFAULT_FEEDER_FEED_RPM,
			RPM);
	private final TunableDouble homingVoltage = tunables.getNested("Hood").value("HomingVoltage",
			DEFAULT_HOMING_VOLTAGE);
	private final TunableDouble distanceStepM = tunables.value("DistanceStepM", 0.5);

	// ==================== Telemetry ====================
	private final ShooterTelemetry telemetry;

	// ==================== Control State (package-private for telemetry) ====================
	AngularVelocity targetVelocity = RPM.of(0);
	AngularVelocity targetFeederVelocity = RPM.of(0);
	Angle targetHoodAngle = Degrees.of(0);
	double hoodMaxDeg = Double.NaN;
	boolean manualDistanceEnabled = false;
	private double manualDistanceM = 1.0;
	private boolean wasAtSpeed = false;
	private Supplier<Distance> autoDistanceSupplier = () -> Meters.of(0);

	public ShooterSubsystem() {
		feeder = new SparkMax(FEEDER_ID, MotorType.kBrushless);
		motor = new TalonFX(MOTOR_ID);
		motor2 = new TalonFX(MOTOR2_ID);
		hood = new SparkMax(HOOD_ID, MotorType.kBrushless);
		feederEncoder = feeder.getEncoder();
		hoodEncoder = hood.getEncoder();
		feedController = feeder.getClosedLoopController();
		hoodController = hood.getClosedLoopController();
		atSpeedDebouncer = new Debouncer(AT_SPEED_DEBOUNCE_S, DebounceType.kRising);
		atFeederSpeedDebouncer = new Debouncer(AT_SPEED_DEBOUNCE_S, DebounceType.kRising);
		stallDebouncer = new Debouncer(STALL_DEBOUNCE_S, DebounceType.kRising);

		configureFlywheelAndFeeder();
		configureFollowerMotor();
		configureHood();
		populateLookupTables();

		tunables.pidTalonFX("Shooter", motor, FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KV, FLYWHEEL_KS);
		tunables.pidSpark("Feeder", feeder, FEEDER_KP, FEEDER_KI, FEEDER_KD, FEEDER_KV);
		tunables.pidSpark("Hood", hood, HOOD_KP, HOOD_KI, HOOD_KD);

		if (RobotBase.isSimulation()) {
			hoodEncoder.setPosition(0.0);
			hoodMaxDeg = 60.0;
		}

		telemetry = new ShooterTelemetry(this);
	}

	// ==================== Motor Configuration ====================

	private void configureFlywheelAndFeeder() {
		SparkMaxConfig feederConfig = new SparkMaxConfig();
		feederConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(FEEDER_CURRENT_LIMIT)
				.inverted(FEEDER_INVERTED);
		feederConfig.closedLoop.pid(FEEDER_KP, FEEDER_KI, FEEDER_KD);
		feederConfig.closedLoop.feedForward.kV(FEEDER_KV);
		feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.Slot0.kP = FLYWHEEL_KP;
		flywheelConfig.Slot0.kI = FLYWHEEL_KI;
		flywheelConfig.Slot0.kD = FLYWHEEL_KD;
		flywheelConfig.Slot0.kV = FLYWHEEL_KV;
		flywheelConfig.Slot0.kA = FLYWHEEL_KA;
		flywheelConfig.Slot0.kS = FLYWHEEL_KS;
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = FLYWHEEL_CURRENT_LIMIT;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.MotorOutput.Inverted = FLYWHEEL_INVERTED ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motor.getConfigurator().apply(flywheelConfig);
		motor2.getConfigurator().apply(flywheelConfig);
	}

	private void configureFollowerMotor() {
		motor2.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
		motor2.setNeutralMode(NeutralModeValue.Coast);
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

		rpmToBallSpeed.put(BALL_SPEED_LOW_RPM, BALL_SPEED_LOW_M_S);
		rpmToBallSpeed.put(BALL_SPEED_HIGH_RPM, BALL_SPEED_HIGH_M_S);

		hoodPercentToLaunchAngle.put(0.00, 30.0);
		hoodPercentToLaunchAngle.put(0.17, 35.0);
		hoodPercentToLaunchAngle.put(0.36, 42.0);
		hoodPercentToLaunchAngle.put(0.51, 48.0);
	}

	@Override
	public void periodic() {
		boolean isCurrentlyAtSpeed = isAtSpeed();
		if (wasAtSpeed != isCurrentlyAtSpeed) {
			DogLog.log("Shooter/Flywheel/AtSpeedRaw", isCurrentlyAtSpeed);
			wasAtSpeed = isCurrentlyAtSpeed;
		}

		if (RobotBase.isSimulation()) {
			hoodEncoder.setPosition(targetHoodAngle.in(Degrees));
			// Note: We can't trivially set Phoenix 6 TalonFX velocity without physics sim,
			// so we might need a workaround for `isAtSpeed()`.
		}

		telemetry.log();
	}

	// ==================== State Queries ====================

	public boolean isAtSpeed() {
		if (RobotBase.isSimulation()) {
			return targetVelocity.in(RPM) > 0;
		}
		double error = Math.abs(targetVelocity.in(RPM) - (motor.getVelocity().getValueAsDouble() * 60));
		boolean withinTolerance = error < VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

	public boolean isFeederAtSpeed() {
		if (RobotBase.isSimulation()) {
			return targetFeederVelocity.in(RPM) > 0;
		}
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
		return stallDebouncer
				.calculate(Math.abs(rpm) < HOOD_STALL_RPM && current > HOOD_CURRENT_LIMIT * HOOD_STALL_CURRENT_RATIO);
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
		motor2.stopMotor();
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

	public double getBallSpeedMPS(Distance distance) {
		// Calculate linear velocity for 4-inch wheels (0.1016 meters diameter)
		// V = RPM * (2 * PI * r) / 60
		double radiusMeters = 0.1016 / 2.0;
		// Multiply by an efficiency factor to account for slip and compression
		double efficiency = 0.4;
		return (targetVelocity.in(RPM) * (2.0 * Math.PI * radiusMeters) / 60.0) * efficiency;
	}

	public double getHorizontalBallSpeedMPS(Distance distance) {
		double exitSpeed = getBallSpeedMPS(distance);
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
		Timer shotTimer = new Timer();
		return new CommandBuilder("Shooter.shootForDistance", this)
				.onInitialize(() -> {
					Distance d = distance.get();
					edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(BallVisualizer.shoot(
							() -> getBallSpeedMPS(d),
							() -> hoodPercentToLaunchAngle.get(getHoodPercentForDistance(d))));
					shotTimer.restart();
				})
				.onExecute(() -> {
					setForDistance(distance);
					if (shotTimer.hasElapsed(1.0 / 8.0)) {
						Distance d = distance.get();
						edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(BallVisualizer.shoot(
								() -> getBallSpeedMPS(d),
								() -> hoodPercentToLaunchAngle.get(getHoodPercentForDistance(d))));
						shotTimer.restart();
					}
				})
				.onEnd(this::stop);
	}

	/**
	 * Spins up the flywheel while reversing the feeder to prevent premature feeding, then switches to forward feed once
	 * at speed.
	 */
	public Command spinUpReverseFeederThenShootCommand(Supplier<Distance> distance) {
		enum Phase{REVERSE,SHOOT}
		Mutable<Phase> phase = new Mutable<>(Phase.REVERSE);
		Timer shotTimer = new Timer();

		return new CommandBuilder("Shooter.reverseAndShoot", this)
				.onInitialize(() -> {
					phase.value = Phase.REVERSE;
					shotTimer.reset();
					shotTimer.start();
				})
				.onExecute(() -> {
					Distance d = distance.get();
					switch (phase.value) {
						case REVERSE:
							setVelocity(getRPMForDistance(d));
							setFeederVelocity(RPM.of(-feederFeedRPM.get()));
							setHoodPercent(getHoodPercentForDistance(d));
							if (isAtSpeed()) {
								phase.value = Phase.SHOOT;
								edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(BallVisualizer.shoot(
										() -> getBallSpeedMPS(d),
										() -> hoodPercentToLaunchAngle.get(getHoodPercentForDistance(d))));
								shotTimer.restart();
							}
							break;
						case SHOOT:
							setForDistance(distance);
							// Simulate repeating shots if held down (8 balls per second)
							if (shotTimer.hasElapsed(1.0 / 8.0)) {
								edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(BallVisualizer.shoot(
										() -> getBallSpeedMPS(d),
										() -> hoodPercentToLaunchAngle.get(getHoodPercentForDistance(d))));
								shotTimer.restart();
							}
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
				Commands.waitUntil(this::isHoodStalled).withTimeout(HOMING_TIMEOUT_S),
				Commands.runOnce(() -> {
					hood.stopMotor();
					hoodEncoder.setPosition(0.0);
					stallDebouncer.calculate(false);
				}),
				// Find max stop
				Commands.runOnce(() -> hood.setVoltage(homingVoltage.get())),
				Commands.waitUntil(this::isHoodStalled).withTimeout(HOMING_TIMEOUT_S),
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
