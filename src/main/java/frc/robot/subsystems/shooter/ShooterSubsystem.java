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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

/**
 * Subsystem controlling a single-motor flywheel shooter.
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
	private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap distanceToHoodPercent = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap rpmToBallSpeed = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap hoodPercentToLaunchAngle = new InterpolatingDoubleTreeMap();
	private final SysIdRoutine sysIdRoutine;
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);
	private final DoubleSubscriber testShooterRPM = DogLog.tunable("Shooter/RPM", 3000.0, RPM);
	private final DoubleSubscriber testFeederRPM = DogLog.tunable("Shooter/Feeder/RPM", 3000.0, RPM);
	private final DoubleSubscriber testHoodPercent = DogLog.tunable("Shooter/Hood/Percent", 0.5);

	// Shooter PID tunables (TalonFX)
	private final DoubleSubscriber tuneShooterKP = DogLog.tunable("Shooter/Shooter/kP", SHOOTER_KP);
	private final DoubleSubscriber tuneShooterKI = DogLog.tunable("Shooter/Shooter/kI", SHOOTER_KI);
	private final DoubleSubscriber tuneShooterKD = DogLog.tunable("Shooter/Shooter/kD", SHOOTER_KD);
	private final DoubleSubscriber tuneShooterKV = DogLog.tunable("Shooter/Shooter/kV", SHOOTER_KV);
	private final DoubleSubscriber tuneShooterKS = DogLog.tunable("Shooter/Shooter/kS", SHOOTER_KS);
	// Feeder PID tunables (SparkMax)
	private final DoubleSubscriber tuneFeederKP = DogLog.tunable("Shooter/Feeder/kP", FEEDER_KP);
	private final DoubleSubscriber tuneFeederKI = DogLog.tunable("Shooter/Feeder/kI", FEEDER_KI);
	private final DoubleSubscriber tuneFeederKD = DogLog.tunable("Shooter/Feeder/kD", FEEDER_KD);
	private final DoubleSubscriber tuneFeederKV = DogLog.tunable("Shooter/Feeder/kV", FEEDER_KV);
	// Hood PID tunables (SparkMax)
	private final DoubleSubscriber tuneHoodKP = DogLog.tunable("Shooter/Hood/kP", HOOD_KP);
	private final DoubleSubscriber tuneHoodKI = DogLog.tunable("Shooter/Hood/kI", HOOD_KI);
	private final DoubleSubscriber tuneHoodKD = DogLog.tunable("Shooter/Hood/kD", HOOD_KD);
	// ==================== Telemetry ====================
	private final ShooterTelemetry telemetry;
	// ==================== Control State (package-private for telemetry) ====================
	public AngularVelocity targetFeederVelocity = RPM.of(0);
	AngularVelocity targetVelocity = RPM.of(0);
	Angle targetHoodAngle = Degrees.of(0);
	double hoodMaxDeg = Double.NaN;
	private double prevShooterKP = SHOOTER_KP, prevShooterKI = SHOOTER_KI, prevShooterKD = SHOOTER_KD,
			prevShooterKV = SHOOTER_KV, prevShooterKS = SHOOTER_KS;
	private double prevFeederKP = FEEDER_KP, prevFeederKI = FEEDER_KI, prevFeederKD = FEEDER_KD,
			prevFeederKV = FEEDER_KV;
	private double prevHoodKP = HOOD_KP, prevHoodKI = HOOD_KI, prevHoodKD = HOOD_KD;

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
		configureMotor();
		configureHood();
		populateLookupTable();

		sysIdRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(null, ShooterConstants.SYSID_STEP_VOLTAGE, null,
						state -> DogLog.log("Shooter/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism(voltage -> motor.setVoltage(voltage.in(Volts)), null, this));

		telemetry = new ShooterTelemetry(this);
	}

	private void configureMotor() {
		SparkMaxConfig clonedConfig = new SparkMaxConfig();
		clonedConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.FEEDER_CURRENT_LIMIT)
				.inverted(ShooterConstants.FEEDER_INVERTED);
		clonedConfig.closedLoop.pid(ShooterConstants.FEEDER_KP, ShooterConstants.FEEDER_KI, ShooterConstants.FEEDER_KD);
		clonedConfig.closedLoop.feedForward.kV(ShooterConstants.FEEDER_KV);
		feeder.configure(clonedConfig, ResetMode.kResetSafeParameters,
				PersistMode.kNoPersistParameters);

		// Configure the TalonFX for basic use
		TalonFXConfiguration configs = new TalonFXConfiguration();
		// This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on
		// slot 0
		configs.Slot0.kP = ShooterConstants.SHOOTER_KP;
		configs.Slot0.kI = SHOOTER_KI;
		configs.Slot0.kD = SHOOTER_KD;
		configs.Slot0.kV = SHOOTER_KV;
		configs.Slot0.kA = SHOOTER_KG;
		configs.Slot0.kS = SHOOTER_KS;

		configs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
		configs.CurrentLimits.SupplyCurrentLimitEnable = true;
		configs.MotorOutput.Inverted = ShooterConstants.INVERTED ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;

		motor.getConfigurator().apply(configs);
	}

	private void configureHood() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit(HOOD_CURRENT_LIMIT)
				.inverted(ShooterConstants.HOOD_INVERTED);
		config.encoder
				// Converts encoder rotations to degrees: (360 deg/rot) / gear_ratio
				.positionConversionFactor(360.0 / ShooterConstants.HOOD_GEAR_RATIO);
		config.closedLoop
				.pid(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD)
				.allowedClosedLoopError(ShooterConstants.HOOD_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

		hood.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void populateLookupTable() {
		// Distance (m) -> Flywheel RPM
		distanceToRPM.put(1.6, 2600.0);
//		distanceToRPM.put(1.93, 2555.0);
		distanceToRPM.put(2.30, 2700.0);
		distanceToRPM.put(2.5, 2750.0);
		distanceToRPM.put(2.8, 2750.0);
		distanceToRPM.put(3.11, 2850.0);
		distanceToRPM.put(3.52, 2950.0);
		distanceToRPM.put(4.00, 3100.0);
//		distanceToRPM.put(4.16, 3100.0);
		distanceToRPM.put(4.6, 3250.0);
//		distanceToRPM.put(4.70, 3250.0);

		// Flywheel RPM -> Ball exit speed (m/s)
		rpmToBallSpeed.put(2555.0, BALL_SPEED_LOW_M_S);
		rpmToBallSpeed.put(3250.0, BALL_SPEED_HIGH_M_S);

		// Hood position (0.0-1.0) -> Ball launch angle (degrees) - NEEDS MEASUREMENT
		hoodPercentToLaunchAngle.put(0.00, 20.0);
		hoodPercentToLaunchAngle.put(0.17, 25.0);
		hoodPercentToLaunchAngle.put(0.36, 32.0);
		hoodPercentToLaunchAngle.put(0.51, 38.0);

		// Distance (m) -> Hood position (0.0 = min stop, 1.0 = max stop)
		distanceToHoodPercent.put(2.5, 0.00);
		distanceToHoodPercent.put(2.8, 0.17);
		distanceToHoodPercent.put(3.5, 0.17);
		distanceToHoodPercent.put(4.00, 0.36);
		distanceToHoodPercent.put(4.6, 0.53);
		distanceToHoodPercent.put(4.70, 0.51);
	}

	@Override
	public void periodic() {
		telemetry.log();
		updateShooterPIDIfChanged();
		updateFeederPIDIfChanged();
		updateHoodPIDIfChanged();
	}

  public Command tune(DoubleSupplier shooter, DoubleSupplier feeder, DoubleSupplier hood){
    return Commands.run(()-> {
      this.setVelocity(RPM.of(shooter.getAsDouble()));
      this.setFeederVelocity(RPM.of(feeder.getAsDouble()));
      this.setHoodPercent(hood.getAsDouble());
    });
  }

	private void updateShooterPIDIfChanged() {
		double kP = tuneShooterKP.getAsDouble(), kI = tuneShooterKI.getAsDouble(),
				kD = tuneShooterKD.getAsDouble(), kV = tuneShooterKV.getAsDouble(),
				kS = tuneShooterKS.getAsDouble();
		if (kP == prevShooterKP && kI == prevShooterKI && kD == prevShooterKD
				&& kV == prevShooterKV && kS == prevShooterKS)
			return;
		prevShooterKP = kP;
		prevShooterKI = kI;
		prevShooterKD = kD;
		prevShooterKV = kV;
		prevShooterKS = kS;
		var configs = new com.ctre.phoenix6.configs.Slot0Configs();
		configs.kP = kP;
		configs.kI = kI;
		configs.kD = kD;
		configs.kV = kV;
		configs.kS = kS;
		motor.getConfigurator().apply(configs);
	}

	private void updateFeederPIDIfChanged() {
		double kP = tuneFeederKP.getAsDouble(), kI = tuneFeederKI.getAsDouble(),
				kD = tuneFeederKD.getAsDouble(), kV = tuneFeederKV.getAsDouble();
		if (kP == prevFeederKP && kI == prevFeederKI && kD == prevFeederKD && kV == prevFeederKV)
			return;
		prevFeederKP = kP;
		prevFeederKI = kI;
		prevFeederKD = kD;
		prevFeederKV = kV;
		SparkMaxConfig config = new SparkMaxConfig();
		config.closedLoop.pid(kP, kI, kD);
		config.closedLoop.feedForward.kV(kV);
		feeder.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void updateHoodPIDIfChanged() {
		double kP = tuneHoodKP.getAsDouble(), kI = tuneHoodKI.getAsDouble(), kD = tuneHoodKD.getAsDouble();
		if (kP == prevHoodKP && kI == prevHoodKI && kD == prevHoodKD)
			return;
		prevHoodKP = kP;
		prevHoodKI = kI;
		prevHoodKD = kD;
		SparkMaxConfig config = new SparkMaxConfig();
		config.closedLoop.pid(kP, kI, kD);
		hood.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	// ==================== State Queries ====================

	public boolean isAtSpeed() {
		double error = Math.abs(targetVelocity.in(RPM) - (motor.getVelocity().getValueAsDouble() * 60));
		boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

	public boolean isFeederAtSpeed() {
		double error = Math.abs(targetFeederVelocity.in(RPM) - feederEncoder.getVelocity());
		boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE.in(RPM) && targetFeederVelocity.in(RPM) > 0;
		return atFeederSpeedDebouncer.calculate(withinTolerance);
	}

	public boolean isHoodAtPosition() {
		return Math.abs(hoodEncoder.getPosition() - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees);
	}

	public boolean isHoodStalled() {
		double hoodMotorCurrent = hood.getOutputCurrent();
		double hoodMotorRPM = hood.getEncoder().getVelocity(); // RPM
		boolean isHoodStalled = Math.abs(hoodMotorRPM) < HOOD_STALL_RPM && hoodMotorCurrent > HOOD_CURRENT_LIMIT * 0.5;
		return stallDebouncer.calculate(isHoodStalled);
	}

	public double getHorizontalBallSpeedMPS(Distance distance) {
		double exitSpeed = rpmToBallSpeed.get(targetVelocity.in(RPM));
		double hoodPercent = getHoodPercentForDistance(distance);
		double launchAngleDeg = hoodPercentToLaunchAngle.get(hoodPercent);
		return exitSpeed * Math.cos(Math.toRadians(launchAngleDeg));
	}

	public AngularVelocity getRPMForDistance(Distance distance) {
		double distanceMeters = distance.in(Meters);
		double clampedDistance = Math.max(1.0, Math.min(6.0, distanceMeters));
		return RPM.of(distanceToRPM.get(clampedDistance));
	}

	public AngularVelocity getTargetVelocity() {
		return targetVelocity;
	}

	public AngularVelocity getTargetFeederVelocity() {
		return targetFeederVelocity;
	}

	// ==================== Control Methods ====================

	public double getHoodPercentForDistance(Distance distance) {
		double clampedDistance = Math.max(1.0, Math.min(6.0, distance.in(Meters)));
		return distanceToHoodPercent.get(clampedDistance);
	}

	public void setForDistance(Supplier<Distance> distance) {
		setVelocity(getRPMForDistance(distance.get()));
		setFeederVelocity(RPM.of(FEEDER_RPM));
		setHoodPercent(getHoodPercentForDistance(distance.get()));
	}

	public void setVelocityForDistance(Distance distance) {
		setVelocity(getRPMForDistance(distance));
	}

	public void stop() {
		targetVelocity = RPM.of(0);
		targetFeederVelocity = RPM.of(0.0);
		setHoodPercent(0);
		motor.stopMotor();
		feeder.stopMotor();
	}

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

	/** Sets the hood to a percentage of its full range. 0.0 = min stop, 1.0 = max stop. */
	public void setHoodPercent(double percent) {
		if (Double.isNaN(hoodMaxDeg))
			return; // not homed yet
		setHoodAngle(Degrees.of(percent * hoodMaxDeg));
	}

	/** Returns the current target hood position as a fraction of its full range (0.0–1.0). */
	public double getTargetHoodPercent() {
		if (Double.isNaN(hoodMaxDeg) || hoodMaxDeg == 0)
			return 0;
		return targetHoodAngle.in(Degrees) / hoodMaxDeg;
	}

	public boolean isHoodHomed() {
		return !Double.isNaN(hoodMaxDeg);
	}

	public void hoodStop() {
		double currentAngle = hoodEncoder.getPosition();
		targetHoodAngle = Degrees.of(currentAngle);
		hoodController.setSetpoint(currentAngle, ControlType.kPosition);
	}

	// ==================== Commands ====================

	public Command spinUpCommand(Supplier<AngularVelocity> velocity) {
		return Commands.runOnce(() -> setVelocity(velocity.get()), this);
	}

	public Command spinUpFeederCommand(Supplier<AngularVelocity> velocity) {
		return Commands.runOnce(() -> setFeederVelocity(velocity.get()), this);
	}

	public Command spinUpForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> setVelocityForDistance(distance.get()), this);
	}

	public Command spinUpAndWaitCommand(Supplier<AngularVelocity> shooterVelocity,
			Supplier<AngularVelocity> feederVelocity) {
		return Commands.sequence(
				spinUpCommand(shooterVelocity),
				spinUpFeederCommand(feederVelocity),
				Commands.waitUntil(() -> isFeederAtSpeed() && isAtSpeed()));
	}

	public Command stopCommand() {
		return Commands.runOnce(this::stop, this);
	}

	public Command shootCommand(AngularVelocity velocity) {
		return Commands.startEnd(() -> setVelocity(velocity), this::stop, this);
	}

	public Command shootForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> {
			Distance d = distance.get();
			setForDistance(distance);
			DogLog.log("Shooter/DistanceM", d.in(Meters));
			DogLog.log("Shooter/ComputedRPM", getRPMForDistance(d).in(RPM));
			DogLog.log("Shooter/ComputedHoodPercent", getHoodPercentForDistance(d));
		}, this).finallyDo(this::stop);
	}

	// ==================== Test Mode ====================

	public Command testShooterMotorCommand() {
		return Commands.run(() -> setVelocity(RPM.of(testShooterRPM.get())), this)
				.finallyDo(() -> {
					motor.stopMotor();
					targetVelocity = RPM.of(0);
				})
				.withName("Test Shooter Motor");
	}

	public Command reverseFeederCommand() {
		return Commands.run(() -> setFeederVelocity(RPM.of(-testFeederRPM.get())), this)
				.finallyDo(() -> {
					feeder.stopMotor();
					targetFeederVelocity = RPM.of(0);
				})
				.withName("Reverse Feeder");
	}

	public Command testFeederCommand() {
		return Commands.run(() -> setFeederVelocity(RPM.of(testFeederRPM.get())), this)
				.finallyDo(() -> {
					feeder.stopMotor();
					targetFeederVelocity = RPM.of(0);
				})
				.withName("Test Feeder");
	}

	public Command testHoodCommand() {
		return Commands.run(() -> setHoodPercent(testHoodPercent.get()), this)
				.finallyDo(hood::stopMotor)
				.withName("Test Hood");
	}

	public Command testFullMotorCommand() {
		return Commands.run(() -> {
					setVelocity(RPM.of(testShooterRPM.get()));
					setFeederVelocity(RPM.of(testFeederRPM.get()));
				}, this)
				.finallyDo(() -> {
					motor.stopMotor();
					feeder.stopMotor();
					targetFeederVelocity = RPM.of(0);
					targetVelocity = RPM.of(0);
				})
				.withName("Test Shooter Motor");
	}

	public AngularVelocity getShooterTestRPM() {
		return RPM.of(testShooterRPM.get());
	}

	public AngularVelocity getFeederTestRPM() {
		return RPM.of(testFeederRPM.get());
	}

	/** Jogs the hood using a joystick axis [-1, 1]. Holds position when released. */
	public Command jogHoodCommand(java.util.function.DoubleSupplier axis) {
		return Commands.run(() -> hood.setVoltage(axis.getAsDouble() * HOOD_HOMING_VOLTAGE), this)
				.finallyDo(this::hoodStop)
				.withName("Jog Hood");
	}

	public Command zeroHood() {
		return Commands.runOnce(() -> {
			setHoodPercent(0);
		}).andThen(Commands.waitUntil(this::isHoodAtPosition));
	}

	public Command homeHoodCommand() {
		return Commands.sequence(
						// Disable soft limits so homing can reach the hard stops
						Commands.runOnce(() -> {
							SparkMaxConfig config = new SparkMaxConfig();
							config.softLimit
									.forwardSoftLimitEnabled(false)
									.reverseSoftLimitEnabled(false);
							hood.configure(config, ResetMode.kNoResetSafeParameters,
									PersistMode.kNoPersistParameters);
						}, this),
						// Drive hood toward min stop
						Commands.runOnce(() -> {
							stallDebouncer.calculate(false); // reset stale debouncer state
							hood.setVoltage(-ShooterConstants.HOOD_HOMING_VOLTAGE);
						}),
						Commands.waitUntil(this::isHoodStalled).withTimeout(5.0),
						Commands.runOnce(() -> {
							hood.stopMotor();
							hoodEncoder.setPosition(0.0);
						}),
						Commands.waitSeconds(0.25),
						// Drive hood toward max stop
						Commands.runOnce(() -> {
							stallDebouncer.calculate(false); // reset debouncer between phases
							hood.setVoltage(ShooterConstants.HOOD_HOMING_VOLTAGE);
						}),
						Commands.waitUntil(this::isHoodStalled).withTimeout(5.0),
						Commands.runOnce(() -> {
							hood.stopMotor();
							hoodMaxDeg = hoodEncoder.getPosition();
							DogLog.log("Shooter/HoodMaxDeg", hoodMaxDeg);
							// Apply soft limits based on measured range
							SparkMaxConfig config = new SparkMaxConfig();
							config.softLimit
									.forwardSoftLimit((float) hoodMaxDeg)
									.forwardSoftLimitEnabled(true)
									.reverseSoftLimit(0.0f)
									.reverseSoftLimitEnabled(true);
							hood.configure(config, ResetMode.kNoResetSafeParameters,
									PersistMode.kNoPersistParameters);
						}), zeroHood())
				.finallyDo(hood::stopMotor)
				.withName("Home Hood");
	}

	/** Marks the current hood position as the minimum (0%). Zeroes the encoder and sets the reverse soft limit. */
	public Command markHoodMinHereCommand() {
		return Commands.runOnce(() -> {
			hoodEncoder.setPosition(0.0);
			hoodMaxDeg = Double.NaN; // range unknown until max is also set
			SparkMaxConfig config = new SparkMaxConfig();
			config.softLimit
					.reverseSoftLimit(0.0f)
					.reverseSoftLimitEnabled(true)
					.forwardSoftLimitEnabled(false);
			hood.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			DogLog.log("Shooter/HoodMinSet", true);
		}, this).withName("Mark Hood Min");
	}

	/** Marks the current hood position as the maximum (100%). Records hoodMaxDeg and sets the forward soft limit. */
	public Command markHoodMaxHereCommand() {
		return Commands.runOnce(() -> {
			hoodMaxDeg = hoodEncoder.getPosition();
			DogLog.log("Shooter/HoodMaxDeg", hoodMaxDeg);
			SparkMaxConfig config = new SparkMaxConfig();
			config.softLimit
					.forwardSoftLimit((float) hoodMaxDeg)
					.forwardSoftLimitEnabled(true)
					.reverseSoftLimit(0.0f)
					.reverseSoftLimitEnabled(true);
			hood.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		}, this).withName("Mark Hood Max");
	}

	// ==================== SysId ====================

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	public Command sysIdFullCommand() {
		return Commands.sequence(sysIdQuasistatic(SysIdRoutine.Direction.kForward),
				Commands.waitSeconds(1), sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
				Commands.waitSeconds(1), sysIdDynamic(SysIdRoutine.Direction.kForward),
				Commands.waitSeconds(1), sysIdDynamic(SysIdRoutine.Direction.kReverse));
	}

}
