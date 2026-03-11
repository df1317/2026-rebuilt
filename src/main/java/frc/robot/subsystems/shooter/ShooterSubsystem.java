package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.AT_POSITION_DEBOUNCE_TIME;
import static frc.robot.Constants.ShooterConstants.AT_SPEED_DEBOUNCE_TIME;
import static frc.robot.Constants.ShooterConstants.CURRENT_DEBOUNCE_TIME;
import static frc.robot.Constants.ShooterConstants.CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.HOOD_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.HOOD_STALL_RPM;
import static frc.robot.Constants.ShooterConstants.HOOD_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_KD;
import static frc.robot.Constants.ShooterConstants.SHOOTER_KG;
import static frc.robot.Constants.ShooterConstants.SHOOTER_KI;
import static frc.robot.Constants.ShooterConstants.SHOOTER_KS;
import static frc.robot.Constants.ShooterConstants.SHOOTER_KV;

import java.util.function.Supplier;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

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
  private final Debouncer stallDebouncer;
  private final Debouncer atPositionDebouncer;
  private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceToHoodDeg = new InterpolatingDoubleTreeMap();
  private final SysIdRoutine sysIdRoutine;
  private final DoubleSubscriber feederRPMTunable = DogLog.tunable("Shooter/feederRPMTunable", 0.0, RPM);
  private final DoubleSubscriber testShooterRPM = DogLog.tunable("Test/ShooterRPM", 3000.0, RPM);
  private final DoubleSubscriber testFeederRPM = DogLog.tunable("Test/FeederRPM", 3000.0, RPM);
  private final DoubleSubscriber testHoodDeg = DogLog.tunable("Test/HoodAngleDeg", 45.0, Degrees);

  private final DoubleSubscriber KP = DogLog.tunable("Test/KP", ShooterConstants.HOOD_KP);
  private final DoubleSubscriber KI = DogLog.tunable("Test/KI", ShooterConstants.HOOD_KI);
  private final DoubleSubscriber KD = DogLog.tunable("Test/KP", ShooterConstants.HOOD_KD);
  // private final DoubleSubscriber KV = DogLog.tunable("Test/KV", ShooterConstants.HOOD_KV);

  private final TrapezoidProfile profile;
  private final ElevatorFeedforward feedforward;
  TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private double lastUpdateTimestamp;

  // ==================== Control State (package-private for telemetry) ====================
  AngularVelocity targetVelocity = RPM.of(0);
  AngularVelocity targetFeederVelocity = RPM.of(0);
  Angle targetHoodAngle = Degrees.of(0);

  // ==================== Telemetry ====================
  private final ShooterTelemetry telemetry;

  public ShooterSubsystem() {
    feeder = new SparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless);
    motor = new TalonFX(MOTOR_ID);
    hood = new SparkMax(ShooterConstants.HOOD_ID, MotorType.kBrushless);
    feederEncoder = feeder.getEncoder();
    hoodEncoder = hood.getEncoder();
    feedController = feeder.getClosedLoopController();
    hoodController = hood.getClosedLoopController();
    atSpeedDebouncer = new Debouncer(AT_SPEED_DEBOUNCE_TIME, DebounceType.kRising);
    stallDebouncer = new Debouncer(CURRENT_DEBOUNCE_TIME, DebounceType.kRising);
    atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_TIME, DebounceType.kRising);
    configureMotor();
    configureHood();
    populateLookupTable();

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, ShooterConstants.SYSID_STEP_VOLTAGE, null,
            state -> DogLog.log("Shooter/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(voltage -> motor.setVoltage(voltage.in(Volts)), null, this));

    profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ShooterConstants.MAX_VELOCITY.in(DegreesPerSecond),
            ShooterConstants.MAX_ACCELERATION.in(DegreesPerSecondPerSecond)));
    feedforward = new ElevatorFeedforward(SHOOTER_KS, SHOOTER_KG, SHOOTER_KV);

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
    distanceToRPM.put(1.0, 2500.0);
    distanceToRPM.put(2.0, 3000.0);
    distanceToRPM.put(3.0, 3500.0);
    distanceToRPM.put(4.0, 4000.0);
    distanceToRPM.put(5.0, 4500.0);
    distanceToRPM.put(6.0, 5000.0);

    // Distance (m) -> Hood angle (degrees)
    // TODO: tune these values on the real robot
    distanceToHoodDeg.put(1.0, 60.0);
    distanceToHoodDeg.put(2.0, 50.0);
    distanceToHoodDeg.put(3.0, 42.0);
    distanceToHoodDeg.put(4.0, 35.0);
    distanceToHoodDeg.put(5.0, 30.0);
    distanceToHoodDeg.put(6.0, 25.0);
  }

  double prevKP = KP.getAsDouble();
  double prevKI = KI.getAsDouble();
  double prevKD = KD.getAsDouble();
  // double prevKV = KV.getAsDouble();

  @Override
  public void periodic() {
    telemetry.log();
    goalState.position = targetHoodAngle.in(Degrees);
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = now - lastUpdateTimestamp;
    lastUpdateTimestamp = now;

    double measuredHeight = hoodEncoder.getPosition();

    currentState = profile.calculate(dt, currentState, goalState);

    double ff = feedforward.calculate(currentState.velocity);

    // hoodController.setSetpoint(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
    // ff);

    // setFeederVelocity(RPM.of(feederRPMTunable.get()));

    if (prevKP != KP.getAsDouble() || prevKI != KI.getAsDouble() || prevKD != KD.getAsDouble()) {

      prevKP = KP.getAsDouble();
      prevKI = KI.getAsDouble();
      prevKD = KD.getAsDouble();
      // prevKV = KV.getAsDouble();

      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake).smartCurrentLimit(HOOD_CURRENT_LIMIT)
          .inverted(ShooterConstants.HOOD_INVERTED);
      config.encoder
          // Converts encoder rotations to degrees: (360 deg/rot) / gear_ratio
          .positionConversionFactor(360.0 / ShooterConstants.HOOD_GEAR_RATIO);
      config.closedLoop
          .pid(prevKP, prevKI, prevKD)
          .allowedClosedLoopError(ShooterConstants.HOOD_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

      hood.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

  }

  // ==================== State Queries ====================

  public boolean isAtSpeed() {
    double error = Math.abs(targetVelocity.in(RPM) - (motor.getVelocity().getValueAsDouble() * 60));
    boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
    return atSpeedDebouncer.calculate(withinTolerance);
  }

  public boolean isHoodAtPosition() {
    boolean atPositionRaw = Math.abs(hoodEncoder.getPosition() - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE
        .in(Degrees);
    return atPositionDebouncer.calculate(atPositionRaw);
  }

  public boolean isHoodStalled() {
    double hoodMotorCurrent = hood.getOutputCurrent();
    double hoodMotorRPM = hood.getEncoder().getVelocity(); // RPM
    boolean isHoodStalled = Math.abs(hoodMotorRPM) < HOOD_STALL_RPM && hoodMotorCurrent > HOOD_CURRENT_LIMIT * 0.5;
    return stallDebouncer.calculate(isHoodStalled);
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

  public Angle getTargetHoodAngle() {

    return targetHoodAngle;
  }

  // ==================== Control Methods ====================

  public Angle getHoodAngleForDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double clampedDistance = Math.max(1.0, Math.min(6.0, distanceMeters));
    return Degrees.of(distanceToHoodDeg.get(clampedDistance));
  }

  public void setForDistance(Distance distance) {
    setVelocity(getRPMForDistance(distance));
    setHoodAngle(getHoodAngleForDistance(distance));
  }

  public void setVelocityForDistance(Distance distance) {
    setVelocity(getRPMForDistance(distance));
  }

  public void stop() {
    targetVelocity = RPM.of(0);
    targetFeederVelocity = RPM.of(0.0);
    motor.stopMotor();
    feeder.stopMotor();
    hood.stopMotor();
  }

  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    motor.setControl(new VelocityVoltage(velocity));
  }

  public void setFeederVelocity(AngularVelocity velocity) {
    targetFeederVelocity = velocity;
    feedController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  public void setHoodAngle(Angle angle) {
    System.out.println("hood should move! to " + angle.in(Degrees));
    targetHoodAngle = angle;
    hoodController.setSetpoint(angle.in(Degrees), ControlType.kPosition);
  }

  public void HoodStop() {
    // return Commands.runOnce(() -> {
    System.out.println("STOP HOOD!");
    double currentAngle = hoodEncoder.getPosition();
    targetHoodAngle = Degrees.of(currentAngle);
    hoodController.setSetpoint(currentAngle, ControlType.kPosition);
    // });
  }

  // ==================== Commands ====================

  public Command spinUpCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> setVelocity(velocity), this);
  }

  public Command spinUpForDistanceCommand(Supplier<Distance> distance) {
    return Commands.run(() -> setVelocityForDistance(distance.get()), this);
  }

  public Command spinUpAndWaitCommand(AngularVelocity velocity) {
    return Commands.sequence(spinUpCommand(velocity), Commands.waitUntil(this::isAtSpeed));
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  public Command shootCommand(AngularVelocity velocity) {
    return Commands.startEnd(() -> setVelocity(velocity), this::stop, this);
  }

  public Command shootForDistanceCommand(Supplier<Distance> distance) {
    return Commands.run(() -> setForDistance(distance.get()), this).finallyDo(this::stop);
  }

  public Command hoodSetpoint(Angle angle) {
    return Commands.runOnce(() -> setHoodAngle(angle))
        .andThen(idle().until(() -> isHoodStalled() || isHoodAtPosition()))
        .andThen(runOnce(() -> setHoodAngle(Degrees.of(hoodEncoder.getPosition()))));
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

  public Command testFeederCommand() {
    return Commands.run(() -> setFeederVelocity(RPM.of(testFeederRPM.get())), this)
        .finallyDo(() -> {
          feeder.stopMotor();
          targetFeederVelocity = RPM.of(0);
        })
        .withName("Test Feeder");
  }

  public Command testHoodCommand() {
    return Commands.run(() -> setHoodAngle(Degrees.of(testHoodDeg.get())), this)
        .finallyDo(() -> hood.stopMotor())
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

  public Command homeHoodCommand() {
    return Commands.sequence(
        // Drive hood toward min stop
        Commands.runOnce(() -> hood.setVoltage(-ShooterConstants.HOOD_HOMING_VOLTAGE)),
        Commands.waitUntil(this::isHoodStalled),
        Commands.runOnce(() -> {
          hood.stopMotor();
          hoodEncoder.setPosition(0.0);
        }),
        Commands.waitSeconds(0.25),
        // Drive hood toward max stop
        Commands.runOnce(() -> hood.setVoltage(ShooterConstants.HOOD_HOMING_VOLTAGE)),
        Commands.waitUntil(this::isHoodStalled),
        Commands.runOnce(() -> {
          hood.stopMotor();
          double maxDeg = hoodEncoder.getPosition();
          DogLog.log("Shooter/HoodMaxDeg", maxDeg);
          // Apply soft limits
          SparkMaxConfig config = new SparkMaxConfig();
          config.softLimit
              .forwardSoftLimit((float) maxDeg)
              .forwardSoftLimitEnabled(true)
              .reverseSoftLimit(0.0f)
              .reverseSoftLimitEnabled(true);
          hood.configure(config, ResetMode.kNoResetSafeParameters,
              PersistMode.kNoPersistParameters);
        })).finallyDo(() -> hood.stopMotor()).withName("Home Hood");
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
