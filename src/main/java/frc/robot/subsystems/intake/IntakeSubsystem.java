package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandBuilder;
import frc.robot.util.TunableBoolean;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableTable;

/**
 * Intake pivot subsystem. The roller is a separate {@link RollerSubsystem} so they can run commands independently (e.g.
 * roller keeps spinning while the pivot retracts).
 */
public class IntakeSubsystem extends SubsystemBase {

  // ==================== Hardware Config ====================
  private static final int PIVOT_MOTOR_ID = 25;
  private static final int PIVOT_CURRENT_LIMIT = 35;
  private static final int EXTERN_ENCODER_CHANNEL_A = 4;
  private static final int EXTERN_ENCODER_CHANNEL_B = 7;
  private static final boolean PIVOT_INVERTED = false;
  private static final double PIVOT_GEAR_RATIO = (48.0 * 22.0) / 14.0;
  private static final Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(8);
  private static final double STALL_DEBOUNCE_S = 1.5;
  private static final double AT_POSITION_DEBOUNCE_S = 0.1;
  private static final double STALL_RPM_THRESHOLD = 2.0;
  private static final double STALL_CURRENT_RATIO = 0.75;
  private static final double JOG_DOWN_SPEED = -0.1;

  // ==================== PID Gains ====================
  private static final double PIVOT_KP = 0.05;
  private static final double PIVOT_KI = 0.0;
  private static final double PIVOT_KD = 0.0;

  // ==================== Default Tunable Values ====================
  private static final double DEFAULT_EXTENDED_ANGLE_DEG = 14.0;
  private static final double DEFAULT_RETRACT_DELTA_DEG = 90.0;
  private static final double DEFAULT_FAST_VELOCITY = 240.0;
  private static final double DEFAULT_FAST_ACCEL = 240.0;
  private static final double DEFAULT_SLOW_VELOCITY = 60.0;
  private static final double DEFAULT_SLOW_ACCEL = 180.0;
  private static final double DEFAULT_KICK_DURATION_S = 0.5;
  private static final double DEFAULT_HOMING_OFFSET_DEG = -20.0;
  // ==================== Tunables ====================
  private static final TunableTable tunables = new TunableTable("Intake");
  private static final TunableTable pivotTunables = tunables.getNested("Pivot");
  // ==================== Hardware ====================
  final SparkMax pivotMotor;
  final RelativeEncoder pivotEncoder;
  final Encoder externEncoder;
  private final SparkClosedLoopController pivotController;
  private final Debouncer atPositionDebouncer;
  private final Debouncer stallDebouncer = new Debouncer(STALL_DEBOUNCE_S, DebounceType.kBoth);
  private final TunableDouble testPivotDeg = pivotTunables.value("Degrees", DEFAULT_EXTENDED_ANGLE_DEG, Degrees);
  private final TunableDouble retractDelta = pivotTunables.value("RetractDelta", DEFAULT_RETRACT_DELTA_DEG, Degrees);
  private final TunableDouble fastVelocity = pivotTunables.value("FastVelDegPerS", DEFAULT_FAST_VELOCITY);
  private final TunableDouble fastAccel = pivotTunables.value("FastAccelDegPerS2", DEFAULT_FAST_ACCEL);
  private final TunableDouble slowVelocity = pivotTunables.value("SlowVelDegPerS", DEFAULT_SLOW_VELOCITY);
  private final TunableDouble slowAccel = pivotTunables.value("SlowAccelDegPerS2", DEFAULT_SLOW_ACCEL);
  private final TunableDouble kickDurationS = pivotTunables.value("KickDurationS", DEFAULT_KICK_DURATION_S);
  private final TunableDouble homingOffset = pivotTunables.value("HomingOffsetDeg", DEFAULT_HOMING_OFFSET_DEG);

  private final TunableDouble stowTune = pivotTunables.value("stowTune", 0.0);
  private final TunableDouble extendTune = pivotTunables.value("extendTune", -340.0);
  private final TunableDouble speedTune = pivotTunables.value("speedTune", 0.1);

  private final TunableDouble THRESHOLD_EXTERNAL = pivotTunables.value("External Threshold", 5);
  private final TunableDouble THRESHOLD_INTERNAL = pivotTunables.value("Internal Threshold", 2);

  double stowStart;

  private double initialValInternalEncoder;
  private double initialValExternalEncoder;
  // final double initialEncoderRatio = 1 / 5.6;
  double encoderRatio = 1 / 5.6;
  private final TunableDouble encoderRatioTunable = pivotTunables.value("encoderRatio", encoderRatio);
  private final TunableBoolean useEncoderConversion = pivotTunables.value("encoderConversion", false);

  // ==================== Telemetry ====================
  private final IntakeTelemetry telemetry;

  public enum IntakeState {
    STOWED, EXTENDING_KICK, EXTENDING_GENTLE, EXTENDED, JOGGING_UP, JOGGING_DOWN, HOMING, TEST, UNKNOWN
  }

  private IntakeState currentState = IntakeState.UNKNOWN;
  private final Timer stateTimer = new Timer();

  public void setState(IntakeState newState) {
    if (currentState != newState) {
      // updateEncoderToRealPos();
      currentState = newState;
      stateTimer.restart();
    }
  }

  private final ProfiledPIDController pivotProfiler = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(DEFAULT_FAST_VELOCITY, DEFAULT_FAST_ACCEL));
  private final RollerSubsystem roller;
  boolean homed = false;
  private boolean wantToExtend = false;
  private Angle extendedPivotAngle = Degrees.of(DEFAULT_EXTENDED_ANGLE_DEG);
  Angle targetPivotAngle = extendedPivotAngle.plus(Degrees.of(DEFAULT_RETRACT_DELTA_DEG));

  private double maxPivotPos = 0;
  private double pivotOffset = 0;

  public IntakeSubsystem(RollerSubsystem roller) {
    this.roller = roller;
    pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotController = pivotMotor.getClosedLoopController();
    pivotEncoder = pivotMotor.getEncoder();
    externEncoder = new Encoder(EXTERN_ENCODER_CHANNEL_A, EXTERN_ENCODER_CHANNEL_B);

    initialValExternalEncoder = externEncoder.get();
    initialValInternalEncoder = pivotEncoder.getPosition();

    configurePivotMotor();

    atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_S, DebounceType.kRising);

    telemetry = new IntakeTelemetry(this, roller);

    tunables.pidSpark("Pivot", pivotMotor, PIVOT_KP, PIVOT_KI, PIVOT_KD, 0.0);

    stowStart = externEncoder.get();

    // if (RobotBase.isSimulation()) {
    // pivotEncoder.setPosition(retractedAngleDeg());
    // }

    // double initialAngle = pivotEncoder.getPosition();
    // pivotProfiler.reset(initialAngle);
    // setPivotAngle(Degrees.of(initialAngle));
  }

  private void configurePivotMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(PIVOT_CURRENT_LIMIT)
        .inverted(PIVOT_INVERTED);
    config.encoder
        .positionConversionFactor(360.0 / PIVOT_GEAR_RATIO);
    config.closedLoop.pid(PIVOT_KP, PIVOT_KI, PIVOT_KD)
        .allowedClosedLoopError(PIVOT_ANGLE_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void updateEncoderToRealPos() {
    encoderRatio = encoderRatioTunable.get();
    pivotEncoder
        .setPosition((externEncoder.get() - initialValExternalEncoder) * encoderRatio + initialValInternalEncoder);
  }

  public boolean isPivotStalled() {
    double pivotMotorCurrent = pivotMotor.getOutputCurrent();
    double pivotMotorRPM = pivotMotor.getEncoder().getVelocity();
    boolean isCurrentStalled = Math.abs(pivotMotorRPM) < STALL_RPM_THRESHOLD
        && pivotMotorCurrent > PIVOT_CURRENT_LIMIT * STALL_CURRENT_RATIO;
    boolean isDesyncStalled = Math.abs(pivotEncoder.getVelocity()) > THRESHOLD_INTERNAL.get()
        && Math.abs(externEncoder.getRate()) > THRESHOLD_EXTERNAL.get();
    DogLog.log("desyncStall", isDesyncStalled);
    DogLog.log("externalEncoderRate", externEncoder.getRate());
    return stallDebouncer.calculate(isCurrentStalled || isDesyncStalled);
  }

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(15, 30);
  TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0, 0.0);
  TrapezoidProfile profile = new TrapezoidProfile(constraints);
  TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  long t = System.nanoTime();

  @Override
  public void periodic() {
    // boolean enabled = edu.wpi.first.wpilibj.DriverStation.isEnabled();
    // if (enabled && !wasEnabled) {
    // double pos = pivotEncoder.getPosition();
    // pivotProfiler.reset(pos);
    // setPivotAngle(Degrees.of(pos));
    // double distToExtended = Math.abs(pos - extendedPivotAngle.in(Degrees));
    // double distToRetracted = Math.abs(pos - retractedAngleDeg());
    // wantToExtend = distToExtended < distToRetracted;
    // if (wantToExtend) {
    // setState(IntakeState.EXTENDED);
    // } else {
    // setState(IntakeState.STOWED);
    // }
    // }
    // wasEnabled = enabled;

    double externEncoderPos = externEncoder.get();
    previousProfiledReference.position = externEncoderPos;

    if (currentState == IntakeState.HOMING && stateTimer.get() > 1.25) {
      currentState = IntakeState.EXTENDED;
      pivotOffset = (maxPivotPos - extendTune.get());
      goal.position = maxPivotPos;
    }

    previousProfiledReference = profile.calculate((System.nanoTime() - t) / 1e9, previousProfiledReference,
        goal);
    t = System.nanoTime();

    if (externEncoderPos < maxPivotPos) {
      maxPivotPos = externEncoderPos;
    }

    DogLog.log("pivotGoal", goal.position);
    DogLog.log("intake/state", currentState);

    DogLog.log("intake/externEcoder", externEncoderPos);

    // double epsilon = 10;
    // if (currentState == IntakeState.STOWED) {
    // sign = externEncoder.get() + stowStart < stowTune.get() ? 1 : -1;
    // if (Math.abs(externEncoder.get() + stowStart - stowTune.get()) < epsilon)
    // sign = 0;
    // } else if (currentState == IntakeState.EXTENDED) {
    // sign = externEncoder.get() + stowStart > extendTune.get() ? -1 : 1;
    // if (Math.abs(-externEncoder.get() + stowStart + extendTune.get()) < epsilon)
    // sign = 0;
    // }

    // if (isPivotStalled()) {
    // // sign = 0;

    // // if (currentState == IntakeState.STOWED) {
    // // stowStart = externEncoder.get() + stowTune.get();
    // // }
    // // if (currentState == IntakeState.EXTENDED) {
    // // stowStart = externEncoder.get() + extendTune.get();
    // // }
    // }

    // DogLog.log("intake/sign",sign);

    if (!MathUtil.isNear(goal.position, externEncoderPos, 10)) {
      pivotMotor.set(previousProfiledReference.velocity * speedTune.get() * encoderRatioTunable.get());
    } else if (currentState == IntakeState.JOGGING_UP) {
      goal.position++;
      goal.velocity = 0;
    } else if (currentState == IntakeState.JOGGING_DOWN) {
      goal.position--;
      goal.velocity = 0;
    } else {
      pivotMotor.set(0);
    }

    DogLog.log("pivot_offset", pivotOffset);
    DogLog.log("pivot_max", maxPivotPos);
    DogLog.log("pivot_pos", previousProfiledReference.position);
    DogLog.log("pivot_velocity", previousProfiledReference.velocity);

  }

  // ==================== Command Factory Methods ====================

  /**
   * Extends the intake using a two-phase motion profile: KICK phase pushes through the panel at full speed, then GENTLE
   * phase slows down for a controlled landing.
   */
  public Command extendCommand() {
    return new CommandBuilder("Intake.extend", this)
        .onInitialize(() -> {
          System.out.println("EXTEND COMMAND!");
          setState(IntakeState.EXTENDED);
          goal.position = extendTune.get() + pivotOffset;
          goal.velocity = 0;
          wantToExtend = true;
        }).isFinished(true);
  }

  public Command retractCommand() {
    return new CommandBuilder("Intake.retract", this)
        .onInitialize(() -> {
          System.out.println("STOW COMMAND!");
          setState(IntakeState.STOWED);
          goal.position = stowTune.get() + pivotOffset;
          goal.velocity = 0;
          wantToExtend = false;
        }).isFinished(true);
  }

  public Command stowCommand() {
    return retractCommand().withName("Intake.stow");
  }

  public Command atBottCommand() {
    return new CommandBuilder("intake.atBottom", this).onInitialize(() -> {
      pivotOffset = (maxPivotPos - extendTune.get());
    }).isFinished(true);
  }

  public Command homeCommand() {
    return new CommandBuilder("intake.home", this).onInitialize(() -> {
      setState(IntakeState.HOMING);
      goal.position = 2 * extendTune.get();
      goal.velocity = 0;
      wantToExtend = true;
    }).isFinished(true);
  }

  /** Extends or stows depending on current position. */
  public Command stowToggleCommand() {
    return Commands.either(stowCommand(), extendCommand(), () -> wantToExtend)
        .withName("Intake.toggle");
  }

  public Command jogDownCommand() {
    return new CommandBuilder("Intake.jogDown", this)
        .onInitialize(() -> {
          currentState = IntakeState.JOGGING_DOWN;
        })
        .onEnd(() -> {
        });
  }

  public Command jogUpCommand() {
    return new CommandBuilder("Intake.jogUp", this)
        .onInitialize(() -> {
          currentState = IntakeState.JOGGING_UP;
        })
        .onEnd(() -> {
        });
  }
}
