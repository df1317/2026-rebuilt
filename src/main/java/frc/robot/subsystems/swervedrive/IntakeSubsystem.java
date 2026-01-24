package frc.robot.subsystems.swervedrive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final int positionMotorID = -1;
  private final int spinMotorID = -1;

  private SparkMax positonMotor;
  private SparkMax spinMotor;

  public SparkBaseConfig configPosition;
  public SparkBaseConfig configSpin;

  private ClosedLoopSlot positionSlot;
  private ClosedLoopSlot spinSlot;

  public final double positionP = 0.0, positionI = 0.0, positionD = 0.0;
  public final double spinP = 0.0, spinI = 0.0, spinD = 0.0;

  public IntakeSubsystem() {

    positionSlot = ClosedLoopSlot.kSlot1;
    spinSlot = ClosedLoopSlot.kSlot1;

    configPosition.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);
    configPosition.closedLoop.allowedClosedLoopError(0.01, positionSlot).p(positionP, positionSlot)
        .i(positionI, positionSlot).d(positionD, positionSlot);

    configSpin.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(35);
    configSpin.closedLoop.allowedClosedLoopError(0.01, spinSlot).p(spinP, spinSlot)
        .i(spinI, spinSlot).d(spinD, spinSlot);

    spinMotor.configure(configSpin, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    positonMotor.configure(configPosition, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

  }
}
