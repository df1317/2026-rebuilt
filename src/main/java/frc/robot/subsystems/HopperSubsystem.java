package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

  private final int motorID = -1;

  private SparkMax motor;

  public SparkBaseConfig configMotor;

  private ClosedLoopSlot slot;

  public final double p = 0.0, i = 0.0, d = 0.0;

  public HopperSubsystem() {

    slot = ClosedLoopSlot.kSlot1;

    configMotor.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);

    configMotor.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(35);
    configMotor.closedLoop.allowedClosedLoopError(0.01, slot).p(p, slot)
      .i(i, slot).d(d, slot);

    motor.configure(configMotor, ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);

  }

  public Command spinup() {

    return runOnce(() -> {
     motor.getClosedLoopController().setSetpoint(2000, ControlType.kVelocity);
    });
  }

  public Command spindown() {

    return runOnce(() -> {
      motor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
    });
  }
}
