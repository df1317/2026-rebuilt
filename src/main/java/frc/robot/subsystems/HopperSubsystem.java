package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

	private final int motorID = -1;

	private SparkMax motor;

	public SparkBaseConfig configMotor;

	private ClosedLoopSlot slot;

	public final double p = 0.0, i = 0.0, d = 0.0;

	public HopperSubsystem() {

		motor = new SparkMax(Constants.HopperConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

		slot = ClosedLoopSlot.kSlot1;

		configMotor.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);

		configMotor.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(35);
		configMotor.closedLoop.allowedClosedLoopError(0.01, slot).p(p, slot)
				.i(i, slot).d(d, slot);

		motor.configure(configMotor, ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);

	}

	/**
	 * Configures the SparkMAX motor controller settings.
	 *
	 * <p>
	 * Sets idle mode, current limit, inversion, and closed-loop PID/feedforward gains
	 * from {@link frc.robot.Constants.HopperConstants}.
	 */
	private void configureMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kCoast)
				.smartCurrentLimit(Constants.HopperConstants.CURRENT_LIMIT)
				.inverted(Constants.HopperConstants.INVERTED);
		config.closedLoop
				.pid(Constants.HopperConstants.KP, Constants.HopperConstants.KI, Constants.HopperConstants.KD);
		config.closedLoop.feedForward
				.kV(Constants.HopperConstants.KV);

		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	// Spins down wheel to active speed set in Constants.HopperConstants
	public Command spinup() {

		return runOnce(() -> {
			motor.getClosedLoopController().setSetpoint(Constants.HopperConstants.WheelSpinupRPM, ControlType.kVelocity);
		});
	}

	// Spins down wheel to idle speed set in Constants.HopperConstants
	public Command spindown() {

		return runOnce(() -> {
			motor.getClosedLoopController().setSetpoint(Constants.HopperConstants.WheelSpindownRPM, ControlType.kVelocity);
		});
	}
}
