package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.PIVOT_ARM_LENGTH;

/**
 * Handles Mechanism2d visualization for the intake subsystem.
 */
public class IntakeVisualization {

	private static final double VIZ_WIDTH = 0.6;
	private static final double VIZ_HEIGHT = 0.6;

	private final Mechanism2d mechanism2d;
	private final MechanismLigament2d pivotLigament;
	private final MechanismLigament2d setpointLigament;
	private final MechanismLigament2d rollerLigament;

	public IntakeVisualization() {
		mechanism2d = new Mechanism2d(VIZ_WIDTH, VIZ_HEIGHT);

		MechanismRoot2d root = mechanism2d.getRoot("IntakeRoot", VIZ_WIDTH / 2, 0.1);

		// Setpoint indicator (thin white line showing target angle)
		setpointLigament = root.append(
				new MechanismLigament2d("Setpoint", PIVOT_ARM_LENGTH.in(Meters), 0, 2, new Color8Bit(Color.kWhite)));

		// Main pivot arm (thick, color changes based on state)
		pivotLigament = root.append(
				new MechanismLigament2d("Pivot", PIVOT_ARM_LENGTH.in(Meters), 0, 6, new Color8Bit(Color.kCyan)));

		// Roller indicator at end of pivot (shows roller activity)
		rollerLigament = pivotLigament.append(
				new MechanismLigament2d("Roller", 0.05, 90, 8, new Color8Bit(Color.kGray)));

		SmartDashboard.putData("Intake/Mechanism", mechanism2d);
	}

	/**
	 * Updates the visualization with the current state.
	 */
	public void update(
			double currentAngleDeg,
			double targetAngleDeg,
			boolean pivotAtPosition,
			AngularVelocity targetRollerVelocity,
			boolean rollerAtSpeed) {

		// Update ligament angles
		pivotLigament.setAngle(currentAngleDeg);
		setpointLigament.setAngle(targetAngleDeg);

		// Pivot color based on state
		if (pivotAtPosition) {
			pivotLigament.setColor(new Color8Bit(Color.kGreen));
		} else {
			pivotLigament.setColor(new Color8Bit(Color.kYellow));
		}

		// Roller color based on activity
		if (targetRollerVelocity.in(RPM) > 0) {
			rollerLigament.setColor(new Color8Bit(rollerAtSpeed ? Color.kLime : Color.kOrange));
		} else if (targetRollerVelocity.in(RPM) < 0) {
			rollerLigament.setColor(new Color8Bit(Color.kRed));
		} else {
			rollerLigament.setColor(new Color8Bit(Color.kGray));
		}
	}
}
