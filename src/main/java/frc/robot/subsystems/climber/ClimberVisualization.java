package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ClimberConstants.MAX_HEIGHT;
import static frc.robot.Constants.ClimberConstants.MIN_HEIGHT;

/**
 * Handles Mechanism2d visualization for the climber subsystem.
 */
public class ClimberVisualization {

	private static final double VIZ_WIDTH = 0.5;
	private static final double VIZ_HEIGHT = MAX_HEIGHT.in(Meters) * 1.2;

	private final ClimberSubsystem climber;
	private final Mechanism2d mechanism2d;
	private final MechanismLigament2d elevatorLigament;
	private final MechanismLigament2d setpointLigament;

	public ClimberVisualization(ClimberSubsystem climber) {
		this.climber = climber;

		mechanism2d = new Mechanism2d(VIZ_WIDTH, VIZ_HEIGHT);

		MechanismRoot2d root = mechanism2d.getRoot("ClimberRoot", VIZ_WIDTH / 2, 0.05);

		// Hard limit indicators
		mechanism2d.getRoot("MinLimit", VIZ_WIDTH / 2 - 0.05, 0.05)
				.append(new MechanismLigament2d("Min", MIN_HEIGHT.in(Meters) + 0.01, 90, 2, new Color8Bit(Color.kRed)));
		mechanism2d.getRoot("MaxLimit", VIZ_WIDTH / 2 - 0.05, 0.05)
				.append(new MechanismLigament2d("Max", MAX_HEIGHT.in(Meters), 90, 2, new Color8Bit(Color.kLimeGreen)));

		// Setpoint indicator (thin white line showing target)
		setpointLigament = root.append(
				new MechanismLigament2d("Setpoint", 0, 90, 3, new Color8Bit(Color.kWhite)));

		// Main elevator ligament (thick cyan showing actual position)
		elevatorLigament = root.append(
				new MechanismLigament2d("Elevator", 0, 90, 6, new Color8Bit(Color.kCyan)));

		SmartDashboard.putData("Climber/mechanism", mechanism2d);
	}

	/**
	 * Updates the visualization with the current state.
	 */
	public void update() {
		elevatorLigament.setLength(climber.getHeightMeters());
		setpointLigament.setLength(climber.goalState.position);
		elevatorLigament.setColor(new Color8Bit(climber.getStatusColor()));
	}
}
