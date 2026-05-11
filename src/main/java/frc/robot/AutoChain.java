package frc.robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopZoneAutomation;
import frc.robot.repulsor.Repulsor;

/**
 * Dashboard-driven auto composer. Each step is a {@link SendableChooser} so the
 * driver can build arbitrary sequences on the fly.
 */
public final class AutoChain {

	private static final int NUM_STEPS = 6;

	private final Repulsor repulsor;
	private final List<SendableChooser<Consumer<AutoBuilder>>> stepChoosers = new ArrayList<>();
	private final Map<String, Consumer<AutoBuilder>> steps = new LinkedHashMap<>();

	public AutoChain(Repulsor repulsor, TeleopZoneAutomation teleopAutomation) {
		this.repulsor = repulsor;
		buildStepMap(teleopAutomation);
		for (int i = 0; i < NUM_STEPS; i++) {
			var chooser = new SendableChooser<Consumer<AutoBuilder>>();
			chooser.setDefaultOption("---", b -> {
			});
			for (var entry : steps.entrySet()) {
				chooser.addOption(entry.getKey(), entry.getValue());
			}
			stepChoosers.add(chooser);
			SmartDashboard.putData("misc/Auto Step " + (i + 1), chooser);
		}
	}

	/**
	 * Returns a deferred command that reads the step choosers when scheduled and
	 * builds the sequence on the fly. Add this as an option in the main auto chooser.
	 */
	public Command asCommand() {
		return Commands.deferredProxy(() -> {
			AutoBuilder builder = new AutoBuilder(repulsor);
			boolean hasSteps = false;
			for (var chooser : stepChoosers) {
				Consumer<AutoBuilder> step = chooser.getSelected();
				if (step != null) {
					int sizeBefore = builder.stepCount();
					step.accept(builder);
					if (builder.stepCount() > sizeBefore) {
						hasSteps = true;
					}
				}
			}
			return hasSteps ? builder.build() : Commands.none();
		});
	}

	private void buildStepMap(TeleopZoneAutomation teleopAutomation) {
		// Drive steps
		steps.put("Drive to Hub Front", b -> b.driveTo(AutoPositions.HUB_FRONT));
		steps.put("Drive to Center", b -> b.driveToAndHold(AutoPositions.CENTER_COLLECT));
		steps.put("Drive to Left Corner", b -> b.driveToFacing(
				AutoPositions.CORNER_HIDE_NEAR_BALLS, AutoPositions.HUB_CENTER));
		steps.put("Drive to Right Corner", b -> b.driveToFacing(
				AutoPositions.CORNER_HIDE, AutoPositions.HUB_CENTER));
		steps.put("Return to Start", b -> b.driveToStart());
		steps.put("Collect (closest side)", b -> b.driveToCollect());

		// Shoot
		if (teleopAutomation != null) {
			steps.put("Shoot", b -> b.run(teleopAutomation.shootCommand()));
			steps.put("Shoot (repeat)", b -> b.run(teleopAutomation.shootCommand().repeatedly()));
		}

		// Wait
		steps.put("Wait 0.5s", b -> b.waitSeconds(0.5));
		steps.put("Wait 1s", b -> b.waitSeconds(1.0));
		steps.put("Wait 2s", b -> b.waitSeconds(2.0));
	}
}
