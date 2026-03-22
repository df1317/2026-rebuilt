package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * NetworkTables-based autonomous chooser that automatically schedules
 * the selected command when the robot enters autonomous mode.
 *
 * <p>
 * Each option is stored as a {@code Supplier<Command>} so a fresh
 * command instance is created every time auto starts. This avoids
 * WPILib's single-use command restriction and ensures runtime state
 * (alliance color, robot pose) is captured at schedule time.
 *
 * <p>
 * Adapted from Team 340 (GRR) — licensed under GPLv3.
 */
public final class AutoChooser {

	private static final String DEFAULT = "Do Nothing";

	private final StringArrayPublisher optionsPub;
	private final StringPublisher activePub;
	private final StringSubscriber selectedSub;

	private final Map<String, Supplier<Command>> options = new LinkedHashMap<>();

	private String activeName = DEFAULT;
	private Command activeCommand = null;
	private boolean running = false;

	/**
	 * Creates an auto chooser published to the given NetworkTables key.
	 *
	 * @param name
	 *          the NT table path (e.g. {@code "/Autos"} or {@code "misc/Auto Chooser"})
	 */
	public AutoChooser(String name) {
		NetworkTable nt = NetworkTableInstance.getDefault().getTable(name);

		nt.getStringTopic(".type").publish().set("String Chooser");
		nt.getBooleanTopic(".controllable").publish().set(true);
		nt.getStringTopic("default").publish().set(DEFAULT);

		optionsPub = nt.getStringArrayTopic("options").publish();
		activePub = nt.getStringTopic("active").publish();
		selectedSub = nt.getStringTopic("selected").subscribe(DEFAULT);

		add(DEFAULT, Commands::none);
		activePub.set(activeName);

		CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::update);
	}

	/**
	 * Adds an option to the chooser.
	 *
	 * @param name
	 *          display name (must be unique)
	 * @param factory
	 *          supplier that creates a fresh command each time auto starts
	 */
	public void add(String name, Supplier<Command> factory) {
		options.put(name, factory);
		optionsPub.set(options.keySet().toArray(String[]::new));
	}

	/**
	 * Sets the default selected option. Call this before any auto period starts.
	 *
	 * @param name
	 *          display name of the option to select by default
	 */
	public void setDefault(String name) {
		if (options.containsKey(name)) {
			activeName = name;
			activePub.set(activeName);
		}
	}

	private void update() {
		if (!running) {
			String selected = selectedSub.get();
			if (!selected.equals(activeName)) {
				activeName = options.containsKey(selected) ? selected : DEFAULT;
				activePub.set(activeName);
			}
		}

		boolean schedule = DriverStation.isAutonomousEnabled();
		if (!running && schedule) {
			activeCommand = options.get(activeName).get();
			CommandScheduler.getInstance().schedule(activeCommand);
			running = true;
		} else if (running && !schedule) {
			if (activeCommand != null) {
				activeCommand.cancel();
			}
			activeCommand = null;
			running = false;
		}
	}
}
