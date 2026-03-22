package frc.robot;

import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopZoneAutomation;
import frc.robot.repulsor.DriveRepulsor;
import frc.robot.repulsor.Repulsor;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Integration test that constructs real subsystems in HAL simulation mode and validates all command
 * compositions using WPILib's own runtime checks.
 *
 * <p>
 * Layers:
 * <ol>
 * <li><b>Requirement declaration</b>: every subsystem command factory must declare its subsystem
 * <li><b>Auto command crawl</b>: builds the real auto chooser, reflectively pulls every option, initializes
 * deferred commands, and recursively walks ConditionalCommand branches
 * <li><b>Teleop compositions</b>: validates TeleopZoneAutomation compositions
 * <li><b>Cross-subsystem pairs</b>: exhaustive pairwise parallel validation
 * </ol>
 */
class CommandIntegrationTest {

	private static RollerSubsystem roller;
	private static IntakeSubsystem intake;
	private static ShooterSubsystem shooter;
	private static HopperSubsystem hopper;
	private static Repulsor repulsor;
	private static SubsystemBase mockDriveSubsystem;
	private static TeleopZoneAutomation teleopAutomation;

	@BeforeAll
	static void init() {
		HAL.initialize(500, 0);
		roller = new RollerSubsystem();
		intake = new IntakeSubsystem(roller);
		shooter = new ShooterSubsystem();
		hopper = new HopperSubsystem();

		mockDriveSubsystem = new SubsystemBase() {
		};
		DriveRepulsor mockDrive = new DriveRepulsor() {

			@Override
			public void runVelocity(ChassisSpeeds speeds) {
			}

			@Override
			public void lock() {
			}

			@Override
			public Pose2d getPose() {
				return new Pose2d();
			}

			@Override
			public PIDController getOmegaPID() {
				return new PIDController(1, 0, 0);
			}

			@Override
			public SubsystemBase asSubsystem() {
				return mockDriveSubsystem;
			}
		};
		repulsor = new Repulsor(mockDrive, 0.35, 0.35);
		teleopAutomation = new TeleopZoneAutomation(
				repulsor, intake, shooter, hopper, Pose2d::new, ChassisSpeeds::new);
	}

	@BeforeEach
	void resetScheduler() {
		CommandScheduler.getInstance().cancelAll();
		CommandScheduler.getInstance().clearComposedCommands();
	}

	// ========================================================================================
	// Layer 1: Every subsystem command must declare its subsystem requirement
	// ========================================================================================

	@Test
	void allSubsystemCommandsDeclareRequirements() {
		List<String> errors = new ArrayList<>();
		checkSubsystemRequirements(intake, "IntakeSubsystem", errors);
		checkSubsystemRequirements(roller, "RollerSubsystem", errors);
		checkSubsystemRequirements(shooter, "ShooterSubsystem", errors);
		checkSubsystemRequirements(hopper, "HopperSubsystem", errors);

		if (!errors.isEmpty()) {
			fail("Found command(s) missing subsystem requirements:\n\n"
					+ String.join("\n", errors));
		}
	}

	private void checkSubsystemRequirements(SubsystemBase subsystem, String name,
			List<String> errors) {
		for (Method method : subsystem.getClass().getMethods()) {
			if (!Command.class.isAssignableFrom(method.getReturnType()))
				continue;
			if (method.getParameterCount() > 0)
				continue;
			if (!Modifier.isPublic(method.getModifiers()))
				continue;
			if (method.getDeclaringClass() == Object.class
					|| method.getDeclaringClass() == SubsystemBase.class)
				continue;

			try {
				Command cmd = (Command) method.invoke(subsystem);
				if (cmd != null && !cmd.getRequirements().contains(subsystem)) {
					errors.add(String.format(
							"  %s.%s() — returns Command \"%s\" but doesn't require %s\n"
									+ "    Fix: use runOnce()/run() instead of Commands.runOnce()/Commands.run()",
							name, method.getName(), cmd.getName(), name));
				}
			} catch (Exception e) {
				// Some commands may throw in sim — skip
			}
		}
	}

	// ========================================================================================
	// Layer 2: Build the real auto chooser and validate every option automatically
	// ========================================================================================

	/**
	 * Constructs a real Autos instance, then reflectively invokes every private
	 * Command-returning method (the auto routine factories). Each returned command
	 * is validated via the composition tree walker.
	 * Any IllegalArgumentException from WPILib means a subsystem conflict exists.
	 */
	@Test
	void allAutoChooserOptionsAreValid() {
		List<String> errors = new ArrayList<>();

		// Build Autos the same way RobotContainer does (null drivebase/climber are safe)
		Autos autos = new Autos(repulsor, null, teleopAutomation, null, intake, roller);

		// Reflectively invoke every Command-returning method on Autos
		for (Method method : Autos.class.getDeclaredMethods()) {
			if (!Command.class.isAssignableFrom(method.getReturnType()))
				continue;
			if (method.getParameterCount() > 0)
				continue;

			method.setAccessible(true);
			String name = method.getName();
			try {
				Command cmd = (Command) method.invoke(autos);
				validateCommandTree("Auto[" + name + "]", cmd, errors, new HashSet<>());
			} catch (java.lang.reflect.InvocationTargetException e) {
				Throwable cause = e.getCause();
				if (cause instanceof IllegalArgumentException) {
					errors.add(String.format("  Auto[%s] — threw on construction:\n    %s",
							name, cause.getMessage()));
				}
				// Other exceptions (e.g. NPE in sim) — skip
			} catch (Exception e) {
				// Skip methods that can't run in sim
			}
		}

		if (!errors.isEmpty()) {
			fail("Found " + errors.size() + " auto command composition error(s):\n\n"
					+ String.join("\n\n", errors));
		}
	}

	// ========================================================================================
	// Layer 3: Teleop compositions through real TeleopZoneAutomation
	// ========================================================================================

	@Test
	void allTeleopCompositionsAreValid() {
		List<String> errors = new ArrayList<>();

		validateComposition("TeleopZoneAutomation.shootCommand()", errors,
				teleopAutomation::shootCommand);
		validateComposition("TeleopZoneAutomation.shootCommand(aimed)", errors,
				() -> teleopAutomation.shootCommand(() -> true));

		// Test panel shooter composition
		validateComposition("Test panel shooter", errors, () -> Commands.parallel(
				Commands.runOnce(() -> shooter.setTestHoodPercent()),
				shooter.spinUpAndWaitCommand(shooter::getShooterTestRPM,
						shooter::getFeederTestRPM)));

		if (!errors.isEmpty()) {
			fail("Found " + errors.size() + " teleop command composition error(s):\n\n"
					+ String.join("\n\n", errors));
		}
	}

	// ========================================================================================
	// Layer 4: Cross-subsystem pair check
	// ========================================================================================

	@Test
	void crossSubsystemCommandsCanParallel() {
		List<String> errors = new ArrayList<>();

		SubsystemBase[] subsystems = { intake, roller, shooter, hopper };
		String[] names = { "intake", "roller", "shooter", "hopper" };

		List<List<NamedCommand>> allCommands = new ArrayList<>();
		for (int i = 0; i < subsystems.length; i++) {
			allCommands.add(getNoArgCommands(subsystems[i], names[i]));
		}

		for (int i = 0; i < allCommands.size(); i++) {
			for (int j = i + 1; j < allCommands.size(); j++) {
				for (NamedCommand a : allCommands.get(i)) {
					for (NamedCommand b : allCommands.get(j)) {
						Command cmdA = null;
						Command cmdB = null;
						try {
							cmdA = a.supplier.get();
							cmdB = b.supplier.get();
							if (cmdA == null || cmdB == null)
								continue;
							Commands.parallel(cmdA, cmdB);
						} catch (IllegalArgumentException e) {
							Set<String> shared = new HashSet<>();
							if (cmdA != null && cmdB != null) {
								for (var req : cmdA.getRequirements()) {
									if (cmdB.getRequirements().contains(req)) {
										shared.add(req.getClass().getSimpleName());
									}
								}
							}
							errors.add(String.format("  %s + %s — both require: %s",
									a.name, b.name, shared));
						} catch (Exception e) {
							// Skip commands that can't be constructed in sim
						}
					}
				}
			}
		}

		if (!errors.isEmpty()) {
			fail("Found " + errors.size()
					+ " cross-subsystem conflict(s):\n\n" + String.join("\n", errors));
		}
	}

	// ========================================================================================
	// Self-test
	// ========================================================================================

	@Test
	void selfTestCatchesConflict() {
		try {
			Commands.parallel(intake.extendCommand(), intake.holdExtendedCommand());
			fail("Expected IllegalArgumentException — two intake commands in parallel should conflict");
		} catch (IllegalArgumentException e) {
			// Expected
		}
	}

	/** Proves the tree walker catches a conflict buried inside a DeferredCommand. */
	@Test
	void selfTestCatchesDeferredConflict() {
		Command bad = Commands.defer(
				() -> Commands.parallel(intake.extendCommand(), intake.holdExtendedCommand()),
				Set.of());
		List<String> errors = new ArrayList<>();
		validateCommandTree("selfTest", bad, errors, new HashSet<>());
		if (errors.isEmpty()) {
			fail("Expected the tree walker to catch a conflict inside a DeferredCommand");
		}
	}

	// ========================================================================================
	// Command tree walker — recursively validates all compositions via reflection
	// ========================================================================================

	/**
	 * Recursively walks a command tree, validating each node. Handles:
	 * <ul>
	 * <li>DeferredCommand — calls m_supplier.get() to construct inner command, then walks it
	 * <li>ConditionalCommand (Commands.either) — walks both m_onTrue and m_onFalse branches
	 * <li>SelectCommand (Commands.select) — walks all m_commands values
	 * <li>ParallelCommandGroup / ParallelDeadlineGroup / ParallelRaceGroup — already validated by
	 * WPILib at construction, but we walk children for nested compositions
	 * <li>SequentialCommandGroup — walks all children
	 * </ul>
	 */
	private void validateCommandTree(String path, Command cmd, List<String> errors,
			Set<Command> visited) {
		if (cmd == null || !visited.add(cmd))
			return;

		String className = cmd.getClass().getSimpleName();

		// DeferredCommand — force construction of inner command
		if (className.equals("DeferredCommand")) {
			try {
				Supplier<?> supplier = getField(cmd, "m_supplier");
				if (supplier != null) {
					Command inner = (Command) supplier.get();
					validateCommandTree(path + " → deferred", inner, errors, visited);
				}
			} catch (IllegalArgumentException e) {
				errors.add(String.format(
						"  %s → DeferredCommand threw on construction:\n"
								+ "    %s\n"
								+ "    This means a Commands.parallel() or .deadlineFor() inside this deferred\n"
								+ "    command has two commands requiring the same subsystem.",
						path, e.getMessage()));
			} catch (Exception e) {
				errors.add(String.format("  %s → DeferredCommand: %s: %s",
						path, e.getClass().getSimpleName(), e.getMessage()));
			}
		}

		// ConditionalCommand (Commands.either) — validate both branches
		if (className.equals("ConditionalCommand")) {
			Command onTrue = getField(cmd, "m_onTrue");
			Command onFalse = getField(cmd, "m_onFalse");
			validateCommandTree(path + " → either(true)", onTrue, errors, visited);
			validateCommandTree(path + " → either(false)", onFalse, errors, visited);
		}

		// SelectCommand (Commands.select) — validate all options
		if (className.equals("SelectCommand")) {
			Map<?, Command> options = getField(cmd, "m_commands");
			if (options != null) {
				for (Map.Entry<?, Command> entry : options.entrySet()) {
					validateCommandTree(path + " → select(" + entry.getKey() + ")",
							entry.getValue(), errors, visited);
				}
			}
		}

		// Walk children of sequential/parallel groups
		walkGroupChildren(path, cmd, "m_commands", errors, visited);
	}

	private void walkGroupChildren(String path, Command cmd, String fieldName,
			List<String> errors, Set<Command> visited) {
		try {
			Object children = getField(cmd, fieldName);
			if (children instanceof List<?> list) {
				for (int i = 0; i < list.size(); i++) {
					Object child = list.get(i);
					if (child instanceof Command childCmd) {
						validateCommandTree(path + "[" + i + "]", childCmd, errors, visited);
					}
				}
			} else if (children instanceof Map<?, ?> map) {
				for (Object child : map.keySet()) {
					if (child instanceof Command childCmd) {
						validateCommandTree(path + " → parallel", childCmd, errors, visited);
					}
				}
			}
		} catch (Exception e) {
			// Field doesn't exist on this command type — that's fine
		}
	}

	// ========================================================================================
	// Reflection helpers
	// ========================================================================================

	@SuppressWarnings("unchecked")
	private static <T> T getField(Object obj, String fieldName) {
		try {
			Class<?> clazz = obj.getClass();
			while (clazz != null) {
				try {
					Field field = clazz.getDeclaredField(fieldName);
					field.setAccessible(true);
					return (T) field.get(obj);
				} catch (NoSuchFieldException e) {
					clazz = clazz.getSuperclass();
				}
			}
		} catch (Exception e) {
			// Field not found — return null
		}
		return null;
	}


	// ========================================================================================
	// Other helpers
	// ========================================================================================

	private static class NamedCommand {

		final String name;
		final Supplier<Command> supplier;

		NamedCommand(String name, Supplier<Command> supplier) {
			this.name = name;
			this.supplier = supplier;
		}
	}

	private List<NamedCommand> getNoArgCommands(SubsystemBase subsystem, String subsystemName) {
		List<NamedCommand> commands = new ArrayList<>();
		Set<String> seen = new HashSet<>();

		for (Method method : subsystem.getClass().getMethods()) {
			if (!Command.class.isAssignableFrom(method.getReturnType()))
				continue;
			if (method.getParameterCount() > 0)
				continue;
			if (!Modifier.isPublic(method.getModifiers()))
				continue;
			if (method.getDeclaringClass() == Object.class
					|| method.getDeclaringClass() == SubsystemBase.class)
				continue;
			if (!seen.add(method.getName()))
				continue;

			String name = subsystemName + "." + method.getName() + "()";
			Method m = method;
			commands.add(new NamedCommand(name, () -> {
				try {
					return (Command) m.invoke(subsystem);
				} catch (Exception e) {
					return null;
				}
			}));
		}
		return commands;
	}

	private void validateComposition(String name, List<String> errors,
			Supplier<Command> factory) {
		try {
			Command cmd = factory.get();
			cmd.getRequirements();
		} catch (IllegalArgumentException e) {
			errors.add(String.format(
					"  %s\n    %s\n    This parallel group has two commands requiring the same subsystem.",
					name, e.getMessage()));
		} catch (Exception e) {
			errors.add(String.format("  %s — %s: %s",
					name, e.getClass().getSimpleName(), e.getMessage()));
		}
	}
}
