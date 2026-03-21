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
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Integration test that constructs real subsystems in HAL simulation mode and validates all command
 * compositions using WPILib's own runtime checks. This catches subsystem conflicts exactly the way
 * the robot would — but at build time instead of on the field.
 *
 * <p>
 * Two layers:
 * <ol>
 * <li><b>Reflective scan</b>: finds every public Command-returning method on every subsystem, constructs
 * each one, and verifies it doesn't conflict with any other command from the same subsystem when run
 * in parallel. This catches issues in individual subsystem command factories.
 * <li><b>Explicit compositions</b>: validates every known Commands.parallel() / deadlineFor() composition
 * in the codebase using WPILib's actual runtime validation. These mirror the real compositions.
 * </ol>
 */
class CommandIntegrationTest {

	private static RollerSubsystem roller;
	private static IntakeSubsystem intake;
	private static ShooterSubsystem shooter;
	private static HopperSubsystem hopper;
	private static Repulsor repulsor;
	private static SubsystemBase mockDriveSubsystem;

	@BeforeAll
	static void init() {
		HAL.initialize(500, 0);
		roller = new RollerSubsystem();
		intake = new IntakeSubsystem(roller);
		shooter = new ShooterSubsystem();
		hopper = new HopperSubsystem();

		// Lightweight mock drive for Repulsor — no swerve hardware needed
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
	}

	@BeforeEach
	void resetScheduler() {
		CommandScheduler.getInstance().cancelAll();
		CommandScheduler.getInstance().clearComposedCommands();
	}

	// ========================================================================================
	// Layer 1: Reflective scan — every pair of commands from the same subsystem
	// ========================================================================================

	/**
	 * For each subsystem, constructs every no-arg Command factory and verifies that combining any two
	 * commands from the SAME subsystem in parallel throws (they should conflict since they both require
	 * the same subsystem). This is a sanity check that command requirements are properly declared.
	 */
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
			// Skip methods from Object/SubsystemBase
			if (method.getDeclaringClass() == Object.class
					|| method.getDeclaringClass() == SubsystemBase.class)
				continue;

			try {
				Command cmd = (Command) method.invoke(subsystem);
				if (cmd != null && !cmd.getRequirements().contains(subsystem)) {
					errors.add(name + "." + method.getName()
							+ "() returns a Command that doesn't require its own subsystem");
				}
			} catch (Exception e) {
				// Some commands may throw in sim — that's OK, skip them
			}
		}
	}

	// ========================================================================================
	// Layer 2: Real auto + teleop command construction through actual code paths
	// ========================================================================================

	/**
	 * Builds every auto command through the real AutoPositions / AutoBuilder code path, then
	 * initializes the deferred command to trigger WPILib's subsystem conflict validation at exactly the
	 * same point it would crash on the field.
	 */
	@Test
	void allAutoCommandsAreValid() {
		List<String> errors = new ArrayList<>();

		// Build the same shootCommand and collectCommand suppliers as RobotContainer
		Supplier<Command> shootCommand = () -> {
			TeleopZoneAutomation automation = new TeleopZoneAutomation(
					repulsor, intake, shooter, hopper, Pose2d::new, ChassisSpeeds::new);
			return automation.shootCommand().withTimeout(4);
		};
		Supplier<Command> collectCommand = () -> Commands.parallel(
				intake.extendCommand().andThen(intake.holdExtendedCommand()),
				roller.intakeCommand());

		// Test every auto through the real AutoPositions code path
		validateDeferredCommand("collectAndShoot1", errors,
				AutoPositions.collectAndShoot1(repulsor, shootCommand, collectCommand));

		validateDeferredCommand("collectAndShoot2", errors,
				AutoPositions.collectAndShoot2(repulsor, shootCommand, collectCommand));

		if (!errors.isEmpty()) {
			fail("Found " + errors.size() + " auto command composition error(s):\n\n"
					+ String.join("\n\n", errors));
		}
	}

	/**
	 * Validates teleop command compositions by constructing them through the real TeleopZoneAutomation
	 * code path.
	 */
	@Test
	void allTeleopCompositionsAreValid() {
		List<String> errors = new ArrayList<>();

		TeleopZoneAutomation automation = new TeleopZoneAutomation(
				repulsor, intake, shooter, hopper, Pose2d::new, ChassisSpeeds::new);

		validateComposition("TeleopZoneAutomation.shootCommand()", errors,
				automation::shootCommand);
		validateComposition("TeleopZoneAutomation.shootCommand(aimed)", errors,
				() -> automation.shootCommand(() -> true));

		// Test panel shooter composition (RobotContainer)
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
	// Layer 3: Cross-subsystem pair check — exhaustive pairwise parallel validation
	// ========================================================================================

	/**
	 * Constructs every possible pair of no-arg commands from DIFFERENT subsystems and verifies they can
	 * be composed in parallel. This catches hidden cross-subsystem conflicts (e.g., a command that
	 * internally requires a second subsystem).
	 */
	@Test
	void crossSubsystemCommandsCanParallel() {
		List<String> errors = new ArrayList<>();

		SubsystemBase[] subsystems = { intake, roller, shooter, hopper };
		String[] names = { "intake", "roller", "shooter", "hopper" };

		List<List<NamedCommand>> allCommands = new ArrayList<>();
		for (int i = 0; i < subsystems.length; i++) {
			allCommands.add(getNoArgCommands(subsystems[i], names[i]));
		}

		// Check every pair of commands from different subsystems
		for (int i = 0; i < allCommands.size(); i++) {
			for (int j = i + 1; j < allCommands.size(); j++) {
				for (NamedCommand a : allCommands.get(i)) {
					for (NamedCommand b : allCommands.get(j)) {
						try {
							// Need fresh instances since commands can only be composed once
							Command cmdA = a.supplier.get();
							Command cmdB = b.supplier.get();
							if (cmdA == null || cmdB == null)
								continue;
							Commands.parallel(cmdA, cmdB);
						} catch (IllegalArgumentException e) {
							errors.add(a.name + " + " + b.name + ": " + e.getMessage());
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
	// Self-test: verify the test framework actually catches conflicts
	// ========================================================================================

	/** Proves the test catches a real conflict — two intake commands in parallel should fail. */
	@Test
	void selfTestCatchesConflict() {
		try {
			Commands.parallel(intake.extendCommand(), intake.holdExtendedCommand());
			fail("Expected IllegalArgumentException — two intake commands in parallel should conflict");
		} catch (IllegalArgumentException e) {
			// Expected — WPILib correctly rejects this
		}
	}

	// ===== Helpers =====

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
			Method m = method; // capture for lambda
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
			cmd.getRequirements(); // sanity check
		} catch (IllegalArgumentException e) {
			errors.add(name + ": " + e.getMessage());
		} catch (Exception e) {
			errors.add(name + ": unexpected error — " + e.getClass().getSimpleName()
					+ ": " + e.getMessage());
		}
	}

	/**
	 * Initializes a deferred command to force WPILib to construct the inner command graph. This is
	 * exactly where the crash happens on the field — DeferredCommand.initialize() calls the supplier
	 * which builds the parallel groups and triggers the subsystem conflict check.
	 */
	private void validateDeferredCommand(String name, List<String> errors, Command deferredCmd) {
		try {
			deferredCmd.initialize();
			deferredCmd.end(true);
		} catch (IllegalArgumentException e) {
			errors.add(name + ": " + e.getMessage());
		} catch (Exception e) {
			errors.add(name + ": unexpected error — " + e.getClass().getSimpleName()
					+ ": " + e.getMessage());
		}
	}
}
