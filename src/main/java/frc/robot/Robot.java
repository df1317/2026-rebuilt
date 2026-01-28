package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.jni.AHRSJNI;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.DevMode;
import frc.robot.util.HubState;

/**
 * ---------- Robot --- The VM is configured to automatically run this class and
 * to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project. ---
 */
public class Robot extends TimedRobot {

	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	private Timer disabledTimer;

	// Loop timing measurement (dev mode only)
	private long lastLoopTimeMicros = 0;
	private double[] loopTimesMs = new double[50]; // Rolling buffer for 1 second of data
	private int loopIndex = 0;

	/**
	 * ---------- RobotInit --- This function is run when the robot is first started
	 * up and should be used for any
	 * initialization code. ---
	 */
	@Override
	public void robotInit() {
		// Configure DogLog for logging
		// - Logs always go to DataLog (.wpilog) for AdvantageScope post-match analysis
		// - NT publishing auto-disables when FMS connected (competition mode)
		// - captureDs: logs joystick/DS data
		// - logExtras: logs PDH currents, CAN usage, battery voltage, etc.
		DogLog.setOptions(new DogLogOptions()
				.withCaptureDs(true)
				.withLogExtras(true));

		// Instantiate our RobotContainer. This will perform all our button bindings and
		// put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();

		// create a webserver for elastic dashboard layouts
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

		// Create a timer to disable motor brake a few seconds after disable
		// This will let the robot stop immediately when disabled, but then also let it
		// be pushed more
		disabledTimer = new Timer();

		if (isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}

		// Initiate new NavX to type kUSB1
		AHRSJNI.c_AHRS_create(AHRS.NavXComType.kUSB1);
	}

	/**
	 * ---------- RobotPeriodic --- This function is called every 20 ms, no matter
	 * the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and
	 * test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating. ---
	 */
	@Override
	public void robotPeriodic() {
		// Measure loop timing (dev mode only)
		if (DevMode.isEnabled()) {
			long nowMicros = RobotController.getFPGATime();

			if (lastLoopTimeMicros != 0) {
				double loopMs = (nowMicros - lastLoopTimeMicros) / 1000.0; // Convert μs to ms
				loopTimesMs[loopIndex % loopTimesMs.length] = loopMs;

				// Log individual loop time
				DogLog.log("Robot/LoopTimeMs", loopMs);

				// Calculate and log rolling average every 50 loops (~1 second)
				if (loopIndex > 0 && loopIndex % loopTimesMs.length == 0) {
					double sum = 0;
					for (double time : loopTimesMs) {
						sum += time;
					}
					double avgLoopMs = sum / loopTimesMs.length;
					DogLog.log("Robot/AvgLoopTimeMs", avgLoopMs);
				}

				loopIndex++;
			}

			lastLoopTimeMicros = nowMicros;
		}

		/*
		 * ----------
		 * Runs the Scheduler. This is responsible for polling buttons, adding newly
		 * scheduled
		 * commands, running already-scheduled commands, removing finished or
		 * interrupted commands,
		 * and running subsystem periodic() methods. This must be called from the
		 * robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */
		long schedulerStartMicros = RobotController.getFPGATime();
		CommandScheduler.getInstance().run();
		long schedulerEndMicros = RobotController.getFPGATime();

		// Log CommandScheduler overhead (dev mode only)
		if (DevMode.isEnabled()) {
			double schedulerMs = (schedulerEndMicros - schedulerStartMicros) / 1000.0;
			DogLog.log("Robot/SchedulerTimeMs", schedulerMs);
		}

		// Essential values for Elastic dashboard - forceNt ensures these are always
		// published
		// even at competition (when regular NT publishing is disabled)
		DogLog.forceNt.log("Dash/MatchTime", DriverStation.getMatchTime());
		DogLog.forceNt.log("Dash/RobotRelative", m_robotContainer.robotRelative);
		DogLog.forceNt.log("Dash/HubStatusColor", HubState.getHubStatusColor().toHexString());

		// Log NavX gyro values
		DogLog.log("gyro yaw", AHRSJNI.c_AHRS_GetYaw());
		DogLog.log("gyro roll", AHRSJNI.c_AHRS_GetRoll());
		DogLog.log("gyro pitch", AHRSJNI.c_AHRS_GetPitch());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		m_robotContainer.setMotorBrake(true);
		disabledTimer.reset();
		disabledTimer.start();
	}

	/**
	 * ---------- DisabledPeriodic --- This function is called periodically while
	 * the robot is in disabled mode. ---
	 */
	@Override
	public void disabledPeriodic() {
		if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
			m_robotContainer.setMotorBrake(false);
			disabledTimer.stop();
			disabledTimer.reset();
		}
	}

	/**
	 * ---------- AutonomousInit --- This autonomous runs the autonomous command
	 * selected by your {@link RobotContainer}
	 * class. ---
	 */
	@Override
	public void autonomousInit() {
		m_robotContainer.setMotorBrake(true);
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	/**
	 * ---------- TeleopInit --- This function is called at the beginning of
	 * operator control. ---
	 */
	@Override
	public void teleopInit() {
		/*
		 * ----------
		 * This makes sure that the autonomous stops running when
		 * teleop starts running. If you want the autonomous to
		 * continue until interrupted by another command, remove
		 * this line or comment it out.
		 */
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		} else {
			CommandScheduler.getInstance().cancelAll();
		}
	}

	/**
	 * ---------- TestInit --- This function is called periodically during test
	 * mode. ---
	 */
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testPeriodic() {
	}
}
