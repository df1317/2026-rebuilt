package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.DevMode;
import frc.robot.util.HubTracker;

public class Robot extends TimedRobot {

	private final double[] loopTimesMs = new double[50];
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	private Timer disabledTimer;
	// Loop timing (dev mode only)
	private long lastLoopTimeMicros = 0;
	private int loopIndex = 0;

	@Override
	public void robotInit() {
		DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withLogExtras(true));
		m_robotContainer = new RobotContainer();
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
		SmartDashboard.putData("Robot/PDH", new PowerDistribution());
		SmartDashboard.putData("Robot/Scheduler", CommandScheduler.getInstance());
		disabledTimer = new Timer();

		if (isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
	}

	@Override
	public void robotPeriodic() {
		if (DevMode.isEnabled()) {
			long nowMicros = RobotController.getFPGATime();

			if (lastLoopTimeMicros != 0) {
				double loopMs = (nowMicros - lastLoopTimeMicros) / 1000.0;
				loopTimesMs[loopIndex % loopTimesMs.length] = loopMs;
				DogLog.log("Robot/LoopTimeMs", loopMs);

				if (loopIndex > 0 && loopIndex % loopTimesMs.length == 0) {
					double sum = 0;
					for (double time : loopTimesMs) {
						sum += time;
					}
					DogLog.log("Robot/AvgLoopTimeMs", sum / loopTimesMs.length);
				}
				loopIndex++;
			}
			lastLoopTimeMicros = nowMicros;
		}

		HubTracker.periodic();

		// Update Repulsor path planner state before CommandScheduler
		m_robotContainer.updateRepulsor();

		long schedulerStartMicros = RobotController.getFPGATime();
		CommandScheduler.getInstance().run();
		long schedulerEndMicros = RobotController.getFPGATime();

		if (DevMode.isEnabled()) {
			DogLog.log("Robot/SchedulerTimeMs", (schedulerEndMicros - schedulerStartMicros) / 1000.0);
		}

		// Elastic dashboard essentials (always published even at competition)
		DogLog.forceNt.log("Dash/MatchTime", DriverStation.getMatchTime());
		DogLog.forceNt.log("Dash/RobotRelative", m_robotContainer.robotRelative);
		DogLog.forceNt.log("Dash/HubStatusColor", HubTracker.getHubStatusColor().toHexString());
	}

	@Override
	public void disabledInit() {
		m_robotContainer.setMotorBrake(true);
		disabledTimer.reset();
		disabledTimer.start();
		HubTracker.reset();
	}

	@Override
	public void disabledPeriodic() {
		if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
			m_robotContainer.setMotorBrake(false);
			disabledTimer.stop();
			disabledTimer.reset();
		}
	}

	@Override
	public void autonomousInit() {
		m_robotContainer.setMotorBrake(true);
		m_robotContainer.autonomousInit();
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		HubTracker.start();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		} else {
			CommandScheduler.getInstance().cancelAll();
		}
	}

	@Override
	public void testInit() {
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
