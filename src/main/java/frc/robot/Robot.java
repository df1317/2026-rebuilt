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
import frc.robot.util.HubTracker;

public class Robot extends TimedRobot {

	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	private Timer disabledTimer;

	// Loop timing (dev mode only)
	private long lastLoopTimeMicros = 0;
	private double[] loopTimesMs = new double[50];
	private int loopIndex = 0;

	@Override
	public void robotInit() {
		DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withLogExtras(true));
		m_robotContainer = new RobotContainer();
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
		disabledTimer = new Timer();

		if (isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}

		AHRSJNI.c_AHRS_create(AHRS.NavXComType.kUSB1);
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

		DogLog.log("gyro yaw", AHRSJNI.c_AHRS_GetYaw());
		DogLog.log("gyro roll", AHRSJNI.c_AHRS_GetRoll());
		DogLog.log("gyro pitch", AHRSJNI.c_AHRS_GetPitch());
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
