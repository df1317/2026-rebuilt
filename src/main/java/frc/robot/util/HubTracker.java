package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Tracks hub scoring eligibility based on match phase timing and auto winner.
 * Call start() in autonomousInit(), periodic() in robotPeriodic(), reset() in disabledInit().
 */
public class HubTracker {

	private static final double MIN_SHOOT_TIME = 0.8;
	private static final double BUFFER_TIME = 3.0;

	private static final Timer phaseTimer = new Timer();

	private static MatchPhase currentPhase = MatchPhase.UNKNOWN;
	private static MatchPhase previousPhase = MatchPhase.UNKNOWN;
	private static Alliance autoWinner = null;
	private static Alliance ourAlliance = null;

	private HubTracker() {
	}

	public static void start() {
		phaseTimer.reset();
		phaseTimer.start();
		currentPhase = MatchPhase.getCurrent();
		previousPhase = currentPhase;
	}

	public static void reset() {
		phaseTimer.stop();
		phaseTimer.reset();
		currentPhase = MatchPhase.UNKNOWN;
		previousPhase = MatchPhase.UNKNOWN;
		autoWinner = null;
		ourAlliance = null;
	}

	public static void periodic() {
		var allianceOpt = DriverStation.getAlliance();
		allianceOpt.ifPresent(alliance -> ourAlliance = alliance);

		if (autoWinner == null) {
			String gameData = DriverStation.getGameSpecificMessage();
			if (gameData != null && !gameData.isEmpty()) {
				char gameChar = gameData.charAt(0);
				if (gameChar == 'R') {
					autoWinner = Alliance.Red;
				} else if (gameChar == 'B') {
					autoWinner = Alliance.Blue;
				}
			}
		}

		previousPhase = currentPhase;
		currentPhase = MatchPhase.getCurrent();

		if (currentPhase != previousPhase) {
			phaseTimer.reset();
			phaseTimer.start();
		}

		DogLog.forceNt.log("Hub/StatusColor", getHubStatusColor().toHexString());
		DogLog.forceNt.log("Hub/Status", getHubStatus().name());
		DogLog.forceNt.log("Hub/Phase", currentPhase.name());
		DogLog.forceNt.log("Hub/PhaseTimeRemaining", getPhaseRemainingTime());
		DogLog.forceNt.log("Hub/CanScore", canScore());
	}

	public static boolean canScore() {
		if (ourAlliance == null)
			return false;
		return canScoreInPhase(currentPhase, phaseTimer.get(), ourAlliance == autoWinner);
	}

	/** True if scoring is allowed now or will be within BUFFER_TIME seconds. */
	public static boolean canScoreBuffered() {
		if (canScore())
			return true;
		return willBeAbleToScoreAt(phaseTimer.get() + BUFFER_TIME);
	}

	private static boolean willBeAbleToScoreAt(double futurePhaseTime) {
		if (ourAlliance == null)
			return false;

		boolean weWonAuto = ourAlliance == autoWinner;
		int phaseDuration = currentPhase.getDuration();

		if (futurePhaseTime >= phaseDuration) {
			MatchPhase nextPhase = currentPhase.getNext();
			if (nextPhase == null)
				return true;
			return canScoreInPhase(nextPhase, futurePhaseTime - phaseDuration, weWonAuto);
		}

		return canScoreInPhase(currentPhase, futurePhaseTime, weWonAuto);
	}

	private static boolean canScoreInPhase(MatchPhase phase, double phaseTime, boolean weWonAuto) {
		int phaseDuration = phase.getDuration();

		return switch (phase) {
			case AUTO, END_GAME -> true;
			case TRANSITION -> !weWonAuto || phaseTime >= MIN_SHOOT_TIME;
			case SHIFT_1, SHIFT_3 -> weWonAuto
					? phaseTime <= phaseDuration - MIN_SHOOT_TIME
					: phaseTime >= MIN_SHOOT_TIME;
			case SHIFT_2, SHIFT_4 -> weWonAuto
					? phaseTime >= MIN_SHOOT_TIME
					: phaseTime <= phaseDuration - MIN_SHOOT_TIME;
			case UNKNOWN -> false;
		};
	}

	public static HubStatus getHubStatus() {
		if (canScore())
			return HubStatus.ACTIVE;
		if (canScoreBuffered())
			return HubStatus.BUFFERED;
		return HubStatus.NOT_AVAILABLE;
	}

	public static Color getHubStatusColor() {
		return switch (getHubStatus()) {
			case ACTIVE -> RobotLog.GREEN;
			case BUFFERED -> RobotLog.YELLOW;
			case NOT_AVAILABLE -> RobotLog.RED;
		};
	}

	public static MatchPhase getCurrentPhase() {
		return currentPhase;
	}

	public static double getPhaseElapsedTime() {
		return phaseTimer.get();
	}

	public static double getPhaseRemainingTime() {
		return Math.max(0, currentPhase.getDuration() - phaseTimer.get());
	}

	public static Alliance getAutoWinner() {
		return autoWinner;
	}

	public static boolean weWonAuto() {
		return ourAlliance != null && ourAlliance == autoWinner;
	}

	public static boolean hasGameData() {
		String gameData = DriverStation.getGameSpecificMessage();
		return gameData != null && !gameData.isEmpty();
	}

	public enum HubStatus {
		NOT_AVAILABLE, BUFFERED, ACTIVE
	}
}
