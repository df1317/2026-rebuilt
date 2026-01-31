package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Stateful tracker for hub scoring eligibility based on match phase timing and which alliance won autonomous.
 *
 * <p>This class tracks the current match phase, elapsed time within each phase, and determines
 * whether scoring is allowed based on the alternating shift schedule and auto winner.
 *
 * <p>Lifecycle methods must be called from Robot:
 * <ul>
 *   <li>{@link #start()} - Call in autonomousInit()
 *   <li>{@link #periodic()} - Call in robotPeriodic()
 *   <li>{@link #reset()} - Call in disabledInit()
 * </ul>
 *
 * <p>Scoring rules:
 * <ul>
 *   <li>AUTO and END_GAME: Both alliances can always score
 *   <li>TRANSITION: Auto loser can score immediately; auto winner must wait MIN_SHOOT_TIME
 *   <li>SHIFT_1, SHIFT_3: Auto winner scores first, then auto loser
 *   <li>SHIFT_2, SHIFT_4: Auto loser scores first, then auto winner
 * </ul>
 */
public class HubTracker {

	/** Minimum time buffer (seconds) to avoid scoring at phase boundaries. */
	private static final double MIN_SHOOT_TIME = 0.8;

	/** Buffer time (seconds) for lookahead scoring predictions. */
	private static final double BUFFER_TIME = 3.0;

	private static final Timer phaseTimer = new Timer();
	private static final Color COLOR_RED = new Color(255, 0, 0);
	private static final Color COLOR_YELLOW = new Color(255, 255, 0);
	private static final Color COLOR_GREEN = new Color(0, 255, 0);

	private static MatchPhase currentPhase = MatchPhase.UNKNOWN;
	private static MatchPhase previousPhase = MatchPhase.UNKNOWN;
	private static Alliance autoWinner = null;
	private static Alliance ourAlliance = null;

	private HubTracker() {
	}

	/**
	 * Starts the hub tracker. Call this in autonomousInit().
	 *
	 * <p>Resets and starts the phase timer and initializes the current phase.
	 */
	public static void start() {
		phaseTimer.reset();
		phaseTimer.start();
		currentPhase = MatchPhase.getCurrent();
		previousPhase = currentPhase;
	}

	/**
	 * Resets the hub tracker to its initial state. Call this in disabledInit().
	 *
	 * <p>Stops the timer and clears all tracked states including alliance and auto winner.
	 */
	public static void reset() {
		phaseTimer.stop();
		phaseTimer.reset();
		currentPhase = MatchPhase.UNKNOWN;
		previousPhase = MatchPhase.UNKNOWN;
		autoWinner = null;
		ourAlliance = null;
	}

	/**
	 * Updates the hub tracker state. Call this in robotPeriodic().
	 *
	 * <p>This method:
	 * <ul>
	 *   <li>Updates the alliance from DriverStation
	 *   <li>Determines the auto winner from the game-specific message (first char: 'R' or 'B')
	 *   <li>Tracks phase transitions and resets the phase timer on change
	 *   <li>Logs hub status to NetworkTables for dashboard display
	 * </ul>
	 */
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

	/**
	 * Returns whether scoring is currently allowed for our alliance.
	 *
	 * <p>Takes into account the current phase, time elapsed in the phase, and whether we won auto.
	 * Includes a MIN_SHOOT_TIME buffer at phase boundaries to avoid edge cases.
	 *
	 * @return true if scoring is allowed now, false otherwise
	 */
	public static boolean canScore() {
		if (ourAlliance == null) {
			return false;
		}
		return canScoreInPhase(currentPhase, phaseTimer.get(), ourAlliance == autoWinner);
	}

	/**
	 * Returns whether scoring will be allowed soon (within a 3-second buffer).
	 *
	 * <p>Useful for pre-positioning or preparing to score before the window opens.
	 * It looks ahead across phase boundaries if needed.
	 *
	 * @return true if scoring is allowed now or will be within BUFFER_TIME seconds
	 */
	public static boolean canScoreBuffered() {
		if (canScore()) {
			return true;
		}
		double phaseTime = phaseTimer.get();
		double bufferedTime = phaseTime + BUFFER_TIME;
		return willBeAbleToScoreAt(bufferedTime);
	}

	/**
	 * Checks if scoring is possible at a future time within the current or next phase.
	 *
	 * @param futurePhaseTime
	 * 		the future elapsed time to check (may exceed current phase duration)
	 * @return true if scoring will be allowed at that time
	 */
	private static boolean willBeAbleToScoreAt(double futurePhaseTime) {
		if (ourAlliance == null) {
			return false;
		}

		boolean weWonAuto = ourAlliance == autoWinner;
		int phaseDuration = currentPhase.getDuration();

		if (futurePhaseTime >= phaseDuration) {
			MatchPhase nextPhase = currentPhase.getNext();
			if (nextPhase == null) {
				return true;
			}
			double timeInNextPhase = futurePhaseTime - phaseDuration;
			return canScoreInPhase(nextPhase, timeInNextPhase, weWonAuto);
		}

		return canScoreInPhase(currentPhase, futurePhaseTime, weWonAuto);
	}

	/**
	 * Determines if scoring is allowed in a specific phase at a specific time.
	 *
	 * @param phase
	 * 		the match phase to check
	 * @param phaseTime
	 * 		the elapsed time within that phase
	 * @param weWonAuto
	 * 		whether our alliance won autonomous
	 * @return true if scoring is allowed under these conditions
	 */
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

	/**
	 * Returns the current hub status for dashboard display.
	 *
	 * @return ACTIVE if we can score now, BUFFERED if we can score soon, NOT_AVAILABLE otherwise
	 */
	public static HubStatus getHubStatus() {
		if (canScore()) {
			return HubStatus.ACTIVE;
		} else if (canScoreBuffered()) {
			return HubStatus.BUFFERED;
		} else {
			return HubStatus.NOT_AVAILABLE;
		}
	}

	/**
	 * Returns a color representing the current hub status.
	 *
	 * @return green if ACTIVE, yellow if BUFFERED, red if NOT_AVAILABLE
	 */
	public static Color getHubStatusColor() {
		return switch (getHubStatus()) {
			case ACTIVE -> COLOR_GREEN;
			case BUFFERED -> COLOR_YELLOW;
			case NOT_AVAILABLE -> COLOR_RED;
		};
	}

	/**
	 * Returns the current match phase being tracked.
	 *
	 * @return the current MatchPhase
	 */
	public static MatchPhase getCurrentPhase() {
		return currentPhase;
	}

	/**
	 * Returns the time elapsed in the current phase.
	 *
	 * @return elapsed time in seconds since the current phase started
	 */
	public static double getPhaseElapsedTime() {
		return phaseTimer.get();
	}

	/**
	 * Returns the time remaining in the current phase.
	 *
	 * @return remaining time in seconds, minimum 0
	 */
	public static double getPhaseRemainingTime() {
		return Math.max(0, currentPhase.getDuration() - phaseTimer.get());
	}

	/**
	 * Returns which alliance won the autonomous period.
	 *
	 * @return the winning alliance, or null if not yet determined
	 */
	public static Alliance getAutoWinner() {
		return autoWinner;
	}

	/**
	 * Returns whether our alliance won the autonomous period.
	 *
	 * @return true if our alliance won auto, false if we lost or alliance is unknown
	 */
	public static boolean weWonAuto() {
		return ourAlliance != null && ourAlliance == autoWinner;
	}

	/**
	 * Returns whether game-specific data has been received from the FMS.
	 *
	 * @return true if game data is available
	 */
	public static boolean hasGameData() {
		String gameData = DriverStation.getGameSpecificMessage();
		return gameData != null && !gameData.isEmpty();
	}

	/**
	 * Represents the current hub scoring availability status.
	 */
	public enum HubStatus {
		/** Hub is not available for scoring. */
		NOT_AVAILABLE,

		/** Hub will be available soon (within buffer time). */
		BUFFERED,

		/** Hub is currently available for scoring. */
		ACTIVE
	}
}
