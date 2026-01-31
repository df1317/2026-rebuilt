package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Stateless enum representing match phases with their time boundaries.
 *
 * <p>Each phase defines its start time, end time, and the next phase in sequence.
 * Times are in seconds remaining on the match clock (counting down from 150).
 *
 * <p>Phase sequence: AUTO -> TRANSITION -> SHIFT_1 -> SHIFT_2 -> SHIFT_3 -> SHIFT_4 -> END_GAME
 */
public enum MatchPhase {
	/** Unknown/invalid phase when match time is negative. */
	UNKNOWN(0, 0, null),

	/** Final 30 seconds of the match where both alliances can score. */
	END_GAME(30, 0, null),

	/** Fourth shift period (55-30 seconds remaining). */
	SHIFT_4(55, 30, END_GAME),

	/** Third shift period (80-55 seconds remaining). */
	SHIFT_3(80, 55, SHIFT_4),

	/** Second shift period (105-80 seconds remaining). */
	SHIFT_2(105, 80, SHIFT_3),

	/** First shift period (130-105 seconds remaining). */
	SHIFT_1(130, 105, SHIFT_2),

	/** Transition period after auto before shifts begin (140-130 seconds remaining). */
	TRANSITION(140, 130, SHIFT_1),

	/** Autonomous period at the start of the match. */
	AUTO(20, 0, TRANSITION);

	private final int startTimeSeconds;
	private final int endTimeSeconds;
	private final MatchPhase nextPhase;

	/**
	 * Creates a match phase with the specified time boundaries.
	 *
	 * @param startTime
	 * 		the match time (seconds remaining) when this phase starts
	 * @param endTime
	 * 		the match time (seconds remaining) when this phase ends
	 * @param next
	 * 		the phase that follows this one or null if this is a terminal phase
	 */
	MatchPhase(int startTime, int endTime, MatchPhase next) {
		this.startTimeSeconds = startTime;
		this.endTimeSeconds = endTime;
		this.nextPhase = next;
	}

	/**
	 * Determines the match phase for a given match time.
	 *
	 * @param matchTime
	 * 		the match time in seconds remaining
	 * @return the corresponding phase, or UNKNOWN if matchTime is negative
	 */
	public static MatchPhase fromMatchTime(double matchTime) {
		if (matchTime < 0) {
			return UNKNOWN;
		}
		for (MatchPhase phase : values()) {
			if (phase != UNKNOWN && phase != AUTO
					&& matchTime > phase.endTimeSeconds && matchTime <= phase.startTimeSeconds) {
				return phase;
			}
		}
		return END_GAME;
	}

	/**
	 * Returns the current match phase based on DriverStation state.
	 *
	 * @return AUTO if in autonomous mode, otherwise the phase corresponding to the current match time
	 */
	public static MatchPhase getCurrent() {
		if (DriverStation.isAutonomous()) {
			return AUTO;
		}
		return fromMatchTime(DriverStation.getMatchTime());
	}

	/**
	 * Returns the next phase in the match sequence.
	 *
	 * @return the next phase, or null if this is END_GAME or UNKNOWN
	 */
	public MatchPhase getNext() {
		return nextPhase;
	}

	/**
	 * Returns the match time when this phase starts.
	 *
	 * @return start time in seconds remaining
	 */
	public int getStartTime() {
		return startTimeSeconds;
	}

	/**
	 * Returns the match time when this phase ends.
	 *
	 * @return end time in seconds remaining
	 */
	public int getEndTime() {
		return endTimeSeconds;
	}

	/**
	 * Returns the duration of this phase in seconds.
	 *
	 * @return phase duration (startTime - endTime)
	 */
	public int getDuration() {
		return startTimeSeconds - endTimeSeconds;
	}
}
