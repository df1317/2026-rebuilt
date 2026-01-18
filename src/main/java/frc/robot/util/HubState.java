package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class HubState {

	public enum Period {
		AUTO, TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, END_GAME, UNKNOWN
	}

	private HubState() {
	}

	public static Period getCurrentPeriod() {
		if (DriverStation.isAutonomous()) {
			return Period.AUTO;
		}
		double matchTime = DriverStation.getMatchTime();
		if (matchTime < 0) {
			return Period.UNKNOWN;
		} else if (matchTime > 130) {
			return Period.TRANSITION;
		} else if (matchTime > 105) {
			return Period.SHIFT_1;
		} else if (matchTime > 80) {
			return Period.SHIFT_2;
		} else if (matchTime > 55) {
			return Period.SHIFT_3;
		} else if (matchTime > 30) {
			return Period.SHIFT_4;
		} else {
			return Period.END_GAME;
		}
	}

	public static boolean isHubActive() {
		return isHubActiveForPeriod(getCurrentPeriod());
	}

	public static boolean isHubActiveBuffered() {
		Period period = getCurrentPeriod();
		if (isHubActiveForPeriod(period)) {
			return true;
		}
		double matchTime = DriverStation.getMatchTime();
		double bufferedTime = matchTime + 3.0;
		Period bufferedPeriod = getPeriodForTime(bufferedTime);
		return isHubActiveForPeriod(bufferedPeriod);
	}

	private static Period getPeriodForTime(double matchTime) {
		if (matchTime < 0) {
			return Period.UNKNOWN;
		} else if (matchTime > 130) {
			return Period.TRANSITION;
		} else if (matchTime > 105) {
			return Period.SHIFT_1;
		} else if (matchTime > 80) {
			return Period.SHIFT_2;
		} else if (matchTime > 55) {
			return Period.SHIFT_3;
		} else if (matchTime > 30) {
			return Period.SHIFT_4;
		} else {
			return Period.END_GAME;
		}
	}

	private static boolean isHubActiveForPeriod(Period period) {
		if (period == Period.UNKNOWN) {
			return false;
		}
		if (period == Period.AUTO || period == Period.TRANSITION || period == Period.END_GAME) {
			return true;
		}

		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData == null || gameData.isEmpty()) {
			return false;
		}

		var allianceOpt = DriverStation.getAlliance();
		if (allianceOpt.isEmpty()) {
			return false;
		}

		Alliance alliance = allianceOpt.get();
		char gameChar = gameData.charAt(0);

		boolean weGoInactiveFirst = (alliance == Alliance.Red && gameChar == 'R')
				|| (alliance == Alliance.Blue && gameChar == 'B');

		if (weGoInactiveFirst) {
			return period == Period.SHIFT_2 || period == Period.SHIFT_4;
		} else {
			return period == Period.SHIFT_1 || period == Period.SHIFT_3;
		}
	}

	public static boolean hasGameData() {
		String gameData = DriverStation.getGameSpecificMessage();
		return gameData != null && !gameData.isEmpty();
	}
}
