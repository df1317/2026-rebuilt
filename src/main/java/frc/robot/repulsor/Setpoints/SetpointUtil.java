/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package frc.robot.repulsor.Setpoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldFlip;

public final class SetpointUtil {
	public static Alliance currentAllianceOrBlue() {
		return DriverStation.getAlliance().orElse(Alliance.Blue);
	}

	public static Pose2d flipToRed(Pose2d bluePose) {
		if (bluePose == null)
			return Pose2d.kZero;
		return FieldFlip.toRed(bluePose);
	}

	public static Translation2d flipToRed(Translation2d blue) {
		if (blue == null)
			return new Translation2d(0.0, 0.0);
		return FieldFlip.toRed(blue);
	}

	public static Pose2d flipToBlue(Pose2d redPose) {
		return flipToRed(redPose); // symmetric operation
	}

	public static Translation2d flipToBlue(Translation2d red) {
		return flipToRed(red); // symmetric operation
	}

	public static Pose2d getSetPose(RepulsorSetpoint sp, SetpointContext ctx) {
		if (sp == null)
			return Pose2d.kZero;
		return sp.get(ctx);
	}
}
