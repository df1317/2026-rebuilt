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

package frc.robot.repulsor;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import frc.robot.repulsor.ExtraPathingHelpers.ExtraPathingBounceListener;
import frc.robot.repulsor.ExtraPathingHelpers.ExtraPathingClearPath;
import frc.robot.repulsor.ExtraPathingHelpers.ExtraPathingCollision;
import frc.robot.repulsor.FieldPlanner.Obstacle;

public class ExtraPathing {

	public static Translation2d[] rectCorners(Translation2d center, double length, double width) {
		return ExtraPathingCollision.rectCorners(center, length, width);
	}

	public static boolean robotIntersects(
			Translation2d center,
			double robotLengthMeters,
			double robotWidthMeters,
			List<? extends Obstacle> obstacles) {
		return ExtraPathingCollision.robotIntersects(
				center, robotLengthMeters, robotWidthMeters, obstacles);
	}

	public static boolean isClearPath(
			String topicRoot,
			Translation2d start,
			Translation2d goal,
			List<? extends Obstacle> obstacles,
			double robotLengthMeters,
			double robotWidthMeters,
			boolean publishSamples) {
		return ExtraPathingClearPath.isClearPath(
				topicRoot, start, goal, obstacles, robotLengthMeters, robotWidthMeters, publishSamples);
	}

	public class BounceListener extends ExtraPathingBounceListener {
		public BounceListener(double bounceDistanceThreshold, int bounceHistoryLimit) {
			super(bounceDistanceThreshold, bounceHistoryLimit);
		}
	}
}
