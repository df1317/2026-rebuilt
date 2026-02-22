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

package frc.robot.repulsor.Tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Predictive.Model.DynamicObject;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Tracking.Model.Alliance;
import frc.robot.repulsor.Tracking.Vision.FieldVision;

public final class FieldTrackerCore {
	private static final FieldTrackerCore INSTANCE = new FieldTrackerCore();
	private static final String DEFAULT_COLLECT_RESOURCE_TYPE = "fuel";

	private final FieldTrackerDynamicTracker dynamicTracker = new FieldTrackerDynamicTracker();

	private FieldTrackerCore() {
	}

	public static FieldTrackerCore getInstance() {
		return INSTANCE;
	}

	public void ingestTracked(String id, String type, Pose3d p, long nowNs) {
		dynamicTracker.ingestTracked(id, type, p, nowNs);
	}

	public Pose2d nextCollectionGoalBlue(Pose2d robotPoseBlue, double speedCap, int goalUnits) {
		List<DynamicObject> dynamics = dynamicTracker.snapshotDynamics();

		// Filter to only collect resources (fuel)
		List<DynamicObject> fuel = new ArrayList<>();
		for (DynamicObject d : dynamics) {
			if (DEFAULT_COLLECT_RESOURCE_TYPE.equalsIgnoreCase(d.type)) {
				fuel.add(d);
			}
		}

		if (fuel.isEmpty()) {
			// Fall back to center field
			return new Pose2d(
					RepulsorConstants.FIELD_LENGTH * 0.5,
					RepulsorConstants.FIELD_WIDTH * 0.5,
					robotPoseBlue.getRotation());
		}

		// Sort by distance to robot, pick nearest
		Translation2d robotPos = robotPoseBlue.getTranslation();
		fuel.sort(Comparator.comparingDouble(d -> robotPos.getDistance(d.pos)));

		DynamicObject nearest = fuel.get(0);
		Rotation2d faceTarget = nearest.pos.minus(robotPos).getAngle();
		return new Pose2d(nearest.pos, faceTarget);
	}

	public void resetAll() {
		dynamicTracker.clear();
	}

	public FieldVision createFieldVision(String name) {
		return new FieldVision(this, name);
	}

	public List<RepulsorSetpoint> getPredictedSetpoints(
			Alliance alliance,
			Translation2d robotPosition,
			double radiusMeters,
			CategorySpec category,
			int maxResults) {
		return new ArrayList<>();
	}
}
