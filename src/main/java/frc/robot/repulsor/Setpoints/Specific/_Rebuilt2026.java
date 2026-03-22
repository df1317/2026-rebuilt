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

package frc.robot.repulsor.Setpoints.Specific;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Setpoints.GameSetpoint;
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.repulsor.Setpoints.SetpointType;

import java.util.Optional;

import frc.robot.util.FieldFlip;

public class _Rebuilt2026 {
	public static final double HUB_FACE_TO_AIMPOINT_METERS = 0.60;
	public static final int BLUE_HUB_ANCHOR_TAG_ID = 20;
	public static final int BLUE_OUTPOST_ANCHOR_TAG_ID = 13;
	// ===== Collection =====
	public static final GameSetpoint CENTER_COLLECT = new StaticPoseSetpoint(
			"CENTER_COLLECT",
			SetpointType.kOther,
			new Pose2d(RepulsorConstants.FIELD_LENGTH / 2.0, RepulsorConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero));
	public static final GameSetpoint OUTPOST_COLLECT = new ApproachFromTagSetpoint(
			"OUTPOST_COLLECT", SetpointType.kHumanPlayer, BLUE_OUTPOST_ANCHOR_TAG_ID, 1.25);
	// ===== Fixed Scoring Poses (5 positions around hub at ~1.8m radius) =====
	private static final double HUB_SCORE_RADIUS_M = 1.8;
	public static final GameSetpoint HUB_SCORE_FRONT = new StaticPoseSetpoint(
			"HUB_SCORE_FRONT", SetpointType.kScore, hubScoringPose(0));
	public static final GameSetpoint HUB_SCORE_FRONT_LEFT = new StaticPoseSetpoint(
			"HUB_SCORE_FRONT_LEFT", SetpointType.kScore, hubScoringPose(60));
	public static final GameSetpoint HUB_SCORE_FRONT_RIGHT = new StaticPoseSetpoint(
			"HUB_SCORE_FRONT_RIGHT", SetpointType.kScore, hubScoringPose(-60));
	public static final GameSetpoint HUB_SCORE_REAR_LEFT = new StaticPoseSetpoint(
			"HUB_SCORE_REAR_LEFT", SetpointType.kScore, hubScoringPose(135));
	public static final GameSetpoint HUB_SCORE_REAR_RIGHT = new StaticPoseSetpoint(
			"HUB_SCORE_REAR_RIGHT", SetpointType.kScore, hubScoringPose(-135));
	public static final GameSetpoint[] HUB_SCORE_ALL = {
			HUB_SCORE_FRONT, HUB_SCORE_FRONT_LEFT, HUB_SCORE_FRONT_RIGHT,
			HUB_SCORE_REAR_LEFT, HUB_SCORE_REAR_RIGHT
	};
	// ===== Climb Poses =====
	private static final double CLIMB_OFFSET_Y_M = 2.5;
	public static final GameSetpoint CLIMB_LEFT = new StaticPoseSetpoint(
			"CLIMB_LEFT", SetpointType.kOther,
			new Pose2d(hubAimpointBlue().getX(), hubAimpointBlue().getY() + CLIMB_OFFSET_Y_M, Rotation2d.kZero));
	public static final GameSetpoint CLIMB_RIGHT = new StaticPoseSetpoint(
			"CLIMB_RIGHT", SetpointType.kOther,
			new Pose2d(hubAimpointBlue().getX(), hubAimpointBlue().getY() - CLIMB_OFFSET_Y_M, Rotation2d.kZero));

	protected _Rebuilt2026() {
	}

	private static Pose2d hubScoringPose(double angleDeg) {
		Translation2d hubCenter = hubAimpointBlue();
		Rotation2d angle = Rotation2d.fromDegrees(angleDeg);
		Translation2d pos = hubCenter.minus(new Translation2d(HUB_SCORE_RADIUS_M / 2, angle));
		Rotation2d faceHub = hubCenter.minus(pos).getAngle();
		return new Pose2d(pos, faceHub);
	}

	public static GameSetpoint nearestScoringPose(Translation2d robotPos) {
		GameSetpoint nearest = HUB_SCORE_FRONT;
		double bestDist = Double.MAX_VALUE;
		for (GameSetpoint sp : HUB_SCORE_ALL) {
			double dist = robotPos.getDistance(sp.bluePose(SetpointContext.EMPTY).getTranslation());
			if (dist < bestDist) {
				bestDist = dist;
				nearest = sp;
			}
		}
		return nearest;
	}

	// ===== Hub Aimpoint =====

	public static Translation2d hubAimpointBlue() {
		return hubAimpointFromAnchorTagBlue(BLUE_HUB_ANCHOR_TAG_ID);
	}

	private static Translation2d hubAimpointFromAnchorTagBlue(int anchorTagId) {
		Optional<Pose3d> pose3d = FieldFlip.aprilTagLayout().getTagPose(anchorTagId);
		if (pose3d.isEmpty()) {
			return new Translation2d(RepulsorConstants.FIELD_LENGTH / 2.0, RepulsorConstants.FIELD_WIDTH / 2.0);
		}
		Pose2d tag2d = pose3d.get().toPose2d();
		Rotation2d intoHub = tag2d.getRotation().plus(Rotation2d.kPi);
		return tag2d.getTranslation().plus(new Translation2d(HUB_FACE_TO_AIMPOINT_METERS, intoHub));
	}

	// ===== Inner setpoint types =====

	private static final class StaticPoseSetpoint extends GameSetpoint {
		private final Pose2d bluePose;

		StaticPoseSetpoint(String name, SetpointType type, Pose2d bluePose) {
			super(name, type);
			this.bluePose = bluePose == null ? Pose2d.kZero : bluePose;
		}

		@Override
		public Pose2d bluePose(SetpointContext ctx) {
			return bluePose;
		}
	}

	private static final class ApproachFromTagSetpoint extends GameSetpoint {
		private final int tagId;
		private final double standoffMeters;

		ApproachFromTagSetpoint(String name, SetpointType type, int tagId, double standoffMeters) {
			super(name, type);
			this.tagId = tagId;
			this.standoffMeters = standoffMeters;
		}

		@Override
		public Pose2d bluePose(SetpointContext ctx) {
			Optional<Pose3d> pose3d = FieldFlip.aprilTagLayout().getTagPose(tagId);
			if (pose3d.isEmpty())
				return Pose2d.kZero;

			Pose2d tag2d = pose3d.get().toPose2d();
			Translation2d approachPos = tag2d.getTranslation().plus(new Translation2d(standoffMeters, tag2d.getRotation()));
			Rotation2d faceTag = tag2d.getRotation().plus(Rotation2d.kPi);
			return new Pose2d(approachPos, faceTag);
		}
	}
}
