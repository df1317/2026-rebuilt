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

package frc.robot.repulsor.Tracking.Model;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Pipe implements PrimitiveObject {
	private final Pose3d position;
	private final Distance radius;
	private final Angle angle;

	public Pipe(Pose3d position, Distance radius, Angle angle) {
		this.position = position;
		this.radius = radius;
		this.angle = angle;
	}

	public Pose3d getPosition() {
		return position;
	}

	public Distance getRadius() {
		return radius;
	}

	public Angle getAngle() {
		return angle;
	}

	@Override
	public boolean intersects(Pose3d pos) {
		if (position == null || pos == null)
			return false;
		double dx = pos.getX() - position.getX();
		double dy = pos.getY() - position.getY();
		double r = radius.in(edu.wpi.first.units.Units.Meters);
		return (dx * dx + dy * dy) <= (r * r);
	}
}
