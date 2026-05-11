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

import frc.robot.repulsor.Fields.FieldDefinition;
import frc.robot.repulsor.Fields.Rebuilt2026;
import frc.robot.util.FieldFlip;

public final class RepulsorConstants {
	public static final double FIELD_LENGTH = FieldFlip.fieldLength();
	public static final double FIELD_WIDTH = FieldFlip.fieldWidth();
	public static final FieldDefinition FIELD = new Rebuilt2026();
}
