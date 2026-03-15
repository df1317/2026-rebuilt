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

import java.util.Optional;
import java.util.function.Predicate;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;

public final class GameElement {
	private final Alliance alliance;
	private final int maxContained;
	private final GameElementModel model;
	private final Predicate<GameObject> filter;
	private final RepulsorSetpoint relatedPoint;
	private final CategorySpec category;

	public GameElement(
			Alliance alliance,
			int maxContained,
			GameElementModel model,
			Predicate<GameObject> filter,
			RepulsorSetpoint relatedPoint,
			CategorySpec category) {
		this.alliance = alliance;
		this.maxContained = maxContained;
		this.model = model;
		this.filter = filter != null ? filter : go -> true;
		this.relatedPoint = relatedPoint;
		this.category = category != null ? category : CategorySpec.kScore;
	}

	public Alliance getAlliance() {
		return alliance;
	}

	public int getMaxContained() {
		return maxContained;
	}

	public GameElementModel getModel() {
		return model;
	}

	public boolean filter(GameObject go) {
		return filter.test(go);
	}

	public Optional<RepulsorSetpoint> getRelatedPoint() {
		return Optional.ofNullable(relatedPoint);
	}

	public CategorySpec getCategory() {
		return category;
	}
}
