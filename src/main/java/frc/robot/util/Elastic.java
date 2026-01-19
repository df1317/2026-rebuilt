// Copyright (c) 2023-2026 Gold87 and other Elastic contributors
// This software can be modified and/or shared under the terms
// defined by the Elastic license:
// https://github.com/Gold872/elastic_dashboard/blob/main/LICENSE

package frc.robot.util;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public final class Elastic {

	private static final StringTopic notificationTopic = NetworkTableInstance.getDefault()
			.getStringTopic("/Elastic/RobotNotifications");
	private static final StringPublisher notificationPublisher = notificationTopic.publish(
			PubSubOption.sendAll(true),
			PubSubOption.keepDuplicates(true));

	private static final StringTopic selectedTabTopic = NetworkTableInstance.getDefault()
			.getStringTopic("/Elastic/SelectedTab");
	private static final StringPublisher selectedTabPublisher = selectedTabTopic
			.publish(PubSubOption.keepDuplicates(true));

	private static final ObjectMapper objectMapper = new ObjectMapper();

	private Elastic() {
	}

	public enum NotificationLevel {
		INFO, WARNING, ERROR
	}

	public static void sendNotification(Notification notification) {
		try {
			notificationPublisher.set(objectMapper.writeValueAsString(notification));
		} catch (JsonProcessingException e) {
			e.printStackTrace();
		}
	}

	public static void selectTab(String tabName) {
		selectedTabPublisher.set(tabName);
	}

	public static void selectTab(int tabIndex) {
		selectTab(Integer.toString(tabIndex));
	}

	public static class Notification {

		@JsonProperty("level")
		private NotificationLevel level;

		@JsonProperty("title")
		private String title;

		@JsonProperty("description")
		private String description;

		@JsonProperty("displayTime")
		private int displayTimeMillis;

		@JsonProperty("width")
		private double width;

		@JsonProperty("height")
		private double height;

		public Notification() {
			this(NotificationLevel.INFO, "", "");
		}

		public Notification(NotificationLevel level, String title, String description) {
			this(level, title, description, 3000, 350, -1);
		}

		public Notification(
				NotificationLevel level,
				String title,
				String description,
				int displayTimeMillis,
				double width,
				double height) {
			this.level = level;
			this.title = title;
			this.displayTimeMillis = displayTimeMillis;
			this.description = description;
			this.height = height;
			this.width = width;
		}

		public Notification withLevel(NotificationLevel level) {
			this.level = level;
			return this;
		}

		public Notification withTitle(String title) {
			this.title = title;
			return this;
		}

		public Notification withDescription(String description) {
			this.description = description;
			return this;
		}

		public Notification withDisplaySeconds(double seconds) {
			return withDisplayMilliseconds((int) Math.round(seconds * 1000));
		}

		public Notification withDisplayMilliseconds(int displayTimeMillis) {
			this.displayTimeMillis = displayTimeMillis;
			return this;
		}

		public Notification withWidth(double width) {
			this.width = width;
			return this;
		}

		public Notification withHeight(double height) {
			this.height = height;
			return this;
		}

		public Notification withAutomaticHeight() {
			this.height = -1;
			return this;
		}

		public Notification withNoAutoDismiss() {
			this.displayTimeMillis = 0;
			return this;
		}

		public NotificationLevel getLevel() {
			return level;
		}

		public String getTitle() {
			return title;
		}

		public String getDescription() {
			return description;
		}

		public int getDisplayTimeMillis() {
			return displayTimeMillis;
		}

		public double getWidth() {
			return width;
		}

		public double getHeight() {
			return height;
		}
	}
}
