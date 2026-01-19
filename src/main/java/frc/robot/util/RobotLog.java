package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

/**
 * Centralized logging and error reporting utility.
 *
 * <p>
 * Provides unified logging to:
 * <ul>
 * <li>Elastic dashboard notifications (driver-facing alerts)</li>
 * <li>DogLog (DataLog + NetworkTables telemetry)</li>
 * <li>DriverStation console</li>
 * <li>WPILib Alerts (persistent dashboard warnings)</li>
 * </ul>
 *
 * <p>
 * Use this instead of scattered calls to {@code e.printStackTrace()}, {@code DriverStation.reportError()}, or throwing
 * raw exceptions.
 */
public final class RobotLog {

	public enum Severity {
		INFO, WARNING, ERROR, FATAL
	}

	private static final Map<String, Alert> alerts = new HashMap<>();
	private static final Map<String, Double> lastEmitSec = new HashMap<>();
	private static final double DEFAULT_THROTTLE_SECONDS = 1.0;

	private RobotLog() {
	}

	/**
	 * Log an informational message. Not sent to Elastic notifications by default.
	 *
	 * @param key
	 *          Unique key for throttling and DogLog organization
	 * @param title
	 *          Short title for the message
	 * @param message
	 *          Detailed message
	 */
	public static void info(String key, String title, String message) {
		emit(key, Severity.INFO, title, message, null, false, 0);
	}

	/**
	 * Log a warning. Sends an Elastic notification to alert drivers.
	 *
	 * @param key
	 *          Unique key for throttling and DogLog organization
	 * @param title
	 *          Short title for the warning
	 * @param message
	 *          Detailed message
	 */
	public static void warn(String key, String title, String message) {
		emit(key, Severity.WARNING, title, message, null, true, DEFAULT_THROTTLE_SECONDS);
	}

	/**
	 * Log a warning with an exception. Sends an Elastic notification.
	 *
	 * @param key
	 *          Unique key for throttling and DogLog organization
	 * @param title
	 *          Short title for the warning
	 * @param message
	 *          Detailed message
	 * @param t
	 *          The exception that caused the warning
	 */
	public static void warn(String key, String title, String message, Throwable t) {
		emit(key, Severity.WARNING, title, message, t, true, DEFAULT_THROTTLE_SECONDS);
	}

	/**
	 * Log an error. Sends an Elastic notification to alert drivers.
	 *
	 * @param key
	 *          Unique key for throttling and DogLog organization
	 * @param title
	 *          Short title for the error
	 * @param message
	 *          Detailed message
	 */
	public static void error(String key, String title, String message) {
		emit(key, Severity.ERROR, title, message, null, true, DEFAULT_THROTTLE_SECONDS);
	}

	/**
	 * Log an error with an exception. Sends an Elastic notification.
	 *
	 * @param key
	 *          Unique key for throttling and DogLog organization
	 * @param title
	 *          Short title for the error
	 * @param message
	 *          Detailed message
	 * @param t
	 *          The exception that caused the error
	 */
	public static void error(String key, String title, String message, Throwable t) {
		emit(key, Severity.ERROR, title, message, t, true, DEFAULT_THROTTLE_SECONDS);
	}

	/**
	 * Log a fatal error that should stop robot initialization.
	 *
	 * <p>
	 * Sends an Elastic notification, logs everything, then returns a RuntimeException to throw.
	 *
	 * @param key
	 *          Unique key for DogLog organization
	 * @param title
	 *          Short title for the fatal error
	 * @param message
	 *          Detailed message
	 * @param t
	 *          The exception that caused the fatal error
	 * @return RuntimeException to throw (preserves original if already a RuntimeException)
	 */
	public static RuntimeException fatal(String key, String title, String message, Throwable t) {
		emit(key, Severity.FATAL, title, message, t, true, 0);
		if (t instanceof RuntimeException re) {
			return re;
		}
		return new RuntimeException(message, t);
	}

	/**
	 * Create or update a persistent WPILib Alert.
	 *
	 * @param key
	 *          Unique key for the alert
	 * @param type
	 *          Alert type (kError, kWarning, kInfo)
	 * @param text
	 *          Alert text to display
	 * @param active
	 *          Whether the alert is currently active
	 */
	public static void setAlert(String key, AlertType type, String text, boolean active) {
		Alert a = alerts.computeIfAbsent(key, k -> new Alert(text, type));
		a.set(active);

		DogLog.log("Alerts/" + key + "/active", active);
		DogLog.log("Alerts/" + key + "/type", type.toString());
	}

	/**
	 * Create or update a warning alert.
	 *
	 * @param key
	 *          Unique key for the alert
	 * @param text
	 *          Alert text to display
	 * @param active
	 *          Whether the alert is currently active
	 */
	public static void setWarningAlert(String key, String text, boolean active) {
		setAlert(key, AlertType.kWarning, text, active);
	}

	/**
	 * Create or update an error alert.
	 *
	 * @param key
	 *          Unique key for the alert
	 * @param text
	 *          Alert text to display
	 * @param active
	 *          Whether the alert is currently active
	 */
	public static void setErrorAlert(String key, String text, boolean active) {
		setAlert(key, AlertType.kError, text, active);
	}

	private static void emit(
			String key,
			Severity severity,
			String title,
			String message,
			Throwable t,
			boolean notifyDrivers,
			double minIntervalSeconds) {
		if (key == null || key.isBlank()) {
			key = title;
		}

		double now = Timer.getFPGATimestamp();
		double last = lastEmitSec.getOrDefault(key, -1e9);
		if (minIntervalSeconds > 0 && (now - last) < minIntervalSeconds) {
			return;
		}
		lastEmitSec.put(key, now);

		DogLog.log("Events/" + key + "/severity", severity.toString());
		DogLog.log("Events/" + key + "/title", title);
		DogLog.log("Events/" + key + "/message", message);
		if (t != null) {
			DogLog.log("Events/" + key + "/exception", t.toString());
		}

		if (severity == Severity.WARNING) {
			DriverStation.reportWarning(title + " - " + message, false);
		} else if (severity == Severity.ERROR || severity == Severity.FATAL) {
			DriverStation.reportError(
					title + " - " + message,
					t != null ? t.getStackTrace() : new StackTraceElement[0]);
		}

		if (notifyDrivers) {
			sendElasticNotification(severity, title, message);
		}
	}

	private static void sendElasticNotification(Severity severity, String title, String message) {
		NotificationLevel level = switch (severity) {
			case INFO -> NotificationLevel.INFO;
			case WARNING -> NotificationLevel.WARNING;
			case ERROR, FATAL -> NotificationLevel.ERROR;
		};

		Elastic.sendNotification(
				new Notification()
						.withLevel(level)
						.withTitle(title)
						.withDescription(message)
						.withDisplaySeconds(severity == Severity.FATAL ? 10 : 5));
	}

	private static String stackTrace(Throwable t) {
		StringWriter sw = new StringWriter();
		t.printStackTrace(new PrintWriter(sw));
		return sw.toString();
	}
}
