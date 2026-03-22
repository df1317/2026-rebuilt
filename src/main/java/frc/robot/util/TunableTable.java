package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Unit;

/**
 * Namespace helper for creating tunables with a shared key prefix.
 * All tunables created through a table are backed by {@link dev.doglog.DogLog#tunable},
 * inheriting its FMS gating and NetworkTables integration.
 *
 * <p>
 * Usage:
 * <pre>{@code
 * private static final TunableTable tunables = new TunableTable("Shooter");
 * private final TunableDouble testRPM = tunables.value("RPM", 3000.0, RPM);
 *
 * // Auto-tune a TalonFX — creates kP/kI/kD/kV/kS tunables and auto-applies on change
 * tunables.pidTalonFX("Flywheel", motor, 1.0, 0.0, 0.1, 0.12, 0.0);
 *
 * // Auto-tune a SparkMax — creates kP/kI/kD/kV tunables and auto-applies on change
 * tunables.pidSpark("Feeder", feeder, 0.5, 0.0, 0.0, 0.0017);
 * }</pre>
 *
 * <p>
 * Inspired by Team 340 (GRR).
 */
public final class TunableTable {

	private final String prefix;

	public TunableTable(String prefix) {
		this.prefix = prefix;
	}

	/** Creates a nested table with a child prefix. */
	public TunableTable getNested(String name) {
		return new TunableTable(prefix + "/" + name);
	}

	/** Creates a tunable double. */
	public TunableDouble value(String name, double defaultValue) {
		return new TunableDouble(prefix + "/" + name, defaultValue);
	}

	/** Creates a tunable double with unit metadata. */
	public TunableDouble value(String name, double defaultValue, Unit unit) {
		return new TunableDouble(prefix + "/" + name, defaultValue, unit);
	}

	/** Creates a tunable boolean. */
	public TunableBoolean value(String name, boolean defaultValue) {
		return new TunableBoolean(prefix + "/" + name, defaultValue);
	}

	/**
	 * Creates kP/kI/kD/kV/kS tunables under a nested table and auto-applies to a TalonFX Slot0
	 * whenever any value changes.
	 */
	public void pidTalonFX(String name, TalonFX motor,
			double kP, double kI, double kD, double kV, double kS) {
		TunableTable pid = getNested(name);
		TunableDouble pTune = pid.value("kP", kP);
		TunableDouble iTune = pid.value("kI", kI);
		TunableDouble dTune = pid.value("kD", kD);
		TunableDouble vTune = pid.value("kV", kV);
		TunableDouble sTune = pid.value("kS", kS);

		Runnable apply = () -> {
			var configs = new Slot0Configs();
			configs.kP = pTune.get();
			configs.kI = iTune.get();
			configs.kD = dTune.get();
			configs.kV = vTune.get();
			configs.kS = sTune.get();
			motor.getConfigurator().apply(configs);
		};
		pTune.addListener(v -> apply.run());
		iTune.addListener(v -> apply.run());
		dTune.addListener(v -> apply.run());
		vTune.addListener(v -> apply.run());
		sTune.addListener(v -> apply.run());
	}

	/**
	 * Creates kP/kI/kD/kV tunables under a nested table and auto-applies to a Spark motor
	 * whenever any value changes.
	 */
	public void pidSpark(String name, SparkBase motor,
			double kP, double kI, double kD, double kV) {
		TunableTable pid = getNested(name);
		TunableDouble pTune = pid.value("kP", kP);
		TunableDouble iTune = pid.value("kI", kI);
		TunableDouble dTune = pid.value("kD", kD);
		TunableDouble vTune = pid.value("kV", kV);

		Runnable apply = () -> {
			SparkMaxConfig config = new SparkMaxConfig();
			config.closedLoop.pid(pTune.get(), iTune.get(), dTune.get());
			config.closedLoop.feedForward.kV(vTune.get());
			motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		};
		pTune.addListener(v -> apply.run());
		iTune.addListener(v -> apply.run());
		dTune.addListener(v -> apply.run());
		vTune.addListener(v -> apply.run());
	}

	/**
	 * Creates kP/kI/kD/kV/kS/kG tunables under a nested table and auto-applies to a TalonFX Slot0
	 * whenever any value changes. Use for mechanisms with gravity compensation (e.g. elevator, climber).
	 */
	public void pidTalonFX(String name, TalonFX motor,
			double kP, double kI, double kD, double kV, double kS, double kG) {
		TunableTable pid = getNested(name);
		TunableDouble pTune = pid.value("kP", kP);
		TunableDouble iTune = pid.value("kI", kI);
		TunableDouble dTune = pid.value("kD", kD);
		TunableDouble vTune = pid.value("kV", kV);
		TunableDouble sTune = pid.value("kS", kS);
		TunableDouble gTune = pid.value("kG", kG);

		Runnable apply = () -> {
			var configs = new Slot0Configs();
			configs.kP = pTune.get();
			configs.kI = iTune.get();
			configs.kD = dTune.get();
			configs.kV = vTune.get();
			configs.kS = sTune.get();
			configs.kG = gTune.get();
			motor.getConfigurator().apply(configs);
		};
		pTune.addListener(v -> apply.run());
		iTune.addListener(v -> apply.run());
		dTune.addListener(v -> apply.run());
		vTune.addListener(v -> apply.run());
		sTune.addListener(v -> apply.run());
		gTune.addListener(v -> apply.run());
	}

	/**
	 * Creates kP/kI/kD tunables under a nested table and auto-applies to a Spark motor
	 * whenever any value changes. Use this for motors that don't need feedforward tuning.
	 */
	public void pidSpark(String name, SparkBase motor,
			double kP, double kI, double kD) {
		TunableTable pid = getNested(name);
		TunableDouble pTune = pid.value("kP", kP);
		TunableDouble iTune = pid.value("kI", kI);
		TunableDouble dTune = pid.value("kD", kD);

		Runnable apply = () -> {
			SparkMaxConfig config = new SparkMaxConfig();
			config.closedLoop.pid(pTune.get(), iTune.get(), dTune.get());
			motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		};
		pTune.addListener(v -> apply.run());
		iTune.addListener(v -> apply.run());
		dTune.addListener(v -> apply.run());
	}
}
