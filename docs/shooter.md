# Shooter Subsystem

Controls a flywheel shooter with a feeder motor and a pivoting hood.

## Hardware

| Component      | Motor    | CAN ID | Inversion Constant    |
|----------------|----------|--------|-----------------------|
| Flywheel       | TalonFX  | `40`   | `INVERTED`            |
| Feeder         | SparkMax | `28`   | `FEEDER_INVERTED`     |
| Hood           | SparkMax | `24`   | `HOOD_INVERTED`       |

> If the hood moves the wrong direction, flip `HOOD_INVERTED` in `ShooterConstants`.

## Hood Homing

The hood uses soft limits derived at runtime from its physical hard stops. It must be homed before percent-based positioning works (`setHoodPercent`, `setForDistance`).

### Auto-home (Button 4 in test mode)
Drives to both hard stops via stall detection, zeroes the encoder at min, records max, then applies soft limits. Completes in ~5s.

### Manual homing (test mode)
Use this when auto-homing doesn't reach the stops cleanly:

1. Hold **Button 9** + move joystick to jog the hood to the minimum position
2. Press **Button 5** — marks current position as 0% (min)
3. Jog the hood to the maximum position
4. Press **Button 6** — marks current position as 100% (max), applies soft limits

> After manual homing, `isHoodHomed()` returns `true` and percent-based control is active.

## Teleop Controls

### Xbox Controller (Driver) — Shooter

| Binding          | Action                        |
|------------------|-------------------------------|
| `Left Trigger`   | Decrease hood angle (–5% per press) |
| `Right Trigger`  | Increase hood angle (+5% per press) |
| `Joystick L Button 2` (toggle) | Spin up flywheel → start feeder → run hopper |

## Test Mode Controls

| Button     | Action                        | Notes                          |
|------------|-------------------------------|--------------------------------|
| `Button 4` | Auto-home hood                | Drives to hard stops via stall |
| `Button 5` | Mark current position as min  | Use after jogging to min stop  |
| `Button 6` | Mark current position as max  | Use after jogging to max stop  |
| `Button 7` | Test flywheel                 | Tunable: `Test/ShooterRPM`    |
| `Button 8` | Test feeder                   | Tunable: `Test/FeederRPM`     |
| `Button 9` | Jog hood (hold + joystick)    | Holds position on release      |

## Distance-to-RPM / Hood Curves

Flywheel RPM and hood position are computed from distance using polynomial curve fits derived from calibration data. This replaces the old linear-interpolation lookup table, enabling extrapolation beyond the tested range (4.6m).

### Calibration Data

| Distance (m) | Flywheel (RPM) | Hood (%) |
|--------------|----------------|----------|
| 1.00         | 3000           | 0.00     |
| 1.60         | 2600           | 0.00     |
| 2.30         | 2700           | 0.00     |
| 2.54         | 2750           | 0.00     |
| 2.80         | 2750           | 0.17     |
| 3.50         | 2950           | 0.17     |
| 4.00         | 3100           | 0.36     |
| 4.60         | 3250           | 0.53     |

### Curve Fitting

We fitted several polynomial orders (quadratic, cubic, quartic) to the RPM data using least-squares regression. The RPM data has a characteristic dip at close range (1-2m) because the hood is flat and we're lobbing the ball nearly straight up, requiring less flywheel speed. As distance increases, the shot flattens and RPM climbs.

- **Quadratic** — smooth extrapolation beyond 4.6m but misses the close-range dip
- **Cubic** — captures the dip well but diverges wildly beyond 5m (drops to 1400 RPM at 7m)
- **Quartic** — nails the close-range dip but explodes upward beyond 5m

The solution: **blend the quartic into the quadratic**. The quartic handles 1.0–2.3m (the dip), a linear blend transitions from 2.3–2.8m, and the quadratic takes over from 2.8m onward for smooth extrapolation.

**RPM (blended):**
- Below 2.3m: `34.381d⁴ - 447.749d³ + 2134.640d² - 4165.446d + 5436.875`
- 2.3–2.8m: linear blend between quartic and quadratic
- Above 2.8m: `109.602d² - 496.440d + 3279.834`

**Hood % (quadratic, clamped to [0, 1]):**
- `0.05706d² - 0.17218d + 0.11705`

### Curve Fit Graph

![Shooter curve fits](https://cdn.hackclub.com/019d081d-e1b3-7bdc-81bb-dcdb1be85623/image.png)

Gray dashed line is the old LUT (linear interpolation, stopped at 4.6m). Blue solid line is the new polynomial curve fit that extends beyond the tested range.

## Key Constants (`ShooterConstants`)

| Constant              | Value   | Description                              |
|-----------------------|---------|------------------------------------------|
| `HOOD_INVERTED`       | `true`  | Flip if hood moves wrong direction       |
| `HOOD_GEAR_RATIO`     | `24.0`  | Used for encoder → degrees conversion   |
| `HOOD_TOLERANCE`      | `3 deg` | Deadband for `isHoodAtPosition()`        |
| `HOOD_HOMING_VOLTAGE` | `2.0 V` | Voltage used during homing and jogging   |
| `HOOD_STALL_RPM`      | `2.0`   | Motor RPM threshold for stall detection  |
| `VELOCITY_TOLERANCE`  | `100 RPM` | Deadband for `isAtSpeed()`             |
