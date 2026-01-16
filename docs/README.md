# Docs

Quick reference documentation for robot controls and configuration.

## Controls

### Teleop

#### Xbox Controller (Driver) [`Port 0`]

| Binding        | Action                | Description                                       |
|----------------|-----------------------|---------------------------------------------------|
| `Button A`     | Zero gyro             | Resets gyro heading                               |
| `Left Bumper`  | Lock drive            | Locks drivebase                                   |
| `Right Bumper` | Toggle robot relative | Switches between field and robot relative driving |

#### Left Joystick (Operator) [`Port 1`]

| Binding | Action | Description |
|---------|--------|-------------|
| TBD     | TBD    | TBD         |

#### Right Joystick (Operator) [`Port 2`]

| Binding | Action | Description |
|---------|--------|-------------|
| TBD     | TBD    | TBD         |

### Test Mode

> **Note:** Test mode bindings TBD

#### Xbox Controller (Driver)

| Binding       | Action         | Description            |
|---------------|----------------|------------------------|
| `A Button`    | Zero gyro      | Resets gyro heading    |
| `Back Button` | Center modules | Centers swerve modules |

## CAN IDs

| Component   | Location    | ID   |
|-------------|-------------|------|
| Drive Motor | Front Right | `12` |
| Drive Motor | Front Left  | `13` |
| Drive Motor | Back Right  | `14` |
| Drive Motor | Back Left   | `15` |
|             |             |      |
| Turn Motor  | Front Right | `16` |
| Turn Motor  | Front Left  | `17` |
| Turn Motor  | Back Right  | `18` |
| Turn Motor  | Back Left   | `19` |
|             |             |      |
| CanCoder    | Front Right | `20` |
| CanCoder    | Front Left  | `21` |
| CanCoder    | Back Right  | `22` |
| CanCoder    | Back Left   | `23` |

## DIO Constants

| Sensor | Port |
|--------|------|
| TBD    | TBD  |
