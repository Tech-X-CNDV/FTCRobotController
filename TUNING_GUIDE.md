# Robot Tuning & Calibration Guide

This document explains how to calibrate the various software features to match your robot's physical hardware performance.

## 1. Outtake System (PIDF Velocity Control)
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

The outtake uses `RUN_USING_ENCODER` (PIDF-based velocity control). This system maps physical distance to a specific **Target Velocity** (ticks/sec) instead of raw power.

### STEP A: Find Physical Limit (Max Velocity)
**Tuning OpMode**: [MaxVelocityTuningOpMode.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/tuning/MaxVelocityTuningOpMode.java)

Before tuning PIDF, you must find the motor's true capacity.
1. Run `Tuning: Find Max Velocity` with a fresh battery.
2. Note the **Average Velocity** (ticks/sec) while at 1.0 power.
3. Update `MAX_VELOCITY` in `OuttakeSubsystem.java` (Line 54).
   - *Example*: If you see 5100, set `MAX_VELOCITY = 5100;`. This provides the benchmark for all dynamic calculations.

### STEP B: Interactive PIDF Tuning
**Tuning OpMode**: [FlywheelTuningOpMode.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/tuning/FlywheelTuningOpMode.java)

The system uses a **Dual-F Model** to account for non-linear friction:
`Actual_F = Velocity_F + (Static_F / Target_Velocity)`

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `Static F` (kS)| Static Friction. | **TUNE FIRST**. Increase until the flywheel just starts spinning at low targets (e.g. 1000). |
| `Velocity F` (kV)| Proportional F. | Increase after kS until you reach ~95% of target at high speed (4800). |
| `flywheelP` | Proportional Gain. | Increase last to eliminate recovery dip and small errors. |

**Tuning Controls (Gamepad 1)**:
- **[Y]**: Toggle High Velocity (4800 / 4000).
- **[X]**: Toggle Low Velocity (1000) for friction test.
- **[B]**: Cycle Step Size (10, 1, 0.1, 0.01, 0.001).
- **Dpad Up/Down**: Adjust **P**.
- **Dpad Right/Left**: Adjust **Velocity F** (kV).
- **Bumpers R/L**: Adjust **Static F** (kS).

### STEP C: Dynamic Distance Integration
In matches, the robot automatically calculates:
`Target Velocity = MIN_SHOOT_VELOCITY + (Distance * VELOCITY_DISTANCE_SCALING) + manualVelocityOffset`

| Variable | Description | Value |
| :--- | :--- | :--- |
| `VELOCITY_DISTANCE_SCALING`| Velocity boost per cm. | Increase if shots fall short only at long range. |
| `MIN_SHOOT_VELOCITY` | Baseline velocity (ticks/sec). | The speed needed to score from the closest point. |

### Dynamic Launcher Angle
| Variable | Description | Value |
| :--- | :--- | :--- |
| `CLOSE_DIST` / `FAR_DIST` | Distance thresholds (cm). | Default: 30 to 65. |
| `CLOSE_ANGLE` / `FAR_ANGLE` | Servo positions at thresholds. | Default: 0.15 (Close) to 0.8 (Far). |

> **Note**: Launcher angle uses a linear ramp between these points. If the ball hits the top of the basket, DECREASE the angle value.

---

## 2. Chassis Lock (Auto-Aim)
**File**: [OPMode.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OPMode.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `autoTurnPower` multiplier | The gain for rotation (currently `1.0`). | **Increase** (e.g. 1.5) if the robot spins too slowly. **Lower** if it overshoots. |

> **Normalization**: The robot automatically takes the shortest turn path (Angle Normalization). If the robot spins in circles, check the IMU/Heading sensor and `PoseStorage.allianceOffset`.

---

## 3. Physical Subsystems
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `TURRET_GEAR_RATIO` | Gear reduction for turret. | Ratio between the servo shaft (with encoder) and the turret. TUNE THIS if angle is off. |
| `TURRET_LIMIT_LEFT/RIGHT`| Safety limits for turret rotation. | Set this to prevent the turret from hitting your chassis or pulling wires. |
| `HUSKYLENS_FOV_DEG` | The camera's field of view. | Default 60.0. Adjust if the tag isn't centered when using visual tracking. |
| `INITIAL_ANGLE` | Starting position of the outtake bucket. | Default 0.9. Adjust to set the default "rest" or "ready" angle. |

---

## 4. Storage Subsystem (Unified State Machine)
**File**: [StorageSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/StorageSubsystem.java)

The storage system manages the conveyor and gate using a precise state machine and a magnetic homing sensor.

### Homing Sequence
1. **HOMING_FAST**: Moves forward until the `MagneticSensor` is triggered.
2. **HOMING_BACKOFF**: Nudges away from the sensor to clear the signal.
3. **HOMING_SLOW**: Precision approach to the sensor for the final zero.

### Shooting Cycle
- **RECOILING**: Pulls back to clear any jammed balls.
- **NUDGING**: Briefly nudges forward to prepare for the shot.
- **SHOOTING**: Rapidly accelerates the conveyor to feed the flywheel.

---

## 5. Hybrid Turret PID Control
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

The turret uses a custom PIDF (Proportional-Integral-Derivative + Feedforward) controller for precise tracking.

### PID Constants
| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `turretP` | Proportional Gain (Main speed). | **Increase** until the turret oscillates, then lower by 20%. |
| `turretI` | Integral Gain (Correction). | **Increase** if the turret stops slightly before its target. Too much causes overshooting. |
| `turretD` | Derivative Gain (Damping). | **Increase** to stop the turret from "bouncing" or vibrating at the target. |
| `turretF` | Feedforward (Static Friction). | The minimal power needed to start the turret moving. |

### The Custom PID Logic Explained
The `updateTurretPID()` function runs every loop (~10ms). It works as follows:

1. **Error Calculation**: `error = targetPosition - currentPosition`.
2. **P (Proportional)**: Multiplies the error. Big error = fast move.
3. **I (Integral)**: Accumulates error over time (`integralSum += error * dt`). This "forces" the turret to move the last few ticks if friction stops it early.
   - *Anti-Windup*: The code caps this sum to prevent the turret from spinning wildly.
4. **D (Derivative)**: Measures how fast the error is changing. It acts like a "brake" to slow down as it approaches the target.
   - *LPF*: A Low-Pass Filter (`turretD_LPF`) is applied to smooth out sensor noise.
5. **F (Feedforward)**: Adds a small constant power in the direction of the error to overcome static friction.
6. **Limit Enforcement**: `setTurretTargetAngle()` automatically handles 360° roll-overs and hard software limits.
