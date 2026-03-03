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
`Target Velocity = (MIN_SHOOT_POWER + (Dist * SCALING)) * MAX_VELOCITY`

| Variable | Description | Value |
| :--- | :--- | :--- |
| `POWER_DISTANCE_SCALING`| Velocity boost per inch. | Increase if shots fall short only at long range. |
| `MIN_SHOOT_POWER` | Baseline velocity ratio. | The "Minimum" percentage of MAX_VELOCITY needed to score. |


### Dynamic Launcher Angle
| Variable | Description | Value |
| :--- | :--- | :--- |
| `CLOSE_DIST` / `FAR_DIST` | Distance thresholds (inches). | Default: 30" to 65". |
| `CLOSE_ANGLE` / `FAR_ANGLE` | Servo positions at thresholds. | Default: 0.5 (Close) to 0.9 (Far). |

> **Note**: Launcher angle uses a linear ramp between these points. If the ball hits the top of the basket, DECREASE the angle value.

---

## 2. Chassis Lock (Auto-Aim)
**File**: [OPMode.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OPMode.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `autoTurnPower` multiplier | The gain for rotation (currently `1.2`). | **Increase** (e.g. 2.0) if the robot spins too slowly. **Lower** if it overshoots. |
| Persistence (100ms) | Hardcoded timeout for tag loss. | Prevents robot "jerking" when the camera feed blocks or the tag is hidden. |

> **Normalization**: The robot automatically takes the shortest turn path (Angle Normalization). If the robot spins in circles, check the IMU/Heading sensor.

---

## 3. Physical Subsystems
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `TICKS_PER_DEGREE` | Turret encoder ratio. | If the turret turns 10° but stops at 12°, decrease this value. |
| `MAX_TURRET_ANGLE_DEG`| Safety limit for turret rotation. | Set this to prevent the turret from hitting your chassis or pulling wires. |
| `HUSKYLENS_FOV_DEG` | The camera's field of view. | Default 60.0. Adjust if the tag isn't centered when the error says 0. |
| `INITIAL_ANGLE` | Starting position of the outtake bucket. | Default 0.9. Adjust to set the default "rest" or "ready" angle. |

---

## 4. Storage (RETIRED)
**Note**: The storage subsystem is currently disabled in the code. Section 4 is preserved for legacy reference but does not affect the robot's current operation.

---

## 5. Hybrid Turret PID Control
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

The turret now uses a custom PIDF (Proportional-Integral-Derivative + Feedforward) controller for high-speed tracking.

### PID Constants
| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `turretP` | Proportional Gain (Main speed). | **Increase** until the turret oscillates, then lower by 20%. |
| `turretI` | Integral Gain (Correction). | **Increase** if the turret stops slightly before its target. Too much causes overshooting. |
| `turretD` | Derivative Gain (Damping). | **Increase** to stop the turret from "bouncing" or vibrating at the target. |
| `turretF` | Feedforward (Static Friction). | The minimal power needed to start the turret moving. Adjust until the turret doesn't "get stuck" on tiny errors. |

### How the Hybrid System Works
1. **HuskyLock (Priority)**: If the HuskyLens sees the target AprilTag, it calculates the visual error (pixels from center) and maps it directly to a turret angle adjustment. This is extremely precise and ignores drive-drift.
2. **PoseLock (Fallback)**: If the tag is hidden, the system calculates the angle to the basket using the robot's current X,Y coordinates. This is less precise but keeps the turret pointed in the right direction.

### The Custom PID Logic Explained
The `updateTurretPID()` function runs every ~10ms. It works as follows:

1. **Error Calculation**: `error = targetPosition - currentPosition`. This is the distance we need to move.
2. **P (Proportional)**: Multiplies the error. Big error = fast move; Small error = slow move.
3. **I (Integral)**: Accumulates error over time (`integralSum += error * dt`). This "forces" the turret to move the last few ticks if physics (like friction) stops it early.
    - *Anti-Windup*: The code caps this sum to prevent the turret from spinning wildly if it's held by hand.
4. **D (Derivative)**: Measures how fast the error is changing (`(error - lastError) / dt`). It acts like a "brake" to slow down the turret as it approaches the target.
5. **F (Feedforward)**: Adds a small constant power in the direction of the error to overcome static friction of the gears.
6. **Safety**: If the error is less than 2 ticks, the turret stops and clears the integral sum to prevent "jittering" at rest.
