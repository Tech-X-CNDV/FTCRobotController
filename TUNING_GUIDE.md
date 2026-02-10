# Robot Tuning & Calibration Guide

This document explains how to calibrate the various software features to match your robot's physical hardware performance.

## 1. Smart Flywheel (Distance & Voltage)
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

The flywheel uses a linear mapping: `Power = (TagWidth * SLOPE) + OFFSET`.

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `NOMINAL_VOLTAGE` | The "Standard" battery level (default 13.2V). | Set this to the voltage of your typical "Fresh" battery. |
| `DIST_POWER_OFFSET` | The vertical shift of the power curve. | Increase if shots are consistently short; decrease if overshooting. |
| `DIST_POWER_SLOPE` | How much power changes per inch (negative value). | Make more negative (e.g. -0.007) if the flywheel is too powerful when close. |
| `DEFAULT_SHOOT_POWER`| Power used when no tag is in sight. | Set to your most reliable "static" shooting power (e.g. 0.75). |

### Calibration Steps:
1. Go to **Close Range** (~2ft). Record `Tag Width` from telemetry and find the `Power` that scores.
2. Go to **Far Range** (~6ft). Record `Tag Width` and finding the `Power`.
3. Calculate:
   - `SLOPE = (PowerFar - PowerClose) / (WidthFar - WidthClose)`
   - `OFFSET = PowerClose - (WidthClose * SLOPE)`

---

## 2. Chassis Tag-Lock (Auto-Aim)
**File**: [OPMode.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OPMode.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `kP_CHASSIS_TURN` | The speed of autonomous rotation. | **Increase** if the robot is too slow to face the tag. **Lower** if it wobbles/vibrates. |

---

## 3. Physical Subsystems
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `TICKS_PER_DEGREE` | Turret encoder ratio. | If the turret turns 10° but stops at 12°, decrease this value. |
| `MAX_TURRET_ANGLE_DEG`| Safety limit for turret rotation. | Set this to prevent the turret from hitting your chassis or pulling wires. |
| `HUSKYLENS_FOV_DEG` | The camera's field of view. | Default 60.0. Adjust if the tag isn't centered when the error says 0. |
| `TURRET_TRACKING_POWER`| Speed of the turret's auto-rotation. | Default 0.6. Increase for snappier tracking. |
| `TURRET_RESET_POWER` | Speed when the turret returns to 0°. | Default 0.5. |
| `INITIAL_ANGLE` | Starting position of the outtake bucket. | Default 0.9. Adjust to set the default "rest" or "ready" angle. |

---

## 4. Storage Watchdog
**File**: [StorageSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/StorageSubsystem.java)

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `STUCK_VELOCITY_THRESHOLD` | Threshold to detect a jam. | If sorting stops for no reason, lower this. If it never detects jams, raise it. |
| `STUCK_TIME_MS` | Delay before recovery starts. | Increase if the robot recovers too soon during normal heavy movement. |
