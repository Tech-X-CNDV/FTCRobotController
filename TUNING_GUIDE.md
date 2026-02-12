# Robot Tuning & Calibration Guide

This document explains how to calibrate the various software features to match your robot's physical hardware performance.

## 1. Outtake System (Power & Logic)
**File**: [OuttakeSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/OuttakeSubsystem.java)

The outtake power is now calculated using PedroPathing distances: `Power = MIN_SHOOT_POWER + (Dist * SCALING)`.

### Shooting Power
| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `VOLTAGE` | The reference battery level (default 13.4V). | Power is scaled UP as battery drops below this. Do not change during competition. |
| `MIN_SHOOT_POWER` | Minimum power to eject a ball (default 0.55). | Adjust until the ball barely leaves the launcher at close range. |
| `POWER_DISTANCE_SCALING`| How much power increases per inch (default 0.0025). | Increase if shots fall short at long range but hit correctly at close range. |
| `RAMP_STEP` | Launch motor acceleration (default 0.05). | **Decrease** (e.g. 0.02) if the gearbox clicks or motor stalls. **Increase** for faster spin-up. |
| `AUTO_SHOOT_POWER`| Static power for Autonomous (default 0.75). | Set to your most consistent scoring power for fixed positions. |

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
| `TURRET_TRACKING_POWER`| Speed of the turret's auto-rotation. | Default 0.6. Increase for snappier tracking. |
| `TURRET_RESET_POWER` | Speed when the turret returns to 0°. | Default 0.5. |
| `INITIAL_ANGLE` | Starting position of the outtake bucket. | Default 0.9. Adjust to set the default "rest" or "ready" angle. |

---

## 4. Storage & Watchdog
**File**: [StorageSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/StorageSubsystem.java)

The watchdog monitors the motor for jams and automatically runs a recovery sequence.

| Variable | Description | Tuning Tips |
| :--- | :--- | :--- |
| `STUCK_VELOCITY_THRESHOLD` | Minimal speed (ticks/sec) before "stuck" (default 50.0). | **Higher** = faster jam detection. **Lower** = prevents false positives on heavy loads. |
| `STUCK_TIMEOUT_MS` | Delay before recovery (default 500ms). | Increase if the motor is "thinking" too long on startup and triggering recovery. |
| `RECOVERY_DELAY_MS` | Wait time between vibrate cycles (default 250ms). | Time for a physical jam to fall out before retrying. |

### Logic Behaviors:
- **Priority Input**: Pressing D-pad Up during a jam will immediately reset the watchdog and attempt your manual move. 
- **Busy Tolerance**: The system ignores "IsBusy" locks when the motor is within **5 ticks** of the target. This ensures rapid-clicking D-pad Up feels responsive.
