# Robot OpMode Documentation

This document provides an overview of the robot's software architecture, control schemes, and autonomous logic.

> [!NOTE]
> For detailed instructions on calibrating the vision and motor constants, see the [TUNING_GUIDE.md](TUNING_GUIDE.md).

## 1. TeleOp: `OPMode.java`

The main TeleOp program handles driver input and coordinates subsystems.

### Controller 1 (Drive & Intake)
- **Left Stick**: Linear movement (Forward/Back/Strafe).
- **Right Stick X**: Rotation.
- **Right Bumper**: Toggle **Slow Mode** (Multiplier: 0.5).
- **Right Trigger**: Intake power.
- **Y Button**: Toggle **Reverse Intake**.
- **A Button**: Automated **Drive to Score Pose**.
- **X Button**: Toggle **Chassis Tag-Lock & Auto-Aim** (Synced rotation towards High Basket).
- **B Button**: Toggle **Small Basket Lock** (Locks orientation towards Low Basket).
- **Left Trigger**: Reset Storage to intake position (Home).
- **Start Button**: Emergency Field-Centric Reset.
- **DPad Up**: Force set pattern to default.

### Controller 2 (Outtake & Storage)
- **A Button**: Reset Storage to intake position (Home).
- **X Button**: Start automated **Shooting sequence**.
- **B Button**: Toggle **Shoot Motor** (Flywheel).
- **Y Button**: Toggle **Turret Lock & Auto-Aim** (Vision/Pose tracking).
- **Right Trigger**: Toggle Storage Gate manually.
- **Left Trigger + Left Stick X**: Manual storage/conveyor movement (overrides auto).
- **Left Stick Y**: Manual Shooter Velocity Offset (Adjusts current target power).
- **Left/Right Bumpers**: Manual Outtake Angle adjustment.

---

## 2. Autonomous: Competition Routines

The project features several autonomous routines tailored for different field positions and scoring strategies.

### Primary Routines (`AutoFarBlueHuman.java` & `AutoFarRedHuman.java`)
These are the most refined routines, focusing on scoring a preload, picking up from the human player station, and cycling back to score.
- **Execution Flow**: 
    1. Score Preload shot.
    2. Drive to Spike Mark/Human station to pick up balls.
    3. Return to scoring position and auto-score.
    4. Repeat cycling if time allows.
    5. Park in the designated zone before the 30s limit.

### Legacy Routines (`AutonomieBlue.java` & `AutonomieRed.java`)
Original routines using a state-based approach for specific pickup sequences (Pickup 1, 2, 3).

### Features
- **Dynamic Aiming**: Automatically calculates distances to the goal (LOCK_POSE) to adjust flywheel velocity and outtake angle mid-flight.
- **Pose Handoff**: At the end of autonomous, the final coordinates and heading are saved to `PoseStorage`. This ensures that TeleOp (`OPMode.java`) starts with a correctly calibrated field-centric orientation.
- **Safety Loops**: Includes a 29.7s failsafe that stops all motors and handles pose handoff before the robot is disabled by the field management system.

---

## 3. Key Subsystems
- **Storage Subsystem**: Unified state machine that handles Homing (magnetic sensor), Shooting sequences (recoil/nudge), and manual overrides.
- **Outtake Subsystem**: 
    - **Turret Control**: Precise positioning using a PID loop and Through-Bore encoder (Through PedroPathing pose logic).
    - **Smart Flywheel**: Dual-motor flywheel with PIDF control for consistent velocity regardless of battery voltage.
    - **Angle Control**: Adaptive servo positioning for variable-distance shots.
- **Intake Subsystem**: Motorized intake with active clearing.

## 4. Calibration Constants
- **Turret Tracking**: `TICKS_PER_DEGREE` (in `OuttakeSubsystem.java`) - Adjust for turret precision.
- **Sensing FOV**: `HUSKYLENS_FOV_DEG` (in `OuttakeSubsystem.java`) - Adjust based on camera lens.
