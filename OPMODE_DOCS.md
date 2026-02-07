# Robot OpMode Documentation

This document provides an overview of the robot's software architecture, control schemes, and autonomous logic.

## 1. TeleOp: `OPMode.java`

The main TeleOp program handles driver input and coordinates subsystems.

### Controller 1 (Drive & Intake)
- **Left Stick**: Linear movement (Forward/Back/Strafe).
- **Right Stick X**: Rotation.
- **Right Bumper**: Toggle **Slow Mode** (Multiplier: 0.5).
- **Right Trigger**: Intake power.
- **Y Button**: Toggle **Reverse Intake**.
- **A Button**: Automated **Drive to Score Pose**.
- **Left Trigger**: Conveyor move relative (475 ticks).
- **DPad Up**: Force set pattern to default.

### Controller 2 (Outtake & Storage)
- **A Button**: Toggle Shoot Motor.
- **X Button**: Auto-Throw sequence (clears storage).
- **Y Button**: Toggle **Turret Tag Lock** (AprilTag tracking).
- **B Button**: Start **Auto-Sort** (if pattern detected).
- **DPad Up**: Manual conveyor move (475 ticks).
- **DPad Down**: Reset Storage (if stuck).
- **DPad Right**: Toggle Outtake Servo position.
- **Left Trigger + Right Stick X**: Manual conveyor control (overrides auto).
- **Left/Right Bumpers**: Manual Outtake Angle adjustment.

---

## 2. Autonomous: `AutonomieBlue.java` & `AutonomieRed.java`

These OpModes use a state machine and **PedroPathing** for precise movement.

### State Machine Overview
1.  **State 0-1**: Score Preload.
2.  **State 2-6**: Alignment, pickup, and score Sequence 1.
3.  **State 7-10**: Alignment, pickup, and score Sequence 2.
4.  **State 11-14**: Alignment, pickup, and score Sequence 3.
5.  **State 15**: Park in designated zone.

### Features
- **Loop Caching**: Optimized performance by caching follower status (isBusy, getPose).
- **Pose Handoff**: At the end of autonomous (`stop()`), the robot's position is saved to `PoseStorage`. This allows the TeleOp to start with the correct heading and coordinates automatically.

---

## 3. Key Subsystems
- **Storage**: Features an auto-jam detection (Watchdog) and recovery system.
- **Outtake**: Includes a motorized turret with ±90° limits and AprilTag tracking using HuskyLens.
- **Intake**: Simple motorized intake with reverse capability.

## 4. Calibration Constants
- **Turret Tracking**: `TICKS_PER_DEGREE` (in `OuttakeSubsystem.java`) - Adjust for turret precision.
- **Sensing FOV**: `HUSKYLENS_FOV_DEG` (in `OuttakeSubsystem.java`) - Adjust based on camera lens.
