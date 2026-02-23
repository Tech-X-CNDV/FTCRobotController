package org.firstinspires.ftc.teamcode.config.subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StorageSubsystem {
    private final DcMotorEx storageMotor;
    private final ColorSensor colorSensor;
    private final Servo servoArunc;

    public static double STUCK_VELOCITY_THRESHOLD = 50.0; // ticks per second
    public static double STUCK_TIMEOUT_MS = 500.0; // 500ms before considering it stuck
    public static double RECOVERY_DELAY_MS = 250.0; // 250ms wait before retrying
    public static int POSITION_TOLERANCE = 12; // Nominal threshold for firing accuracy
    public static int WATCHDOG_THRESHOLD = 25; // Lenient threshold for watchdog disarming

    private final ElapsedTime stuckTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    public boolean isStuck = false;
    public boolean isManual = false;
    private boolean watchdogArmed = false;

    public enum RecoveryState {
        IDLE,
        RETURNING,
        WAITING_FOR_RETRY,
        RETRYING
    }

    public RecoveryState recoveryState = RecoveryState.IDLE;
    private int lastMoveStartPos = 0;
    private int lastMoveTargetPos = 0;
    private double lastMovePower = 0;

    public StorageSubsystem(HardwareMap hardwareMap) {
        storageMotor = hardwareMap.get(DcMotorEx.class, "StorageMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorS");
        servoArunc = hardwareMap.get(Servo.class, "ServoArunc");
    }

    public void InitStorage() {
        storageMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storageMotor.setPower(0);
        storageMotor.setPositionPIDFCoefficients(40);

        PIDFCoefficients defaultVelocityPID = storageMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double Kp_velocity = defaultVelocityPID.p * 0.60; // Reduce Kpv to make it less aggressive
        double Ki_velocity = 0.0;
        double Kd_velocity = 3; // CRITICAL: Damping term for high inertia
        double Kf_velocity = defaultVelocityPID.f; // Retain the manufacturer's Feedforward

        storageMotor.setVelocityPIDFCoefficients(Kp_velocity, Ki_velocity, Kd_velocity, Kf_velocity);
        storageMotor.setTargetPosition(0);
        storageMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        servoArunc.setPosition(0.97);
    }

    public double ReturnVelocity() {
        return storageMotor.getVelocity();
    }

    private int lastTarget = 0;

    public void MoveRelative(int delta, double power) {
        if (isBusy())
            return;

        servoArunc.setPosition(0.97);
        // Drift Prevention: Base new target on the INTENDED previous target, not
        // physical spot.
        setTarget(storageMotor.getTargetPosition() + delta, power);
    }

    public boolean autoThrow = false;

    private int throwState = 0;
    private int turns = 0;

    public void ThrowAll(double servoWaitTime) {
        switch (throwState) {
            case 0: // PHASE 1: FIRE
                // Ensure indexer is dead-still before firing to prevent jams
                if (!isBusy()) {
                    servoArunc.setPosition(0.65);
                    servoTimer.reset();
                    throwState = 1;
                }
                break;

            case 1: // PHASE 2: RETRACT & INDEX SIMULTANEOUSLY
                // Adjusted to servoWaitTime to give the Axon time to complete the full 0.6 arc
                if (servoTimer.seconds() > servoWaitTime) { // dynamic wait time
                    servoArunc.setPosition(0.97); // Start returning to home

                    turns++;
                    if (turns >= 3) {
                        turns = 0;
                        throwState = 2; // Move to wrap-up
                    } else {
                        // Drift Prevention: Base new target on the INTENDED previous target.
                        setTarget(storageMotor.getTargetPosition() + 475, 1.0);

                        throwState = 0; // Jump back to Case 0 to wait for the next ball to arrive
                    }
                }
                break;

            case 2: // PHASE 3: WRAP UP
                // Final safety delay to ensure the flicker is home before ending the auto
                // sequence
                if (servoTimer.seconds() > 0.2) {
                    autoThrow = false;
                    throwState = 0;
                }
                break;
        }
    }

    int pos = 0;
    public ElapsedTime checkTimer = new ElapsedTime();
    public ElapsedTime servoTimer = new ElapsedTime();
    boolean isMoving = false;
    public boolean autoSort = false;

    public void PatternSortAuto(char[] pattern) {
        if (isBusy()) {
            isMoving = true;
            return;
        }

        if (isMoving) {
            checkTimer.reset();
            isMoving = false;
        }
        if (checkTimer.seconds() > 2 && servoTimer.seconds() > 1) {
            MoveRelative(475, 1);
            turns++;
            checkTimer.reset(); // Reset AFTER starting the move
        }

        if (pos > 2) {
            pos = 0;
            turns = 0;
            servoArunc.setPosition(0.97);
            this.autoSort = false;
            return;
        }

        char currentColor = idenColor();

        if (autoSort && currentColor != pattern[pos]) {
            servoArunc.setPosition(0.97);
            if (turns >= 3) {
                turns = 0;
                this.autoSort = false;
            }
        } else if (currentColor == pattern[pos]) {
            turns = 0;
            pos++;
            servoArunc.setPosition(0.6);
            servoTimer.reset();
        }
    }

    public void update() {
        if (!isManual) {
            updateWatchdog();
            updateRecovery();
        }
        if (autoSort || colorSensingEnabled) {
            updateColor();
        }
    }

    private void updateWatchdog() {
        // MONITOR IN ALL ACTIVE STATES: Idle, Returning, or Retrying.
        // If it jams during a recovery phase, we need to know!
        if (storageMotor.isBusy() && !isStuck && watchdogArmed) {
            // Watchdog disarms early (WATCHDOG_THRESHOLD) to avoid monitoring jitter.
            if (Math.abs(storageMotor.getTargetPosition() - storageMotor.getCurrentPosition()) < WATCHDOG_THRESHOLD) {
                watchdogArmed = false; // Move effectively completed, disarm.
                stuckTimer.reset();
                return;
            }

            // Check velocity (ticks/second)
            if (Math.abs(storageMotor.getVelocity()) < STUCK_VELOCITY_THRESHOLD) {
                if (stuckTimer.milliseconds() > STUCK_TIMEOUT_MS) {
                    isStuck = true;
                    // Trigger Recovery
                    recoveryState = RecoveryState.RETURNING;
                    storageMotor.setTargetPosition(lastMoveStartPos);
                    storageMotor.setPower(lastMovePower);
                }
            } else {
                stuckTimer.reset();
            }
        } else {
            stuckTimer.reset();
        }
    }

    private void updateRecovery() {
        int error = Math.abs(storageMotor.getTargetPosition() - storageMotor.getCurrentPosition());

        if (recoveryState == RecoveryState.RETURNING) {
            if (error < POSITION_TOLERANCE) {
                recoveryState = RecoveryState.WAITING_FOR_RETRY;
                recoveryTimer.reset();
            }
        } else if (recoveryState == RecoveryState.WAITING_FOR_RETRY) {
            if (recoveryTimer.milliseconds() > RECOVERY_DELAY_MS) {
                recoveryState = RecoveryState.RETRYING;
                storageMotor.setTargetPosition(lastMoveTargetPos);
                storageMotor.setPower(lastMovePower);
            }
        } else if (recoveryState == RecoveryState.RETRYING) {
            if (error < POSITION_TOLERANCE) {
                recoveryState = RecoveryState.IDLE;
                isStuck = false;
            }
        }
    }

    private void setTarget(int target, double power) {
        lastMoveStartPos = storageMotor.getCurrentPosition();
        lastMoveTargetPos = target;
        lastMovePower = power;
        recoveryState = RecoveryState.IDLE;
        isStuck = false;

        storageMotor.setTargetPosition(target);
        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storageMotor.setPower(power);
        watchdogArmed = true; // NEW MOVEMENT: Start monitoring
    }

    public void Abort() {
        autoThrow = false; // Stop the ThrowAll loop
        autoSort = false; // Stop the PatternSortAuto loop
        throwState = 0; // Reset the state machine to start
        turns = 0; // Reset ball count
        recoveryState = RecoveryState.IDLE;
        isStuck = false;
        watchdogArmed = false;
        servoArunc.setPosition(0.97); // Reset flicker to home

        // Kill motor movement and lock it at current position
        storageMotor.setPower(0);
        storageMotor.setTargetPosition(storageMotor.getCurrentPosition());
        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ResetStuck() {
        isStuck = false;
        watchdogArmed = false;
        stuckTimer.reset();
    }

    public void ManualMove(double power) {
        isManual = true;
        isStuck = false;
        watchdogArmed = false;
        recoveryState = RecoveryState.IDLE;
        storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        storageMotor.setPower(power);
    }

    public void RestoreAuto() {
        isManual = false;
        stuckTimer.reset();
        storageMotor.setTargetPosition(storageMotor.getCurrentPosition());
        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public float hue;
    public float sat;
    public boolean colorSensingEnabled = false;
    private final float[] hsvValues = new float[3];
    private char cachedColor = 'N';

    private void updateColor() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        hue = hsvValues[0];
        sat = hsvValues[1];
        if (sat < 0.35)
            cachedColor = 'P';
        else if (hue > 120 && hue < 150)
            cachedColor = 'G';
        else
            cachedColor = 'N';
    }

    public char idenColor() {
        return cachedColor;
    }

    public void setServoPos(double target) {
        servoArunc.setPosition(target);
    }

    public double getServoPos() {
        return servoArunc.getPosition();
    }

    public int getTurns() {
        return turns;
    }

    public int getPos() {
        return pos;
    }

    public double getPower() {
        return storageMotor.getPower();
    }

    public int getPosition() {
        return storageMotor.getCurrentPosition();
    }

    public boolean isBusy() {
        // If we are in a recovery state, we are definitely busy.
        if (recoveryState != RecoveryState.IDLE)
            return true;

        int error = Math.abs(storageMotor.getTargetPosition() - storageMotor.getCurrentPosition());

        // OPTION 1: Target reached within nominal precision
        if (error < POSITION_TOLERANCE) {
            return false;
        }

        // OPTION 2: Adaptive Readiness (Close enough and stopped)
        // If we are within the watchdog window and the motor has effectively stopped,
        // we allow firing to prevent mechanical friction from hanging the sequence.
        if (error < WATCHDOG_THRESHOLD && Math.abs(ReturnVelocity()) < STUCK_VELOCITY_THRESHOLD) {
            return false;
        }

        // Otherwise, check the motor.
        return storageMotor.isBusy();
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("  Storage Status", isStuck ? "STUCK (" + recoveryState + ")" : "OK");
        telemetry.addData("  Storage Velocity", ReturnVelocity());
        telemetry.addData("Storage AutoSort", autoSort);
        telemetry.addData("Storage AutoThrow", autoThrow);
        telemetry.addData("Storage Target Progress", turns + "/3 (" + pos + ")");
        telemetry.addData("Storage Pos", getPosition());
        telemetry.addData("Shooter Servo", getServoPos());

        // char color = idenColor();
        // String colorStr = "NONE";
        // if (color == 'G')
        // colorStr = "GREEN";
        // else if (color == 'P')
        // colorStr = "PURPLE";
        // telemetry.addData("Detected Artifact", colorStr);
    }
}
