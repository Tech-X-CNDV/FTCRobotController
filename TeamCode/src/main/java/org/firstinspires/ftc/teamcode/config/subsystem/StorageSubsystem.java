package org.firstinspires.ftc.teamcode.config.subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StorageSubsystem {
    private final DcMotorEx storageMotor;
    private final ColorSensor colorSensor;
    private final Servo servoArunc;

    public StorageSubsystem(HardwareMap hardwareMap) {
        storageMotor = hardwareMap.get(DcMotorEx.class, "StorageMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorS");
        servoArunc = hardwareMap.get(Servo.class, "ServoArunc");
    }

    public void InitStorage() {
        storageMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storageMotor.setPower(0);
        storageMotor.setPositionPIDFCoefficients(47);

        PIDFCoefficients defaultVelocityPID = storageMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double Kp_velocity = defaultVelocityPID.p * 0.66; // Reduce Kpv to make it less aggressive
        double Ki_velocity = 0.0;
        double Kd_velocity = 2.0;                       // CRITICAL: Damping term for high inertia
        double Kf_velocity = defaultVelocityPID.f;      // Retain the manufacturer's Feedforward

        storageMotor.setVelocityPIDFCoefficients(Kp_velocity, Ki_velocity, Kd_velocity, Kf_velocity);
        storageMotor.setTargetPosition(0);
        storageMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        servoArunc.setPosition(1);
    }

    public void MoveToPosition(int target, double power) {
        if (storageMotor.isBusy())
            return;
        servoArunc.setPosition(1);
        storageMotor.setTargetPosition(storageMotor.getCurrentPosition() + target);
        storageMotor.setPower(power);
    }

    private int lastTarget = 0;

    public void MoveRelative(int delta, double power) {
        // If the motor is still busy moving to the previous increment, wait.
        if (storageMotor.isBusy()) return;

        // We only increment if the code explicitly asks for a NEW movement
        servoArunc.setPosition(1);
        int newTarget = storageMotor.getCurrentPosition() + delta;
        storageMotor.setTargetPosition(newTarget);
        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storageMotor.setPower(power);
    }

    public boolean autoThrow = false;

    private int throwState = 0;
    private int turns = 0;

//    public void ThrowAll() {
//        switch (throwState) {
//            case 0: // Phase 1: Fire the flicker
//                // Wait for indexer to stop moving before firing
//                if (!storageMotor.isBusy()) {
//                    servoArunc.setPosition(0.6);
//                    servoTimer.reset();
//                    throwState = 1;
//                }
//                break;
//
//            case 1: // Phase 2: Retract the flicker
//                if (servoTimer.seconds() > 0.4) {
//                    servoArunc.setPosition(1.0);
//                    servoTimer.reset();
//                    throwState = 2;
//                }
//                break;
//
//            case 2: // Phase 3: Move indexer to next ball
//                if (servoTimer.seconds() > 0.2) {
//                    turns++;
//                    if (turns >= 3) {
//                        // Sequence Complete after 3 shots
//                        turns = 0;
//                        throwState = 0;
//                        autoThrow = false;
//                    } else {
//                        // Not done yet? Shift indexer 475 ticks for the next ball
//                        int newTarget = storageMotor.getCurrentPosition() + 475;
//                        storageMotor.setTargetPosition(newTarget);
//                        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        storageMotor.setPower(1.0);
//                        throwState = 0; // Loop back to Case 0 (Wait for motor, then fire)
//                    }
//                }
//                break;
//        }
//    }
public void ThrowAll() {
    switch (throwState) {
        case 0: // PHASE 1: FIRE
            // Ensure indexer is dead-still before firing to prevent jams
            if (!storageMotor.isBusy()) {
                servoArunc.setPosition(0.6);
                servoTimer.reset();
                throwState = 1;
            }
            break;

        case 1: // PHASE 2: RETRACT & INDEX SIMULTANEOUSLY
            // Adjusted to 0.32s to give the Axon time to complete the full 0.6 arc
            if (servoTimer.seconds() > 0.32) {
                servoArunc.setPosition(1.0); // Start returning to home

                turns++;
                if (turns >= 3) {
                    turns = 0;
                    throwState = 2; // Move to wrap-up
                } else {
                    // SPEED GAIN: We start the next ball move immediately.
                    // The servo retracts while the motor is already bringing the next ball up.
                    int newTarget = storageMotor.getCurrentPosition() + 475;
                    storageMotor.setTargetPosition(newTarget);
                    storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    storageMotor.setPower(1.0);

                    throwState = 0; // Jump back to Case 0 to wait for the next ball to arrive
                }
            }
            break;

        case 2: // PHASE 3: WRAP UP
            // Final safety delay to ensure the flicker is home before ending the auto sequence
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
    int add = 1;

    public void PatternSortAuto(char[] pattern) {
        if (storageMotor.isBusy()) {
            isMoving = true;
            return;
        }

        if (isMoving) {
            checkTimer.reset();
            add = 1;
            isMoving = false;
        }
        if (checkTimer.seconds() > 2 && servoTimer.seconds() > 1) {
            MoveToPosition(475, 1);
            turns++;
            checkTimer.reset(); // Reset AFTER starting the move
        }

        if (pos > 2) {
            pos = 0;
            turns = 0;
            servoArunc.setPosition(1);
            this.autoSort = false;
            return;
        }

        char currentColor = idenColor();

        if (autoSort && currentColor != pattern[pos]) {
            servoArunc.setPosition(1);
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

    public void Abort() {
        autoThrow = false;      // Stop the ThrowAll loop
        throwState = 0;         // Reset the state machine to start
        turns = 0;              // Reset ball count
        servoArunc.setPosition(1.0); // Reset flicker to home

        // Kill motor movement and lock it at current position
        storageMotor.setPower(0);
        storageMotor.setTargetPosition(storageMotor.getCurrentPosition());
        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ManualMove(double power){
        storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        storageMotor.setPower(power);
    }

    public void RestoreAuto(){
        storageMotor.setTargetPosition(storageMotor.getCurrentPosition());
        storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public float hue;
    public float sat;

    public char idenColor() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        hue = hsvValues[0];
        sat = hsvValues[1];
        if (sat < 0.35)
            return 'P';
        else if (hue > 120 && hue < 150)
            return 'G';
        else
            return 'N';
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
        return storageMotor.isBusy();
    }
}
