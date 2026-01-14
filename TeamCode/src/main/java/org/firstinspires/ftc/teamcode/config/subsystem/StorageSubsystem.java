package org.firstinspires.ftc.teamcode.config.subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
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

    public boolean autoThrow = false;

    public void ThrowAll() {
        if (turns > 3) {
            turns = 0;
            servoArunc.setPosition(1);
            this.autoThrow = false;
        }
        if (!storageMotor.isBusy()) {
            servoArunc.setPosition(0.6);
            servoTimer.reset();
        }
        if (servoTimer.seconds() > 1) {
            if (autoThrow) {
                servoArunc.setPosition(1);
                MoveToPosition(475, 1);
                turns++;
                if (turns > 3)
                    this.autoThrow = false;
            }
        }
    }

    int pos = 0, turns = 0;
    ElapsedTime checkTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();
    boolean isMoving = false;
    public boolean autoSort = false;

    public void PatternSortAuto(char[] pattern) {
        if (storageMotor.isBusy()) {
            isMoving = true;
            return;
        }

        if (isMoving) {
            checkTimer.reset();
            isMoving = false;
        }
        if (checkTimer.seconds() > 2 && servoTimer.seconds() > 1) {
            MoveToPosition(475, 1);
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
            turns++;
            if (turns > 3) this.autoSort = false;
        } else if (currentColor == pattern[pos]) {
            turns = 0;
            pos++;
            servoArunc.setPosition(0.6);
            servoTimer.reset();
        }
    }

    public char idenColor() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        float hue = hsvValues[0];
        float sat = hsvValues[1];
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
}
