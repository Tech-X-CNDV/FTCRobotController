package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StorageSubsystem {
    private final DcMotorEx storageMotor;
    private final TouchSensor magneticSensor;
    private final Servo servoGate;

    // Explicit states to prevent logic "deadlocks"
    private enum State {
        IDLE, HOMING, RECOILING, NUDGING, SHOOTING
    }

    private State currentState = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // --- TUNING CONSTANTS ---
    private static final int RECOIL_TICKS = -130;
    private static final double GATE_CLOSE_POS = 0.19;
    private static final double GATE_OPEN_POS = 0.0;

    private static final double NUDGE_POWER = 0.05;
    private static final double SHOOT_POWER = 1;
    private static final double RECOIL_POWER = 0.8;

    private static final long GATE_MOVEMENT_TIME_MS = 350;
    private static final long SHOOTING_DURATION_MS = 1200;

    public StorageSubsystem(HardwareMap hardwareMap) {
        storageMotor = hardwareMap.get(DcMotorEx.class, "StorageMotor");
        magneticSensor = hardwareMap.get(TouchSensor.class, "MagneticSensor");
        servoGate = hardwareMap.get(Servo.class, "GateServo");
    }

    public void InitStorage() {
        storageMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        OpenGate();
        currentState = State.IDLE;
    }

    public void update() {
        switch (currentState) {
            case HOMING:
                handleHoming();
                break;

            case RECOILING:
                // Check if motor is finished OR within 5 ticks of target
                boolean closeEnough = Math.abs(storageMotor.getCurrentPosition() - RECOIL_TICKS) <= 5;
                if (!storageMotor.isBusy() || closeEnough) {
                    CloseGate();
                    storageMotor.setMode(RunMode.RUN_USING_ENCODER);
                    storageMotor.setPower(NUDGE_POWER);
                    timer.reset();
                    currentState = State.NUDGING;
                }
                break;

            case NUDGING:
                // Wait for gate to move while pushing the ball forward slowly
                if (timer.milliseconds() > GATE_MOVEMENT_TIME_MS) {
                    storageMotor.setPower(SHOOT_POWER);
                    timer.reset();
                    currentState = State.SHOOTING;
                }
                break;

            case SHOOTING:
                // After 1 second of full power, reset to home
                if (timer.milliseconds() > SHOOTING_DURATION_MS) {
                    ResetToIntake();
                }
                break;

            case IDLE:
                storageMotor.setPower(0);
                break;
        }
    }

    private void handleHoming() {
        OpenGate();
        if (magneticSensor.isPressed()) {
            storageMotor.setPower(0);
            storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setTargetPosition(0);
            storageMotor.setMode(RunMode.RUN_TO_POSITION);
            storageMotor.setPower(1.0);
            currentState = State.IDLE;
        } else {
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setPower(0.2);
        }
    }

    // --- PUBLIC METHODS ---

    public void StartShooting() {
        // Start by backing up
        storageMotor.setTargetPosition(RECOIL_TICKS);
        storageMotor.setMode(RunMode.RUN_TO_POSITION);
        storageMotor.setPower(RECOIL_POWER);
        currentState = State.RECOILING;
    }

    public void ResetToIntake() {
        OpenGate();
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        currentState = State.HOMING;
    }

    public void Abort() {
        storageMotor.setPower(0);
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        currentState = State.IDLE;
    }

    public void OpenGate() {
        servoGate.setPosition(GATE_OPEN_POS);
    }

    public void CloseGate() {
        servoGate.setPosition(GATE_CLOSE_POS);
    }

    // --- GETTERS ---

    public double GetMotorPower() {
        return storageMotor.getPower();
    }

    public double GetServoPosition() {
        return servoGate.getPosition();
    }

    public String getState() {
        return currentState.toString();
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("Storage State", currentState);
        telemetry.addData("Encoder Pos", storageMotor.getCurrentPosition());
        telemetry.addData("Motor Power", GetMotorPower());
        telemetry.addData("Is Busy", storageMotor.isBusy());
    }
}