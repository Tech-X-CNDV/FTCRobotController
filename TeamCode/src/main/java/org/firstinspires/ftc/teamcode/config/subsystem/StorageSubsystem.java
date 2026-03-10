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

    private enum State {
        IDLE,
        HOMING_FAST, // Rapidly seeking the sensor
        HOMING_BACKOFF, // Moving away to clear the sensor signal
        HOMING_SLOW, // Precision approach for the final zero
        RECOILING,
        NUDGING,
        SHOOTING,
        MANUAL
    }

    private State currentState = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // --- TUNING CONSTANTS ---
    private static final int RECOIL_TICKS = -120;
    private static final int BACKOFF_TICKS = -80; // Distance to move away from sensor

    private static final double GATE_CLOSE_POS = 0.235;
    private static final double GATE_OPEN_POS = 0.0;

    private static final double HOMING_FAST_POWER = 0.65;
    private static final double HOMING_SLOW_POWER = 0.08; // High precision speed

    private static final double NUDGE_POWER = 0.1;
    private static final double SHOOT_POWER = 1.0;
    private static final double RECOIL_POWER = 0.8;

    private static final long GATE_MOVEMENT_TIME_MS = 550;
    private static final long SHOOTING_DURATION_MS = 600;

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
            case HOMING_FAST:
                handleHomingFast();
                break;

            case HOMING_BACKOFF:
                handleHomingBackoff();
                break;

            case HOMING_SLOW:
                handleHomingSlow();
                break;

            case RECOILING:
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
                if (timer.milliseconds() > GATE_MOVEMENT_TIME_MS) {
                    storageMotor.setPower(SHOOT_POWER);
                    timer.reset();
                    currentState = State.SHOOTING;
                }
                break;

            case SHOOTING:
                if (timer.milliseconds() > SHOOTING_DURATION_MS) {
                    ResetToIntake();
                }
                break;

            case IDLE:
                storageMotor.setPower(0);
                break;

            case MANUAL:
                // Motor power is set externally via Abort(power)
                break;
        }
    }

    // --- HOMING LOGIC STEPS ---

    private void handleHomingFast() {
        OpenGate();
        if (magneticSensor.isPressed()) {
            storageMotor.setPower(0);
            // Switch to RUN_TO_POSITION to back off accurately
            storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setTargetPosition(BACKOFF_TICKS);
            storageMotor.setMode(RunMode.RUN_TO_POSITION);
            storageMotor.setPower(0.3);
            currentState = State.HOMING_BACKOFF;
        } else {
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setPower(HOMING_FAST_POWER);
        }
    }

    private void handleHomingBackoff() {
        // Once the motor reaches the backoff position, start the slow crawl
        if (!storageMotor.isBusy()) {
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            currentState = State.HOMING_SLOW;
        }
    }

    private void handleHomingSlow() {
        if (magneticSensor.isPressed()) {
            storageMotor.setPower(0);
            // Final Zero calibration
            storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setTargetPosition(0);
            storageMotor.setMode(RunMode.RUN_TO_POSITION);
            storageMotor.setPower(1.0);
            currentState = State.IDLE;
        } else {
            // Crawl forward slowly to minimize inertia overshoot
            storageMotor.setPower(HOMING_SLOW_POWER);
        }
    }

    // --- PUBLIC METHODS ---

    public void StartShooting() {
        storageMotor.setTargetPosition(RECOIL_TICKS);
        storageMotor.setMode(RunMode.RUN_TO_POSITION);
        storageMotor.setPower(RECOIL_POWER);
        currentState = State.RECOILING;
    }

    public void ResetToIntake() {
        OpenGate();
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        currentState = State.HOMING_FAST;
    }

    public void Abort() {
        OpenGate();
        storageMotor.setPower(0);
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        currentState = State.IDLE;
    }

    /**
     * Overloaded Abort to allow manual movement.
     * Switches state to MANUAL and applies power.
     */
    public void Abort(double power) {
        OpenGate();
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        storageMotor.setPower(power);
        if (Math.abs(power) > 0.05) {
            currentState = State.MANUAL;
        } else {
            currentState = State.IDLE;
        }
    }

    public void OpenGate() {
        servoGate.setPosition(GATE_OPEN_POS);
    }

    public void CloseGate() {
        servoGate.setPosition(GATE_CLOSE_POS);
    }

    // --- GETTERS ---

    public boolean isIdle() {
        return currentState == State.IDLE;
    }

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
        telemetry.addData("Sensor Pressed", magneticSensor.isPressed());
    }
}