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

    private OuttakeSubsystem outtake;

    private enum State {
        IDLE,
        HOMING_FAST, // Rapidly seeking the sensor
        HOMING_BACKOFF, // Moving away to clear the sensor signal
        HOMING_SLOW, // Precision approach for the final zero
        RECOILING,
        NUDGING,
        SHOOTING,
        FAST_RESET,
        MANUAL
    }

    private boolean hasCalibrated = false;

    private State currentState = State.IDLE;
    private boolean shootQueued = false;
    private boolean openedGate = false;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime timerHome = new ElapsedTime();

    // --- TUNING CONSTANTS ---
    private static final int RECOIL_TICKS = -110;
    private static final int BACKOFF_TICKS = -50; // Distance to move away from sensor
    private static final double TICKS_PER_REV = 384.5;

    private static final double GATE_CLOSE_POS = 0.23;
    private static final double GATE_OPEN_POS = 0.04;

    private static final double HOMING_FAST_POWER = 0.3;
    private static final double HOMING_SLOW_POWER = 0.1; // Increased to prevent stalling during auto

    private static final double NUDGE_POWER = 0.1;
    private static final double SHOOT_POWER = 1;
    private static final double RECOIL_POWER = 0.7;

    private static final long GATE_MOVEMENT_TIME_MS = 560;
    private static final long SHOOTING_DURATION_MS = 650;

    public StorageSubsystem(HardwareMap hardwareMap, OuttakeSubsystem outtake) {
        this.outtake = outtake;
        storageMotor = hardwareMap.get(DcMotorEx.class, "StorageMotor");
        magneticSensor = hardwareMap.get(TouchSensor.class, "MagneticSensor");
        servoGate = hardwareMap.get(Servo.class, "GateServo");
    }

    public void InitStorage() {
        storageMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        OpenGate();
        shootQueued = false;
        hasCalibrated = false;
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
                    storageMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
                    storageMotor.setPower(SHOOT_POWER);
                    timer.reset();
                    currentState = State.SHOOTING;
                }
                break;

            case SHOOTING:
                double elapsed = timer.milliseconds();

                // Divide the duration into 3 segments for the 3 balls
                if (elapsed < (SHOOTING_DURATION_MS / 3.0)) {
                    outtake.autoShotOffset = -50; // First ball
                } else if (elapsed < (2.0 * SHOOTING_DURATION_MS / 3.0)) {
                    outtake.autoShotOffset = 0; // Second ball
                } else {
                    outtake.autoShotOffset = 50; // Final ball
                }

                if (elapsed > SHOOTING_DURATION_MS) {
                    outtake.autoShotOffset = 0; // CRITICAL: Reset to 0 when done
                    ResetToIntake();
                }
                break;

            case FAST_RESET:
                // 1. Timeout failsafe
                if (timer.milliseconds() > 1500) {
                    hasCalibrated = false;
                    ResetToIntake(true); // Force full homing if stuck
                    return;
                }

                int error = Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition());
                // 2. More generous tolerance (15 ticks) for when the robot is moving
                if (error < 15 || !storageMotor.isBusy()) {
                    // 3. Brief pause to let the magnetic sensor stabilize
                    if (timer.milliseconds() > 700) {
                        if (magneticSensor.isPressed()) {
                            storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
                            storageMotor.setTargetPosition(0);
                            storageMotor.setMode(RunMode.RUN_TO_POSITION);
                            storageMotor.setPower(1.0); // Hold zero
                            currentState = State.IDLE;
                        } else {
                            // We missed the magnet! Go back to full homing
                            hasCalibrated = false;
                            ResetToIntake(true);
                        }
                    }
                }
                break;

            case IDLE:
                // Hold the zero position if we've finished homing
                if (storageMotor.getMode() == RunMode.RUN_TO_POSITION) {
                    storageMotor.setPower(1.0);
                } else {
                    storageMotor.setPower(0);
                }
                break;

            case MANUAL:
                // Motor power is set externally via Abort(power)
                break;
        }
    }

    // --- HOMING LOGIC STEPS ---

    private void handleHomingFast() {
        if (!openedGate)
            OpenGate();

        if (timerHome.seconds() > 5.0) {
            Abort();
            return;
        }

        if (magneticSensor.isPressed()) {
            // 1. HARD STOP: This kills the momentum that causes overshooting in Auto
            storageMotor.setPower(0);

            // 2. RESET: Mark this exact spot as "0" temporarily
            // This prevents the motor from trying to "run back" 360 degrees
            storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);

            // 3. RELATIVE BACKOFF: Move in reverse (e.g., -50 ticks)
            storageMotor.setTargetPosition(BACKOFF_TICKS);
            storageMotor.setMode(RunMode.RUN_TO_POSITION);
            storageMotor.setPower(0.3);

            currentState = State.HOMING_BACKOFF;
        } else {
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setPower(HOMING_FAST_POWER); // Moving Forward (+)
        }
    }

    private void handleHomingBackoff() {
        // Use a positional check instead of isBusy() to prevent the "twitch" transition
        // Target is now relative to the hit position, not 0
        int currentPos = storageMotor.getCurrentPosition();
        int targetPos = storageMotor.getTargetPosition();
        boolean reachedBackoff = Math.abs(currentPos - targetPos) < 5;

        if (reachedBackoff) {
            storageMotor.setPower(0);
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            currentState = State.HOMING_SLOW;
        } else {
            // MANDATORY ENFORCEMENT: Ensure the backoff move is completed
            storageMotor.setPower(0.15);
        }
    }

    private void handleHomingSlow() {
        if (magneticSensor.isPressed()) {
            storageMotor.setPower(0);

            // Debounce: Wait 50ms to ensure the signal is stable
            if (timerHome.milliseconds() > 50) {
                storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
                storageMotor.setTargetPosition(0);
                storageMotor.setMode(RunMode.RUN_TO_POSITION);
                storageMotor.setPower(1.0); // Hold position at 0

                currentState = State.IDLE;
                hasCalibrated = true;

                if (shootQueued) {
                    shootQueued = false;
                    StartShooting();
                }
            }
        } else {
            timerHome.reset();
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setPower(HOMING_SLOW_POWER); // Slow Forward (+)
        }
    }

    // --- PUBLIC METHODS ---

    public void StartShooting() {
        // Guard: If we are already shooting or have a shot queued, ignore new requests
        if (shootQueued || currentState == State.RECOILING || currentState == State.NUDGING
                || currentState == State.SHOOTING) {
            return;
        }

        if (currentState == State.HOMING_FAST || currentState == State.HOMING_BACKOFF
                || currentState == State.HOMING_SLOW) {
            shootQueued = true;
            return;
        }
        storageMotor.setTargetPosition(RECOIL_TICKS);
        storageMotor.setMode(RunMode.RUN_TO_POSITION);
        storageMotor.setPower(RECOIL_POWER);
        currentState = State.RECOILING;
    }

    // Default version (used by the state machine)
    public void ResetToIntake() {
        ResetToIntake(false); // Default to fast reset if possible
    }

    // Overloaded version for manual calls from OpMode
    public void ResetToIntake(boolean forceFullHoming) {
        storageMotor.setPower(0); // Kill shooting power immediately

        if (!hasCalibrated || forceFullHoming) {
            // --- FULL HOMING ---
            hasCalibrated = false;
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setPower(HOMING_FAST_POWER);

            OpenGate();
            timerHome.reset();
            currentState = State.HOMING_FAST; // <--- ENSURE THIS LINE EXECUTES
        } else {
            // --- FAST RESET ---
            int currentPos = storageMotor.getCurrentPosition();
            double normalizedPos = currentPos % TICKS_PER_REV;
            if (normalizedPos < 0)
                normalizedPos += TICKS_PER_REV;

            double delta = (normalizedPos > (TICKS_PER_REV / 2.0))
                    ? (TICKS_PER_REV - normalizedPos)
                    : -normalizedPos;

            storageMotor.setTargetPosition((int) (currentPos + delta));
            storageMotor.setMode(RunMode.RUN_TO_POSITION);
            storageMotor.setPower(0.6);

            OpenGate();
            timer.reset();
            currentState = State.FAST_RESET; // <--- ENSURE THIS LINE EXECUTES
        }
    }

    public void Abort() {
        OpenGate();
        shootQueued = false;
        hasCalibrated = false; // Force re-home after an abort
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
        shootQueued = false;
        storageMotor.setMode(RunMode.RUN_USING_ENCODER);
        storageMotor.setPower(power);
        if (Math.abs(power) > 0.05) {
            hasCalibrated = false; // Encoder may drift during manual move
            currentState = State.MANUAL;
        } else {
            currentState = State.IDLE;
        }
    }

    public void OpenGate() {
        servoGate.setPosition(GATE_OPEN_POS);
        openedGate = true;
    }

    public void CloseGate() {
        servoGate.setPosition(GATE_CLOSE_POS);
        openedGate = false;
    }

    // --- GETTERS ---

    public boolean isIdle() {
        return currentState == State.IDLE;
    }

    public boolean isDoneShooting() {
        if (shootQueued)
            return false;

        // Return true as soon as we start homing to allow driving in parallel
        return currentState == State.IDLE ||
                currentState == State.HOMING_FAST ||
                currentState == State.HOMING_BACKOFF ||
                currentState == State.HOMING_SLOW ||
                currentState == State.FAST_RESET;
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

    public boolean isGateOpen() {
        return openedGate;
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("Storage State", currentState);
        telemetry.addData("Encoder Pos", storageMotor.getCurrentPosition());
        telemetry.addData("Motor Power", GetMotorPower());
        telemetry.addData("Sensor Pressed", magneticSensor.isPressed());
        telemetry.addData("Servo Position", GetServoPosition());
        telemetry.addData("Gate Open", openedGate);
        telemetry.addData("Shoot Queued", shootQueued);
    }
}