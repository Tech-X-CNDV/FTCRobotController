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
    private final ElapsedTime driftTimer = new ElapsedTime();

    private int consecutivePresses = 0;
    private static final int DEBOUNCE_THRESHOLD = 2; // Required consecutive loops of sensor detection

    // --- TUNING CONSTANTS ---
    private static final int RECOIL_TICKS_DELTA = -100; // How much to pull back to clear the gate
    private static final int BACKOFF_TICKS = -65; // Increased to ensure clear sensor release
    private static final double TICKS_PER_REV = 384.5;
    private static final double FULL_CYCLE_TICKS = 3 * TICKS_PER_REV; // 3 ball slots per drum revolution (1:1 per slot)

    private static final double GATE_CLOSE_POS = 0.15;
    private static final double GATE_OPEN_POS = 0;

    private static final double HOMING_FAST_VELOCITY = 800; // Lowered for precision and stability while driving
    private static final double HOMING_SLOW_VELOCITY = 300; // Reduced for precision (overshoot)

    private static final double SHOOT_POWER = 1;
    private static final double RECOIL_POWER = 0.6;

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
                // Wait for BOTH the motor to reach the back position AND the gate to finish moving
                boolean motorDone = !storageMotor.isBusy() || Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) <= 5;
                boolean gateDone = timer.milliseconds() > GATE_MOVEMENT_TIME_MS;
                boolean recoilTimeout = timer.milliseconds() > 1000; // Safety timeout to prevent getting stuck

                if ((motorDone && gateDone) || recoilTimeout) {
                    // Target a full drum cycle (3 motor revolutions) to push all 3 balls and land on home
                    int shootStart = storageMotor.getCurrentPosition();
                    storageMotor.setTargetPosition(shootStart + (int) FULL_CYCLE_TICKS);
                    storageMotor.setMode(RunMode.RUN_TO_POSITION);
                    storageMotor.setPower(SHOOT_POWER);
                    timer.reset();
                    currentState = State.SHOOTING;
                }
                break;

            case NUDGING:
                // Effectively merged into RECOILING, kept for enum compatibility
                currentState = State.IDLE; 
                break;

            case SHOOTING:
                double elapsed = timer.milliseconds();

                // Divide the duration into 3 segments for the 3 balls
                if (outtake != null) {
                    if (elapsed < (SHOOTING_DURATION_MS / 3.0)) {
                        outtake.autoShotOffset = -50; // First ball
                    } else if (elapsed < (2.0 * SHOOTING_DURATION_MS / 3.0)) {
                        outtake.autoShotOffset = 0; // Second ball
                    } else {
                        outtake.autoShotOffset = 50; // Final ball
                    }
                }

                // Complete when motor reaches target (3 revs) or safety timeout
                boolean shootDone = (storageMotor.getCurrentPosition() >= storageMotor.getTargetPosition() - 10) || !storageMotor.isBusy();
                if (shootDone || elapsed > 1000) {
                    if (outtake != null)
                        outtake.autoShotOffset = 0; // CRITICAL: Reset to 0 when done
                    ResetToIntake();
                }
                break;

            case FAST_RESET:
                // 1. IMMEDIATE STOP: If we hit the magnet at any point, we zero and calibrate.
                if (magneticSensor.isPressed()) {
                    consecutivePresses++;
                    if (consecutivePresses >= DEBOUNCE_THRESHOLD) {
                        consecutivePresses = 0;
                        storageMotor.setPower(0);
                        storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
                        storageMotor.setTargetPosition(0);
                        storageMotor.setMode(RunMode.RUN_TO_POSITION);
                        storageMotor.setPower(1.0); // Hold zero
                        currentState = State.IDLE;
                        hasCalibrated = true;
                        
                        if (shootQueued) {
                            shootQueued = false;
                            StartShooting();
                        }
                        return;
                    }
                } else {
                    consecutivePresses = 0;
                }

                // 2. Timeout failsafe
                if (timer.milliseconds() > 1500) {
                    hasCalibrated = false;
                    ResetToIntake(true); // Force full homing if stuck
                    return;
                }

                // 3. ARRIVAL: If we reached the target zero but haven't hit the magnet yet,
                // we might have overshot or undershot. Give the drum a short correction window
                // before bailing to full homing — prevents a premature full-home on fast overshoot.
                int error = Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition());
                if (error < 15 || !storageMotor.isBusy()) {
                    if (timer.milliseconds() > 300) { // Extended settle: let the drum stop fully
                        if (magneticSensor.isPressed()) {
                            consecutivePresses++;
                            if (consecutivePresses >= DEBOUNCE_THRESHOLD) {
                                consecutivePresses = 0;
                                storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
                                storageMotor.setTargetPosition(0);
                                storageMotor.setMode(RunMode.RUN_TO_POSITION);
                                storageMotor.setPower(1.0);
                                currentState = State.IDLE;
                                hasCalibrated = true;

                                if (shootQueued) {
                                    shootQueued = false;
                                    StartShooting();
                                }
                            }
                        } else {
                            consecutivePresses = 0;
                            // Missed the magnet — try a small correction nudge in both directions
                            // before bailing to full homing. Pick the shorter path: nudge forward
                            // (positive, toward the next slot) or back (negative, toward the hit spot).
                            int currentPos = storageMotor.getCurrentPosition();
                            // Nudge toward where the magnet should be: slightly forward if we undershot,
                            // slightly back if we overshot. Use a fixed 25-tick probe in each direction.
                            int nudgeTarget = (currentPos > storageMotor.getTargetPosition())
                                    ? currentPos - 25  // overshot: go back toward magnet
                                    : currentPos + 25; // undershot: go forward toward magnet
                            storageMotor.setTargetPosition(nudgeTarget);
                            storageMotor.setMode(RunMode.RUN_TO_POSITION);
                            storageMotor.setPower(0.5);
                            timer.reset(); // Give another settle window to detect the magnet

                            // If the timeout failsafe (1500ms) fires before we find it, full homing kicks in
                        }
                    }
                }
                break;

            case IDLE:
                // Hold the zero position if we've finished homing
                if (storageMotor.getMode() == RunMode.RUN_TO_POSITION) {
                    int idleError = Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition());
                    if (idleError > 8) {
                        storageMotor.setPower(0.8); // Apply high power to correct the drift immediately
                    } else {
                        storageMotor.setPower(0.2); // Lowered to prevent overheating & buzzing once close
                    }
                } else {
                    storageMotor.setPower(0);
                }

                // Physical drift monitoring: if calibrated and in IDLE, the magnet must be pressed.
                // If it's not pressed, we track it to detect true physical drift (slippage).
                if (hasCalibrated) {
                    if (magneticSensor.isPressed()) {
                        driftTimer.reset();
                    } else {
                        if (driftTimer.milliseconds() > 800) { // If magnet is lost for > 800ms in IDLE
                            hasCalibrated = false; // Mark uncalibrated so it forces re-homing next time or auto-homes
                            ResetToIntake(true); // Force full homing to find the magnet again
                        }
                    }
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
            consecutivePresses++;
            if (consecutivePresses >= DEBOUNCE_THRESHOLD) {
                consecutivePresses = 0;
                // 1. HARD STOP: Immediate power kill
                storageMotor.setPower(0);

                // 2. RESET: Mark this exact spot as "0" temporarily
                storageMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);

                // 3. RELATIVE BACKOFF: Move in reverse
                storageMotor.setTargetPosition(BACKOFF_TICKS);
                storageMotor.setMode(RunMode.RUN_TO_POSITION);
                storageMotor.setPower(0.5); // Increased for reliable backoff

                currentState = State.HOMING_BACKOFF;
            }
        } else {
            consecutivePresses = 0;
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setVelocity(HOMING_FAST_VELOCITY); // Moving Forward (+)
        }
    }

    private void handleHomingBackoff() {
        // Use a positional check instead of isBusy() to prevent the "twitch" transition
        // Target is now relative to the hit position, not 0
        int currentPos = storageMotor.getCurrentPosition();
        int targetPos = storageMotor.getTargetPosition();
        boolean reachedBackoff = Math.abs(currentPos - targetPos) < 4;
        boolean sensorReleased = !magneticSensor.isPressed();

        // Ensure we both reach the position AND the sensor is clear to handle wide
        // magnets
        if (reachedBackoff && sensorReleased) {
            storageMotor.setPower(0);
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            timerHome.reset(); // CRITICAL: Reset timer before entering SLOW state
            currentState = State.HOMING_SLOW;
        } else {
            // If we reached the target but sensor is still pressed, keep moving back
            if (reachedBackoff && !sensorReleased) {
                storageMotor.setTargetPosition(storageMotor.getTargetPosition() - 15);
            }
            storageMotor.setPower(0.4); // More power to overcome friction
        }
    }

    private void handleHomingSlow() {
        if (magneticSensor.isPressed()) {
            consecutivePresses++;
            if (consecutivePresses >= DEBOUNCE_THRESHOLD) {
                storageMotor.setPower(0);

                // Precision: Find the exact edge. Debounce is minimized as we move slowly.
                if (timerHome.milliseconds() > 20) {
                    consecutivePresses = 0;
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
            }
        } else {
            consecutivePresses = 0;
            timerHome.reset();
            storageMotor.setMode(RunMode.RUN_USING_ENCODER);
            storageMotor.setVelocity(HOMING_SLOW_VELOCITY); // Slow Forward (+)
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
                || currentState == State.HOMING_SLOW || currentState == State.FAST_RESET) {
            shootQueued = true;
            return;
        }

        // CRITICAL FAILSAFE: If not calibrated (magnet position unknown) or if not physically on the magnet, force homing first and queue the shot.
        if (!hasCalibrated || (currentState == State.IDLE && !magneticSensor.isPressed())) {
            shootQueued = true;
            ResetToIntake(true); // Force full homing
            return;
        }

        // Start closing the gate AND reversing the motor at the same time
        CloseGate();
        timer.reset();

        int currentPos = storageMotor.getCurrentPosition();
        storageMotor.setTargetPosition(currentPos + RECOIL_TICKS_DELTA);
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
            storageMotor.setVelocity(HOMING_FAST_VELOCITY);

            OpenGate();
            timerHome.reset();
            currentState = State.HOMING_FAST; // <--- ENSURE THIS LINE EXECUTES
        } else {
            // --- FAST RESET ---
            // --- FAST RESET (Shortest Path to Magnet) ---
            int currentPos = storageMotor.getCurrentPosition();
            double normalizedPos = currentPos % FULL_CYCLE_TICKS;
            if (normalizedPos < 0)
                normalizedPos += FULL_CYCLE_TICKS;

            double delta = (normalizedPos > (FULL_CYCLE_TICKS / 2.0))
                    ? (FULL_CYCLE_TICKS - normalizedPos)
                    : -normalizedPos;

            storageMotor.setTargetPosition((int) (currentPos + delta));
            storageMotor.setMode(RunMode.RUN_TO_POSITION);
            storageMotor.setPower(0.7); // Increased: faster arrival = less coasting overshoot

            OpenGate();
            timer.reset();
            currentState = State.FAST_RESET;
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