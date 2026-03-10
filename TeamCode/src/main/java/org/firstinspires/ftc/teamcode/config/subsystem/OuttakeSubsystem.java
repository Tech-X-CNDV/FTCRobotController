package org.firstinspires.ftc.teamcode.config.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class OuttakeSubsystem {
    private final HuskyLens hLens;
    private final DcMotorEx shootMotor, shootMotor2;
    private final Servo outtakeAngle;
    private final CRServo turretServo1, turretServo2;
    private final DcMotorEx turretEncoder;

    public double targetVelocity = 2000;
    public double flywheelP = 30;
    public double flywheelF = 12.150;
    public double flywheelStaticF = 0.03;
    private double lastAppliedTarget = -1;
    private double lastAppliedP = -1;
    private double lastAppliedF_base = -1;
    private double lastAppliedStaticF = -1;
    private final ElapsedTime pidfUpdateTimer = new ElapsedTime();
    private static final double PIDF_UPDATE_INTERVAL_MS = 100; // Update max 10 times per second

    // Flywheel Velocity Constants (ticks/sec)
    public static double DEFAULT_SHOOT_VELOCITY = 2000; // Default target velocity
    public static double AUTO_SHOOT_VELOCITY = 2000; // Used by auto-aim static mode
    private boolean shootMotorEnabled = false;
    private double manualVelocityOffset = 0;
    private double manualAngleOffset = 0;
    public static double INITIAL_ANGLE = 0.9;

    // Turret CRServo Constants (Axon Max)
    public static double MAX_TURRET_ANGLE_DEG = 180; // Expanded limit for Axon
    public static double HUSKYLENS_FOV_DEG = 60.0;
    public static double TURRET_ENCODER_OFFSET_DEG = 0.0; // Subtract from raw reading to zero
    // If the Through Bore encoder sits on the SERVO shaft and the servo drives a
    // 10:1 gearbox to the turret, set this to 10.0. Every servo revolution = 1/10
    // turret revolution, so we must divide encoder ticks by this ratio.
    public static double TURRET_GEAR_RATIO = 7.46; // 1.0 = encoder is directly on turret shaft. TUNE THIS!
    private static boolean hasBeenReset = false;

    private double turretTargetAngleDeg = 0;
    public int TARGET_TAG_ID = 1;
    private boolean isHuskyLock = false;
    private boolean autoAimEnabled = false;

    // Turret PID Constants (Tune these for Power control)
    public static double turretP = 0.04;
    public static double turretI = 0.0001;
    public static double turretD = 0.001;
    public static double turretF = 0.01; // Static friction feedforward
    public static double turretD_LPF = 0.8; // Low-pass filter for derivative

    // Turret State
    private double turretIntegralSum = 0;
    private double lastTurretError = 0;
    private double lastFilteredDerivative = 0;
    private final ElapsedTime turretTimer = new ElapsedTime();

    // Velocity Distance Scaling for shoot motor
    public static double MIN_SHOOT_VELOCITY = 1300; // Ticks/sec for close shots
    public static double MAX_SHOOT_VELOCITY = 2770; // Ticks/sec for far shots (also used as hard clamp)
    public static double MAX_VELOCITY = 2770; // Absolute max the motor can physically do
    public static double VELOCITY_DISTANCE_SCALING = 5; // Extra ticks/sec per cm of distance

    // Angle Distance Scaling for outtake angle
    private final double CLOSE_DIST = 30.0;
    private final double FAR_DIST = 65.0;
    private final double CLOSE_ANGLE = 0.15;
    private final double FAR_ANGLE = 0.7;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        shootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "ShootMotor2");
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "PrindMotor");
        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
    }

    private boolean huskyLensInitialized = false;

    public void InitOuttake() {
        targetVelocity = DEFAULT_SHOOT_VELOCITY;

        shootMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shootMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        shootMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretServo1.setDirection(CRServo.Direction.REVERSE);
        turretServo2.setDirection(CRServo.Direction.REVERSE);
        turretServo1.setPower(0);
        turretServo2.setPower(0);

        // Reset the encoder so turret starts at 0 degrees wherever it is physically
        // placed
        if (!hasBeenReset) {
            turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hasBeenReset = true;
        }

        turretTargetAngleDeg = 0; // We start at 0 by definition
        lastFilteredDerivative = 0;
        turretTimer.reset();
    }

    public void InitVision() {
        if (huskyLensInitialized)
            return;
        hLens.initialize();
        hLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLensInitialized = true;
    }

    public void update() {
        if (!shootMotorEnabled) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
        } else {
            // DIRECT VELOCITY
            updatePIDF(); // Recalculate F based on targetVelocity
            shootMotor.setVelocity(targetVelocity);
            shootMotor2.setVelocity(targetVelocity);
        }

        // Always run turret PID loop
        updateTurretPID();
    }

    public void updateAutoAimPower(double deltaX, double deltaY) {
        double distance = Math.hypot(deltaX, deltaY);
        // Velocity scales linearly with distance
        double velocity = MIN_SHOOT_VELOCITY + (distance * VELOCITY_DISTANCE_SCALING) + manualVelocityOffset;
        targetVelocity = Math.max(MIN_SHOOT_VELOCITY, Math.min(velocity, MAX_SHOOT_VELOCITY));
    }

    public void updateStaticPower() {
        double velocity = AUTO_SHOOT_VELOCITY + manualVelocityOffset;
        targetVelocity = Math.max(MIN_SHOOT_VELOCITY, Math.min(velocity, MAX_SHOOT_VELOCITY));
    }

    public void updateFixedPower(double basePower) {
        // Accept legacy ratio (0.0-1.0) from old callers and convert to velocity
        targetVelocity = Math.max(MIN_SHOOT_VELOCITY, Math.min(basePower * MAX_VELOCITY, MAX_SHOOT_VELOCITY));
    }

    public void updateAutoAimAngle(double deltaX, double deltaY) {
        double distance = Math.hypot(deltaX, deltaY);

        // Map distance to angle: 0.15 (close) to 0.9 (far) + ManualOffset
        double angle = CLOSE_ANGLE + (distance - CLOSE_DIST) * (FAR_ANGLE - CLOSE_ANGLE) / (FAR_DIST - CLOSE_DIST)
                + manualAngleOffset;

        // Clamp to [0.5, 0.9]
        angle = Math.max(CLOSE_ANGLE, Math.min(FAR_ANGLE, angle));

        SetAngle(angle);
    }

    public void SetAutoAim(boolean enabled) {
        this.autoAimEnabled = enabled;
        if (enabled)
            InitVision();
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public void ToggleShootMotor() {
        shootMotorEnabled = !shootMotorEnabled;
    }

    public void ToggleShootMotorAuto() {
        shootMotorEnabled = !shootMotorEnabled;
    }

    public void SetShootMotorPower(double power) {
        // Legacy shim: accepts 0.0-1.0 ratio, converts to velocity
        targetVelocity = power * MAX_VELOCITY;
        shootMotorEnabled = (power > 0);
    }

    public void setTargetVelocity(double velocityTicksPerSec) {
        targetVelocity = Math.max(0, Math.min(velocityTicksPerSec, MAX_VELOCITY));
        shootMotorEnabled = (targetVelocity > 0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setManualVelocityOffset(double offset) {
        this.manualVelocityOffset = offset;
    }

    public double getManualVelocityOffset() {
        return manualVelocityOffset;
    }

    /** @deprecated Use getManualVelocityOffset() */
    public double getManualPowerOffset() {
        return manualVelocityOffset / MAX_VELOCITY;
    }

    /** @deprecated Use setManualVelocityOffset() */
    public void setManualPowerOffset(double offset) {
        this.manualVelocityOffset = offset * MAX_VELOCITY;
    }

    // Kept for backward compat
    public double getTargetBasePower() {
        return targetVelocity / MAX_VELOCITY;
    }

    public double getManualAngleOffset() {
        return manualAngleOffset;
    }

    public void setManualAngleOffset(double offset) {
        this.manualAngleOffset = offset;
    }

    public void IncreaseAngleOffset() {
        this.manualAngleOffset += 0.05;
    }

    public void DecreaseAngleOffset() {
        this.manualAngleOffset -= 0.05;
    }

    public void IncreaseDirectAngle() {
        double currentAngle = outtakeAngle.getPosition();
        double newAngle = Math.min(1.0, currentAngle + 0.05);
        SetAngle(newAngle);
    }

    public void DecreaseDirectAngle() {
        double currentAngle = outtakeAngle.getPosition();
        double newAngle = Math.max(0.0, currentAngle - 0.05);
        SetAngle(newAngle);
    }

    public boolean isReadyToFire() {
        // Ready if enabled and velocity is within 10% of our expected target velocity
        double currentVelocity = getVelocity();
        return shootMotorEnabled
                && (currentVelocity >= targetVelocity * 0.90 && currentVelocity <= targetVelocity * 1.1);
    }

    public void AutoAngle() {
        outtakeAngle.setPosition(0.7);
    }

    public void SetAngle(double angle) {
        outtakeAngle.setPosition(angle);
    }

    public HuskyLens.Block[] GetCameraFeed() {
        return hLens.blocks();
    }

    public void updateTurretLock(Pose robotPose, Pose targetPose, double manualOffset) {
        InitVision(); // Ensure HuskyLens is initialized
        HuskyLens.Block[] blocks = hLens.blocks();
        HuskyLens.Block targetBlock = null;

        for (HuskyLens.Block block : blocks) {
            if (block.id == TARGET_TAG_ID) {
                targetBlock = block;
                break;
            }
        }

        if (targetBlock != null) {
            // Precise Lock using HuskyLens
            double visualErrorDeg = (targetBlock.x - 160) * (HUSKYLENS_FOV_DEG / 320.0);

            // Add relative visual error to our current absolute target
            double rawTargetDeg = turretTargetAngleDeg + visualErrorDeg + (manualOffset * 57.2958);

            // Note: In visual mode, we dampen the update to prevent crazy oscillation if
            // the tag shakes
            // A simple proportional approach is often better than raw absolute summing
            turretTargetAngleDeg = turretTargetAngleDeg + (rawTargetDeg - turretTargetAngleDeg) * 0.3; // Very basic
                                                                                                       // P-gain for
                                                                                                       // vision

            isHuskyLock = true;
        } else {
            // Fallback to Pedro Pathing Pose Lock
            double deltaX = targetPose.getX() - robotPose.getX();
            double deltaY = targetPose.getY() - robotPose.getY();

            double angleToTarget = Math.atan2(deltaY, deltaX);
            double relativeAngle = angleToTarget - robotPose.getHeading() + Math.toRadians(5) + manualOffset;

            while (relativeAngle > Math.PI)
                relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI)
                relativeAngle += 2 * Math.PI;

            turretTargetAngleDeg = Math.toDegrees(relativeAngle);
            isHuskyLock = false;
        }

        // Target angle is naturally constrained by the physical robot, but we can clamp
        // the target here
        if (turretTargetAngleDeg > MAX_TURRET_ANGLE_DEG)
            turretTargetAngleDeg = MAX_TURRET_ANGLE_DEG;
        if (turretTargetAngleDeg < -MAX_TURRET_ANGLE_DEG)
            turretTargetAngleDeg = -MAX_TURRET_ANGLE_DEG;
    }

    private double getRawTurretEncoderAngle() {
        // Rev Through Bore Encoder gives precisely 8192 ticks per full revolution of
        // whatever shaft IT is mounted on (servo or turret directly).
        // Dividing by TURRET_GEAR_RATIO converts from servo-shaft revolutions
        // to actual turret-output revolutions.
        double ticks = -turretEncoder.getCurrentPosition();
        return (ticks / (8192.0 * TURRET_GEAR_RATIO)) * 360.0;
    }

    public double getMeasuredTurretAngle() {
        double rawAngle = getRawTurretEncoderAngle();
        double adjustedAngle = rawAngle - TURRET_ENCODER_OFFSET_DEG;

        // Wrap to -180 to 180 to match standard FTC conventions
        while (adjustedAngle > 180)
            adjustedAngle -= 360;
        while (adjustedAngle < -180)
            adjustedAngle += 360;

        return adjustedAngle;
    }

    private void updateTurretPID() {
        double currentAngle = getMeasuredTurretAngle();
        double error = turretTargetAngleDeg - currentAngle;

        // Adjust for shortest path if needed
        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        double dt = turretTimer.seconds();
        turretTimer.reset();

        if (dt > 0.1)
            dt = 0.01; // Prevent massive spikes

        turretIntegralSum += error * dt;

        // Anti-windup
        if (Math.abs(turretIntegralSum) > 0.2 / (turretI + 0.000001)) {
            turretIntegralSum = Math.signum(turretIntegralSum) * (0.2 / (turretI + 0.000001));
        }

        double rawDerivative = (error - lastTurretError) / dt;
        lastTurretError = error;

        // LPF on derivative
        double filteredDerivative = (turretD_LPF * lastFilteredDerivative) + ((1 - turretD_LPF) * rawDerivative);
        lastFilteredDerivative = filteredDerivative;

        double power = (error * turretP) + (turretIntegralSum * turretI) + (filteredDerivative * turretD);

        // Feedforward for static friction
        if (Math.abs(error) > 1.0) { // 1 degree deadband for static F
            power += Math.signum(error) * turretF;
        }

        // Clamp power and apply
        power = Math.max(-1.0, Math.min(1.0, power));

        turretServo1.setPower(power);
        turretServo2.setPower(power);
    }

    public void resetTurret() {
        turretTargetAngleDeg = 0;
    }

    public void resetTurretEncoder() {
        // Stop the motor briefly to reset the count
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reset our software targets so they don't jump
        turretTargetAngleDeg = 0;
        turretIntegralSum = 0;
        lastTurretError = 0;
    }

    public void setTurretTargetAngle(double angleDeg) {
        this.turretTargetAngleDeg = angleDeg;
        // Clamp Angle
        if (turretTargetAngleDeg > MAX_TURRET_ANGLE_DEG)
            turretTargetAngleDeg = MAX_TURRET_ANGLE_DEG;
        if (turretTargetAngleDeg < -MAX_TURRET_ANGLE_DEG)
            turretTargetAngleDeg = -MAX_TURRET_ANGLE_DEG;
    }

    public double getTurretAngle() {
        return getMeasuredTurretAngle(); // Now returns the actual real-world angle!
    }

    public double getVelocity() {
        return (shootMotor.getVelocity() + shootMotor2.getVelocity()) / 2;
    }

    public void setFlywheelPIDF(double p, double f, double s) {
        this.flywheelP = p;
        this.flywheelF = f;
        this.flywheelStaticF = s;
    }

    private void updatePIDF() {
        // 1. Performance Guard
        if (pidfUpdateTimer.milliseconds() < PIDF_UPDATE_INTERVAL_MS)
            return;

        double appliedF = 0;

        // 2. The "Direct Power" Math
        if (Math.abs(targetVelocity) > 1) {
            // flywheelF (kV): Power per tick/sec
            // flywheelStaticF (kS): Flat power to overcome gearbox friction

            // We divide staticF by targetVelocity because the REV Hub
            // internally MULTIPLIES the whole F by targetVelocity.
            // This ensures flywheelStaticF acts as a constant "Base Power" boost.
            appliedF = flywheelF + (flywheelStaticF / Math.abs(targetVelocity));
        }

        // 3. Significant Change Detection
        // Lowered threshold to 20 for more precise tuning
        if (Math.abs(targetVelocity - lastAppliedTarget) > 20 ||
                flywheelP != lastAppliedP ||
                flywheelF != lastAppliedF_base ||
                flywheelStaticF != lastAppliedStaticF) {

            com.qualcomm.robotcore.hardware.PIDFCoefficients coefficients = new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                    flywheelP, 0, 0, appliedF);

            shootMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
            shootMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

            // Update tracking variables
            lastAppliedTarget = targetVelocity;
            lastAppliedP = flywheelP;
            lastAppliedF_base = flywheelF;
            lastAppliedStaticF = flywheelStaticF;
            pidfUpdateTimer.reset();
        }
    }

    public void displayTelemetry(Telemetry telemetry) {
        if (isReadyToFire()) {
            telemetry.addLine("<h1><font color='#00FF00'>*** READY TO FIRE ***</font></h1>");
        } else if (shootMotorEnabled) {
            telemetry.addLine("<i>Spinning Up...</i>");
        } else {
            telemetry.addLine("<i>Stopped</i>");
        }
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Outtake Angle Target", "%.2f", outtakeAngle.getPosition());
        telemetry.addData("ShootMotor Velocity", getVelocity());
        telemetry.addData("Turret Target Angle", "%.1f deg", turretTargetAngleDeg);
        telemetry.addData("Turret Raw Angle", "%.1f deg", getRawTurretEncoderAngle());
        telemetry.addData("Turret Lock Mode", isHuskyLock ? "HUSKY (PRECISE)" : "POSE (FALLBACK)");
    }
}