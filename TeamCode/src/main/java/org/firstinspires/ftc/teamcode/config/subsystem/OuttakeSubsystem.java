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

    public double targetVelocity = 0;
    public double flywheelP = 50;
    public double flywheelF = 13.350;
    public double flywheelD = 5.0;
    private double lastAppliedD = -1;
    private double lastAppliedTarget = -1;
    private double lastAppliedP = -1;
    private double lastAppliedF_base = -1;
    private final ElapsedTime pidfUpdateTimer = new ElapsedTime();
    private static final double PIDF_UPDATE_INTERVAL_MS = 100; // Update max 10 times per second

    // Flywheel Velocity Constants (ticks/sec)
    public static double DEFAULT_SHOOT_VELOCITY = 2000; // Default target velocity
    public static double AUTO_SHOOT_VELOCITY = 2000; // Used by auto-aim static mode
    public static double IDLE_SHOOT_VELOCITY = 1000; // Lower speed to save battery/motor heat
    public double autoShotOffset = 0; // Temporary offset for staged shooting
    private boolean shootMotorEnabled = false;
    private double manualVelocityOffset = 0;
    private double manualAngleOffset = 0;
    public static double INITIAL_ANGLE = 0.9;

    // Turret CRServo Constants (Axon Max)
    public static double TURRET_LIMIT_LEFT = -180;
    public static double TURRET_LIMIT_RIGHT = 180;
    public static double HUSKYLENS_FOV_DEG = 60.0;
    public static double TURRET_ENCODER_OFFSET_DEG = 0.0; // Subtract from raw reading to zero
    // If the Through Bore encoder sits on the SERVO shaft and the servo drives a
    // 10:1 gearbox to the turret, set this to 10.0. Every servo revolution = 1/10
    // turret revolution, so we must divide encoder ticks by this ratio.
    public static double TURRET_GEAR_RATIO = 7.46; // 1.0 = encoder is directly on turret shaft. TUNE THIS!
    private static boolean hasBeenReset = false;

    private double turretTargetAngleDeg = 0;
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
    public static double VELOCITY_DISTANCE_SCALING = 5.2; // Extra ticks/sec per cm of distance

    // Angle Distance Scaling for outtake angle
    private final double CLOSE_DIST = 30.0;
    private final double FAR_DIST = 65.0;
    private final double CLOSE_ANGLE = 0.15;
    private final double FAR_ANGLE = 0.8;

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
        updateTurretPID();

        if (!shootMotorEnabled) {
            shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }

        // Apply identical target velocities (PIDF control handles spinup natively)
        updatePIDF();
        shootMotor.setVelocity(targetVelocity);
        shootMotor2.setVelocity(targetVelocity);
    }

    public void updateAutoAimPower(double deltaX, double deltaY) {
        double distance = Math.hypot(deltaX, deltaY);
        // Velocity scales linearly with distance
        double velocity = MIN_SHOOT_VELOCITY + (distance * VELOCITY_DISTANCE_SCALING) + manualVelocityOffset
                + autoShotOffset;
        targetVelocity = Math.max(MIN_SHOOT_VELOCITY, Math.min(velocity, MAX_SHOOT_VELOCITY));
    }

    public void updateStaticPower() {
        double velocity = AUTO_SHOOT_VELOCITY + manualVelocityOffset + autoShotOffset;
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

        // Clamp to physical servo limits [0, 1] to allow manual offset to work
        angle = Math.max(0.1, Math.min(1.0, angle));

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

    public void StartShootMotor() {
        shootMotorEnabled = true;
    }

    public void StopShootMotor() {
        shootMotorEnabled = false;
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

    public void SetAngle(double angle) {
        // Centralized safety clamp: prevent the servo from bottoming out below 0.12, 
        // which physically jams/binds the outtake linkage.
        double safeAngle = Math.max(0.12, Math.min(1.0, angle));
        outtakeAngle.setPosition(safeAngle);
    }

    public void AutoAngle() {
        SetAngle(INITIAL_ANGLE);
    }

    public void updateTurretLock(Pose robotPose, Pose targetPose, double manualOffset) {
        // HuskyLens bypassed - using Pedro Pathing Pose Lock only
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();

        double angleToTarget = Math.atan2(deltaY, deltaX);
        double relativeAngle = angleToTarget - robotPose.getHeading() +
                Math.toRadians(5) + manualOffset;

        while (relativeAngle > Math.PI)
            relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI)
            relativeAngle += 2 * Math.PI;

        double newTarget = Math.toDegrees(relativeAngle);
        isHuskyLock = false;

        // Apply limits and roll-over through the centralized setter
        setTurretTargetAngle(newTarget);
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

        // Note: Shortest-path wrapping removed to support software limits.
        // The roll-over logic in setTurretTargetAngle handles direction.

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
        double target = angleDeg;

        // Roll-over logic: if target is outside limits, try geometric equivalents
        // (±360)
        if (target > TURRET_LIMIT_RIGHT) {
            if (target - 360 >= TURRET_LIMIT_LEFT) {
                target -= 360;
            } else {
                target = TURRET_LIMIT_RIGHT; // Hard clamp if roll-over not possible
            }
        } else if (target < TURRET_LIMIT_LEFT) {
            if (target + 360 <= TURRET_LIMIT_RIGHT) {
                target += 360;
            } else {
                target = TURRET_LIMIT_LEFT; // Hard clamp
            }
        }

        this.turretTargetAngleDeg = target;
    }

    public double getTurretAngle() {
        return getMeasuredTurretAngle(); // Now returns the actual real-world angle!
    }

    public double getVelocity() {
        return shootMotor.getVelocity();
    }

    public double getPower() {
        return shootMotor.getPower();
    }

    public void setFlywheelPIDF(double p, double f) {
        this.flywheelP = p;
        this.flywheelF = f;
    }

    private void updatePIDF() {
        if (pidfUpdateTimer.milliseconds() < PIDF_UPDATE_INTERVAL_MS)
            return;

        // Detection logic updated to include D
        if (Math.abs(targetVelocity - lastAppliedTarget) > 20 ||
                flywheelP != lastAppliedP ||
                flywheelD != lastAppliedD || // Check for D changes
                flywheelF != lastAppliedF_base) {

            com.qualcomm.robotcore.hardware.PIDFCoefficients coefficients = new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                    flywheelP, 0, flywheelD, flywheelF);

            shootMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
            shootMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

            lastAppliedTarget = targetVelocity;
            lastAppliedP = flywheelP;
            lastAppliedD = flywheelD;
            lastAppliedF_base = flywheelF;
            pidfUpdateTimer.reset();
        }
    }

    public void displayTelemetry(Telemetry telemetry) {
        if (shootMotorEnabled) {
            telemetry.addLine("<h2><font color='#ff0000'>Shoot Motor Enabled</font></h2>");
        }
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