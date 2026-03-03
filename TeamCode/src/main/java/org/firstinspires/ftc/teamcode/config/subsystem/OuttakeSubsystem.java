package org.firstinspires.ftc.teamcode.config.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class OuttakeSubsystem {
    private final HuskyLens hLens;
    private final DcMotorEx shootMotor, shootMotor2, turretMotor;
    private final Servo outtakeAngle;

    public double targetVelocity = 4800;
    public double flywheelP = 0;
    public double flywheelF = 0;

    public static double DEFAULT_SHOOT_POWER = 0.7;
    private boolean shootMotorEnabled = false;
    private double targetBasePower = 0;
    private double manualPowerOffset = 0;
    private double manualAngleOffset = 0;
    public static double INITIAL_ANGLE = 0.9;

    // Turret Constants
    public static double TICKS_PER_DEGREE = 7.78;
    public static int MAX_TURRET_ANGLE_DEG = 90;
    public static double HUSKYLENS_FOV_DEG = 60.0;
    private int turretTargetPos = 0;
    private boolean autoAimEnabled = false;
    public static double TURRET_TRACKING_POWER = 0.6;
    public static double TURRET_RESET_POWER = 0.5;
    public static int TARGET_TAG_ID = 1;
    public static double AUTO_SHOOT_POWER = 0.75;
    private boolean isHuskyLock = false;

    // Turret PID Constants (Tune these!)
    public static double turretP = 0.005;
    public static double turretI = 0.0001;
    public static double turretD = 0.0001;
    public static double turretF = 0.01;
    public static double turretD_LPF = 0.8; // 0.8 means keep 80% of old value, 20% of new

    private double turretIntegralSum = 0;
    private double lastTurretError = 0;
    private double lastFilteredDerivative = 0;
    private final ElapsedTime turretTimer = new ElapsedTime();

    // Power Distance Scaling for shoot motor
    private final double MIN_SHOOT_POWER = 0.47;
    private final double MAX_SHOOT_POWER = 1.0;
    private final double POWER_DISTANCE_SCALING = 0.0012; // Adjust this to tune how hard it shoots
    public static double MAX_VELOCITY = 5300; // Ticks per second at 1.0 power based on 6000RPM+ headroom. TUNE THIS!

    // Angle Distance Scaling for outtake angle
    private final double CLOSE_DIST = 30.0;
    private final double FAR_DIST = 65.0;
    private final double CLOSE_ANGLE = 0.15;
    private final double FAR_ANGLE = 0.9;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        shootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "ShootMotor2");
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
    }

    private boolean huskyLensInitialized = false;

    public void InitOuttake() {
        targetBasePower = DEFAULT_SHOOT_POWER;

        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shootMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shootMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        shootMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretTargetPos = 0;
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
            return;
        }

        // DIRECT VELOCITY
        shootMotor.setVelocity(targetVelocity);
        shootMotor2.setVelocity(targetVelocity);

        // Update Turret PID
        updateTurretPID();
    }

    public void updateAutoAimPower(double deltaX, double deltaY) {
        // 1. CALCULATE DYNAMIC VELOCITY
        double distance = Math.hypot(deltaX, deltaY);

        // Linear formula: PowerRatio = MinPower + (Dist * Scale) + ManualOffset
        double powerRatio = MIN_SHOOT_POWER + (distance * POWER_DISTANCE_SCALING) + manualPowerOffset;

        // Convert to Velocity Ticks/Sec (PIDF handles voltage compensation)
        targetVelocity = powerRatio * MAX_VELOCITY;

        // Clamp and update base power for telemetry
        targetBasePower = Math.max(MIN_SHOOT_POWER, Math.min(powerRatio, MAX_SHOOT_POWER));
        targetVelocity = Math.max(MIN_SHOOT_POWER * MAX_VELOCITY, Math.min(targetVelocity, MAX_VELOCITY));
    }

    public void updateStaticPower() {
        targetBasePower = Math.max(MIN_SHOOT_POWER, Math.min(AUTO_SHOOT_POWER + manualPowerOffset, MAX_SHOOT_POWER));
        targetVelocity = targetBasePower * MAX_VELOCITY;
    }

    public void updateFixedPower(double basePower) {
        targetBasePower = Math.max(MIN_SHOOT_POWER, Math.min(basePower + manualPowerOffset, MAX_SHOOT_POWER));
        targetVelocity = targetBasePower * MAX_VELOCITY;
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
        targetBasePower = power;
        shootMotorEnabled = (power > 0);
    }

    public double getTargetBasePower() {
        return targetBasePower;
    }

    public void setManualPowerOffset(double offset) {
        this.manualPowerOffset = offset;
    }

    public double getManualPowerOffset() {
        return manualPowerOffset;
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
            // Center is 160, FOV is ~60 degrees
            double visualErrorDeg = (targetBlock.x - 160) * (HUSKYLENS_FOV_DEG / 320.0);

            // RELATIVE UPDATE: Calculate target relative to current position to prevent
            // "jumps"
            int currentPos = turretMotor.getCurrentPosition();
            int errorTicks = (int) (visualErrorDeg * TICKS_PER_DEGREE);
            int manualOffsetTicks = (int) (manualOffset * 57.2958 * TICKS_PER_DEGREE); // Convert rad to deg then ticks

            int potentialTarget = currentPos + errorTicks + manualOffsetTicks;

            // Clamp to physical limits in degrees before setting
            double potentialTargetDeg = potentialTarget / TICKS_PER_DEGREE;
            if (potentialTargetDeg > MAX_TURRET_ANGLE_DEG)
                potentialTargetDeg = MAX_TURRET_ANGLE_DEG;
            if (potentialTargetDeg < -MAX_TURRET_ANGLE_DEG)
                potentialTargetDeg = -MAX_TURRET_ANGLE_DEG;

            turretTargetPos = (int) (potentialTargetDeg * TICKS_PER_DEGREE);
            isHuskyLock = true;
        } else {
            // Fallback to Pedro Pathing Pose Lock
            double deltaX = targetPose.getX() - robotPose.getX();
            double deltaY = targetPose.getY() - robotPose.getY();

            // Calculate absolute angle to target
            double angleToTarget = Math.atan2(deltaY, deltaX);

            // Calculate relative angle for turret (target - robot heading)
            double relativeAngle = angleToTarget - robotPose.getHeading() + manualOffset;

            // Normalize relative angle to [-PI, PI]
            while (relativeAngle > Math.PI)
                relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI)
                relativeAngle += 2 * Math.PI;

            double targetAngleDeg = Math.toDegrees(relativeAngle);

            // Clamp to physical limits
            if (targetAngleDeg > MAX_TURRET_ANGLE_DEG)
                targetAngleDeg = MAX_TURRET_ANGLE_DEG;
            if (targetAngleDeg < -MAX_TURRET_ANGLE_DEG)
                targetAngleDeg = -MAX_TURRET_ANGLE_DEG;

            turretTargetPos = (int) (targetAngleDeg * TICKS_PER_DEGREE);
            isHuskyLock = false;
        }
    }

    private void updateTurretPID() {
        double currentPos = turretMotor.getCurrentPosition();
        double error = turretTargetPos - currentPos;
        double dt = turretTimer.seconds();
        turretTimer.reset();

        if (dt > 0.1)
            dt = 0.01; // Prevent massive spikes on first loop or lag

        turretIntegralSum += error * dt;

        // Anti-windup: cap the integral sum
        if (Math.abs(turretIntegralSum) > 0.2 / turretI) {
            turretIntegralSum = Math.signum(turretIntegralSum) * (0.2 / turretI);
        }

        double rawDerivative = (error - lastTurretError) / dt;
        lastTurretError = error;

        // Apply Low-Pass Filter to Derivative to reduce noise from flywheel vibrations
        double filteredDerivative = (turretD_LPF * lastFilteredDerivative) + ((1 - turretD_LPF) * rawDerivative);
        lastFilteredDerivative = filteredDerivative;

        double power = (error * turretP) + (turretIntegralSum * turretI) + (filteredDerivative * turretD);

        // Add Feedforward based on target direction (simple static friction
        // compensation)
        if (Math.abs(error) > 2) {
            power += Math.signum(error) * turretF;
        }

        // Clamp power and apply
        power = Math.max(-1.0, Math.min(1.0, power));

        // NOTE: Hard-zeroing deadband removed to maintain gear tension on Lazy Susan
        /*
         * if (Math.abs(error) < 2 && Math.abs(derivative) < 1) {
         * power = 0;
         * turretIntegralSum = 0;
         * }
         */

        turretMotor.setPower(power);
    }

    public void resetTurret() {
        turretTargetPos = 0;
    }

    public int getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }

    public int getTurretTargetPos() {
        return turretTargetPos;
    }

    public double getVelocity() {
        return (shootMotor.getVelocity() + shootMotor2.getVelocity()) / 2;
    }

    public void setFlywheelPIDF(double p, double f) {
        com.qualcomm.robotcore.hardware.PIDFCoefficients coefficients = new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                p, 0, 0, f);
        shootMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        shootMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
    }

    public void displayTelemetry(Telemetry telemetry) {
        if (isReadyToFire()) {
            telemetry.addLine("<h1><font color='#00FF00'>*** READY TO FIRE ***</font></h1>");
        } else if (shootMotorEnabled) {
            telemetry.addLine("<i>Spinning Up...</i>");
        } else {
            telemetry.addLine("<i>Stopped</i>");
        }
        // telemetry.addData("Outtake Pos", getOuttakeMotorPosition());
        telemetry.addData("Target Power", "%.2f", getTargetBasePower());
        telemetry.addData("Target Velocity", "%.0f", getTargetBasePower() * MAX_VELOCITY);
        telemetry.addData("ShootMotor Velocity", getVelocity());
        telemetry.addData("Turret Pos", getTurretPosition());
        telemetry.addData("Turret Target", getTurretTargetPos());
        telemetry.addData("Turret Lock Mode", isHuskyLock ? "HUSKY (PRECISE)" : "POSE (FALLBACK)");
    }
}