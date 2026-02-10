package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSubsystem {
    private final HuskyLens hLens;
    private final DcMotorEx outtakeMotor, shootMotor;
    private final Servo outtakeAngle;
    private final VoltageSensor voltageSensor;

    public static double NOMINAL_VOLTAGE = 13.2;
    public static double TICKS_PER_DEGREE = 7.78; // Approximate, adjust as needed
    public static int MAX_TURRET_ANGLE_DEG = 90;
    public static double HUSKYLENS_FOV_DEG = 60.0;
    private int turretTargetPos = 0;
    private double targetBasePower = 0;
    private boolean autoAimEnabled = false;
    private boolean shootMotorEnabled = false;
    private int lastTagWidth = 0;
    private double lastTagError = 0;
    private double compensatedPower = 0;

    // TODO: Tune these mapping constants based on physical testing
    public static double DIST_POWER_SLOPE = -0.005;
    public static double DIST_POWER_OFFSET = 1.2;
    public static double DEFAULT_SHOOT_POWER = 0.75;
    public static double TURRET_TRACKING_POWER = 0.6;
    public static double TURRET_RESET_POWER = 0.5;
    public static double INITIAL_ANGLE = 0.9;
    public static int TARGET_TAG_ID = 1; // Default tag to aim at

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        shootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    private boolean huskyLensInitialized = false;

    public void InitOuttake() {
        outtakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setTargetPosition(0);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotor.setPower(0);

        targetBasePower = DEFAULT_SHOOT_POWER;

        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shootMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void InitVision() {
        if (huskyLensInitialized)
            return;
        hLens.initialize();
        hLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLensInitialized = true;
    }

    public void OuttakeMotorControl(double power) {
        if (outtakeMotor.getCurrentPosition() > -500 && power < 0)
            outtakeMotor.setPower(power);
        else if (outtakeMotor.getCurrentPosition() < 700 && power > 0)
            outtakeMotor.setPower(power);
        else
            outtakeMotor.setPower(0);
    }

    public void update() {
        if (autoAimEnabled) {
            updateAutoAimPower();
        }

        double currentVoltage = voltageSensor.getVoltage();
        if (currentVoltage < 1.0)
            currentVoltage = NOMINAL_VOLTAGE;

        compensatedPower = targetBasePower * (NOMINAL_VOLTAGE / currentVoltage);
        shootMotor.setPower(shootMotorEnabled ? compensatedPower : 0);
    }

    private void updateAutoAimPower() {
        HuskyLens.Block[] blocks = hLens.blocks();
        lastTagWidth = 0; // Reset if no tag is found in this update cycle
        for (HuskyLens.Block block : blocks) {
            if (block.id == TARGET_TAG_ID) {
                // Using width as a simple proxy for distance
                // Larger width = closer = lower power
                // Smaller width = further = higher power
                targetBasePower = (block.width * DIST_POWER_SLOPE) + DIST_POWER_OFFSET;
                lastTagWidth = block.width;

                // Clamp power between reasonable safe limits
                if (targetBasePower > 1.0)
                    targetBasePower = 1.0;
                if (targetBasePower < 0.4)
                    targetBasePower = 0.4;
                return;
            }
        }
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

    public void IncreaseAngle() {
        if (outtakeAngle.getPosition() < 1)
            outtakeAngle.setPosition(outtakeAngle.getPosition() + 0.1);
    }

    public void DecreaseAngle() {
        if (outtakeAngle.getPosition() > 0)
            outtakeAngle.setPosition(outtakeAngle.getPosition() - 0.1);
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

    public double getTagCenterError(int targetTagId) {
        InitVision();
        HuskyLens.Block[] blocks = hLens.blocks();
        for (HuskyLens.Block block : blocks) {
            if (block.id == targetTagId) {
                // Error in pixels from center (320x240 resolution)
                double errorPixels = block.x - 160;
                // Convert pixel error to angular error (approximate)
                lastTagError = errorPixels * (HUSKYLENS_FOV_DEG / 320.0);
                return lastTagError;
            }
        }
        lastTagError = 0;
        return 0; // Not found or no error
    }

    public void updateTurretLock(int targetTagId) {
        InitVision();
        HuskyLens.Block[] blocks = hLens.blocks();
        boolean found = false;

        for (HuskyLens.Block block : blocks) {
            if (block.id == targetTagId) {
                // Error in pixels from center (320x240 resolution)
                double errorPixels = block.x - 160;
                // Convert pixel error to angular error (approximate)
                double errorDegrees = errorPixels * (HUSKYLENS_FOV_DEG / 320.0);

                // Calculate new target relative to current angle
                // Note: This logic assumes the camera moves WITH the turret.
                // If the camera is static, the logic would be different.
                // Assuming Camera is ON the turret:
                double currentAngleDeg = outtakeMotor.getCurrentPosition() / TICKS_PER_DEGREE;
                double targetAngleDeg = currentAngleDeg + errorDegrees;

                // Clamp to limits
                if (targetAngleDeg > MAX_TURRET_ANGLE_DEG)
                    targetAngleDeg = MAX_TURRET_ANGLE_DEG;
                if (targetAngleDeg < -MAX_TURRET_ANGLE_DEG)
                    targetAngleDeg = -MAX_TURRET_ANGLE_DEG;

                turretTargetPos = (int) (targetAngleDeg * TICKS_PER_DEGREE);
                found = true;
                break;
            }
        }

        outtakeMotor.setTargetPosition(found ? turretTargetPos : outtakeMotor.getTargetPosition());
        outtakeMotor.setPower(TURRET_TRACKING_POWER);
    }

    public void resetTurret() {
        turretTargetPos = 0;
        outtakeMotor.setTargetPosition(0);
        outtakeMotor.setPower(TURRET_RESET_POWER);
    }

    public int getOuttakeMotorPosition() {
        return outtakeMotor.getCurrentPosition();
    }

    public int getTurretTargetPos() {
        return turretTargetPos;
    }

    public int getLastTagWidth() {
        return lastTagWidth;
    }

    public double getLastTagError() {
        return lastTagError;
    }

    public double getCompensatedPower() {
        return compensatedPower;
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("  Outtake Pos", getOuttakeMotorPosition());
        // Note: For Turret Lock and Auto Aim status, we'll keep the logic in OPMode for
        // now
        // as they use local boolean flags (turretLockEnabled, chassisLockEnabled).
        // However, we can show the core subsystem data here.
        telemetry.addData("Target Base Power", "%.2f", getTargetBasePower());
        telemetry.addData("Compensated Power", "%.2f", getCompensatedPower());
        telemetry.addData("Tag Width", getLastTagWidth());
        telemetry.addData("Tag Error", "%.2f", getLastTagError());
        telemetry.addData("Turret Target", getTurretTargetPos());
    }
}
