package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem {
    private final HuskyLens hLens;
    private final DcMotorEx outtakeMotor, shootMotor;
    private final Servo outtakeAngle;

    public static double TICKS_PER_DEGREE = 7.78; // Approximate, adjust as needed
    public static int MAX_TURRET_ANGLE_DEG = 90;
    public static double HUSKYLENS_FOV_DEG = 60.0;
    private int turretTargetPos = 0;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        shootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
    }

    private boolean huskyLensInitialized = false;

    public void InitOuttake() {
        outtakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setTargetPosition(0);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotor.setPower(0);

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

    public void ToggleShootMotor() {
        shootMotor.setPower(shootMotor.getPower() > 0 ? 0 : 0.75);
    }

    public void ToggleShootMotorAuto() {
        shootMotor.setPower(shootMotor.getPower() > 0 ? 0 : 1);
    }

    public void SetShootMotorPower(double power) {
        shootMotor.setPower(power);
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
        outtakeMotor.setPower(0.6); // Reasonable power for tracking
    }

    public void resetTurret() {
        turretTargetPos = 0;
        outtakeMotor.setTargetPosition(0);
        outtakeMotor.setPower(0.5);
    }

    public int getOuttakeMotorPosition() {
        return outtakeMotor.getCurrentPosition();
    }

    public int getTurretTargetPos() {
        return turretTargetPos;
    }
}
