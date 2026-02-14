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

    public static double DEFAULT_SHOOT_POWER = 1;
    private boolean shootMotorEnabled = false;
    private double targetBasePower = 0;
    public static double INITIAL_ANGLE = 0.9;

    // Maybe used at a later time
    public static double TICKS_PER_DEGREE = 7.78;
    public static int MAX_TURRET_ANGLE_DEG = 90;
    public static double HUSKYLENS_FOV_DEG = 60.0;
    private int turretTargetPos = 0;
    private boolean autoAimEnabled = false;
    public static double TURRET_TRACKING_POWER = 0.6;
    public static double TURRET_RESET_POWER = 0.5;
    public static int TARGET_TAG_ID = 1;
    public static double AUTO_SHOOT_POWER = 0.75;
    //

    // Power Distance Scaling for shoot motor
    private final double MIN_SHOOT_POWER = 0.55;
    private final double MAX_SHOOT_POWER = 1.0;
    private final double POWER_DISTANCE_SCALING = 0.0012; // Adjust this to tune how hard it shoots
    public static double MAX_VELOCITY = 2300; // Ticks per second at 1.0 power. TUNE THIS!
    private final double VOLTAGE = 13.4; // Fresh battery
    private double filteredVoltage = 13.0; // Start at a healthy middle ground
    private final double LPF_COEFFICIENT = 0.95; // 0.95 means it keeps 95% of old value, 5% of new
    private VoltageSensor voltageSensor;

    // Angle Distance Scaling for outtake angle
    private final double CLOSE_DIST = 30.0;
    private final double FAR_DIST = 65.0;
    private final double CLOSE_ANGLE = 0.15;
    private final double FAR_ANGLE = 0.9;

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
        filteredVoltage = voltageSensor.getVoltage();

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
        if (!shootMotorEnabled) {
            shootMotor.setPower(0);
            return;
        }

        // This "Smooths" the voltage readings
        // It ignores sudden spikes from the drivetrain but tracks the battery's real
        // state
        double instantVoltage = voltageSensor.getVoltage();
        filteredVoltage = (LPF_COEFFICIENT * filteredVoltage) + ((1 - LPF_COEFFICIENT) * instantVoltage);

        // DIRECT POWER (Ramping removed)
        shootMotor.setPower(targetBasePower);
    }

    public void updateAutoAimPower(double deltaX, double deltaY) {
        // 1. CALCULATE DYNAMIC POWER
        double distance = Math.hypot(deltaX, deltaY);

        // Linear formula: Power = MinPower + (Dist * Scale)
        double basePower = MIN_SHOOT_POWER + (distance * POWER_DISTANCE_SCALING);

        // Apply Voltage Compensation
        double voltageComp = VOLTAGE / filteredVoltage;

        targetBasePower = basePower * voltageComp;

        // 2. CLAMP AND APPLY
        targetBasePower = Math.max(MIN_SHOOT_POWER, Math.min(targetBasePower, MAX_SHOOT_POWER));
    }

    public void updateStaticPower() {
        // Apply Voltage Compensation to the static AUTO_SHOOT_POWER
        double voltageComp = VOLTAGE / filteredVoltage;

        targetBasePower = AUTO_SHOOT_POWER * voltageComp;

        // Clamp to [MIN, MAX]
        targetBasePower = Math.max(MIN_SHOOT_POWER, Math.min(targetBasePower, MAX_SHOOT_POWER));
    }

    public void updateAutoAimAngle(double deltaX, double deltaY) {
        double distance = Math.hypot(deltaX, deltaY);

        // Map distance to angle: 0.5 (close) to 0.9 (far)
        double angle = CLOSE_ANGLE + (distance - CLOSE_DIST) * (FAR_ANGLE - CLOSE_ANGLE) / (FAR_DIST - CLOSE_DIST);

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

    public boolean isReadyToFire() {
        // Ready if enabled and velocity is within 5% of our expected target velocity
        double targetVelocity = targetBasePower * MAX_VELOCITY;
        return shootMotorEnabled && (getVelocity() >= targetVelocity * 0.95);
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
                double errorPixels = block.x - 160;
                double errorDegrees = errorPixels * (HUSKYLENS_FOV_DEG / 320.0);
                double currentAngleDeg = outtakeMotor.getCurrentPosition() / TICKS_PER_DEGREE;
                double targetAngleDeg = currentAngleDeg + errorDegrees;

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

    public double getVelocity() {
        return shootMotor.getVelocity();
    }

    public void displayTelemetry(Telemetry telemetry) {
        if (isReadyToFire()) {
            telemetry.addLine("<b><font color='lime'><h1>*** READY TO FIRE ***</h1></font></b>");
        } else if (shootMotorEnabled) {
            telemetry.addLine("<i>Spinning Up...</i>");
        } else {
            telemetry.addLine("<i>Stopped</i>");
        }
        telemetry.addData("Outtake Pos", getOuttakeMotorPosition());
        telemetry.addData("Target Power", "%.2f", getTargetBasePower());
        telemetry.addData("Target Velocity", "%.0f", getTargetBasePower() * MAX_VELOCITY);
        telemetry.addData("ShootMotor Velocity", getVelocity());
        telemetry.addData("Turret Target", getTurretTargetPos());
    }
}