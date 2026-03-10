package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Raw Tuning", group = "Tuning")
public class TurretRawTuningOpMode extends LinearOpMode {

    // Rev Through Bore Encoder: 8192 ticks per revolution
    private static final double TICKS_PER_REV = 8192.0;

    private CRServo turretServo1;
    private CRServo turretServo2;
    private DcMotorEx turretEncoder; // Plugged into the PrindMotor encoder port
    public static double TURRET_GEAR_RATIO = 7.46; // 1.0 = encoder is directly on turret shaft. TUNE THIS!

    @Override
    public void runOpMode() {
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
        // Encoder plugged into the intake motor (PrindMotor) encoder port
        turretEncoder = hardwareMap.get(DcMotorEx.class, "PrindMotor");

        // Direction: one reversed so both push the turret the same way
        turretServo1.setDirection(CRServo.Direction.FORWARD);
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        turretServo1.setPower(0);
        turretServo2.setPower(0);

        // Zero the encoder right here so wherever the turret is placed physically = 0
        // degrees
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.addLine("");
        telemetry.addLine("Encoder zeroed to current position.");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick X: Drive turret (-1.0 to 1.0 power)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joystick with small deadband
            double power = gamepad1.left_stick_x;
            if (Math.abs(power) < 0.05) {
                power = 0;
            }

            // Apply power to both servos
            turretServo1.setPower(power);
            turretServo2.setPower(power);

            // Calculate angle from ticks
            int ticks = -turretEncoder.getCurrentPosition();
            double rawAngle = (ticks / (8192.0 * TURRET_GEAR_RATIO)) * 360.0;

            // Telemetry
            telemetry.addLine("=== RAW TURRET CONTROL ===");
            telemetry.addData("Gamepad Power", "%.2f", power);
            telemetry.addLine("");
            telemetry.addData("Encoder Ticks", ticks);
            telemetry.addData("Turret Angle (degs)", "%.1f", rawAngle);
            telemetry.update();
        }
    }
}
