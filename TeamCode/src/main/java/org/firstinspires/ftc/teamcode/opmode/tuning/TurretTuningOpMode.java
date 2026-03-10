package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;

@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuningOpMode extends LinearOpMode {

    private OuttakeSubsystem outtakeSubsystem;

    @Override
    public void runOpMode() {
        // Initialize the Outtake Subsystem (contains the Turret)
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();

        // Ensure shoot motors won't run during turret tuning
        outtakeSubsystem.SetShootMotorPower(0);

        outtakeSubsystem.SetAngle(0);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.addLine("");
        telemetry.addLine("=== CALIBRATION INSTRUCTIONS ===");
        telemetry.addLine("1. With robot off/init, physically point the turret straight forward.");
        telemetry.addLine("2. Read 'TURRET RAW ANGLE' in the telemetry.");
        telemetry.addLine("3. Put that exact number into 'TURRET_ENCODER_OFFSET_DEG' in Dashboard.");
        telemetry.addLine("4. The 'Turret Real Angle' should now read 0.0.");
        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("D-Pad Up: Go to 0 Degrees");
        telemetry.addLine("D-Pad Right: Go to -45 Degrees");
        telemetry.addLine("D-Pad Left: Go to +45 Degrees");
        telemetry.addLine("Right Stick X: Manual Analog Setpoint Adjust");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        double currentTarget = outtakeSubsystem.getTurretAngle(); // Start where we are

        // Run until the end of the match
        while (opModeIsActive()) {

            // D-Pad Snapping
            if (gamepad1.dpad_up) {
                currentTarget = 0;
            } else if (gamepad1.dpad_right) {
                currentTarget = 45;
            } else if (gamepad1.dpad_left) {
                currentTarget = -45;
            }

            if (gamepad1.rightBumperWasPressed())
                outtakeSubsystem.SetAngle(1);

            if (gamepad1.leftBumperWasPressed())
                outtakeSubsystem.SetAngle(0);

            // Analog adjusting (slow mapping of stick to degrees)
            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                currentTarget -= -gamepad1.right_stick_x * 0.5; // Decreases currentTarget to move right/left
            }

            // Command the angle
            outtakeSubsystem.setTurretTargetAngle(currentTarget);

            // Let the PID update loop run
            outtakeSubsystem.update();

            // Display Telemetry
            telemetry.addLine("=== TURRET TUNING CONTROLS ===");
            telemetry.addLine("DPad Up: 0 | Left: 45 | Right: -45");
            telemetry.addLine("Right Stick X: Manual Adjust Target");
            telemetry.addLine("");
            telemetry.addLine("=== SUBSYSTEM STATE (Use FTC Dashboard to tune) ===");
            telemetry.addData("1. Target Angle", "%.2f", currentTarget);
            telemetry.addData("2. Real Angle (Measured)", "%.2f", outtakeSubsystem.getTurretAngle());
            telemetry.addData("3. Error", "%.2f", currentTarget - outtakeSubsystem.getTurretAngle());

            telemetry.addLine("");
            outtakeSubsystem.displayTelemetry(telemetry);
            telemetry.update();
        }
    }
}
