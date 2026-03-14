package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsystem;

@TeleOp(name = "Intake Fast Tuning", group = "Tuning")
public class IntakeTuningOpMode extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() {
        // Initialize the Intake Subsystem
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeSubsystem.InitIntake();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Trigger: Intake Power (0.0 to 1.0)");
        telemetry.addLine("Left Bumper (Hold): Reverse Intake Direction");
        telemetry.addLine("A Button: Intake at 100% Power");
        telemetry.addLine("B Button: Outtake at 100% Power");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double power = 0;

            // Give priority to specific speed buttons (A and B)
            if (gamepad1.a) {
                power = 1.0;
            } else if (gamepad1.b) {
                power = -1.0;
            } else {
                // Otherwise read the trigger for variable speed
                power = gamepad1.right_trigger;

                // If left bumper is held down, we reverse the trigger power
                if (gamepad1.left_bumper) {
                    power = -power;
                }
            }

            // Apply the commanded power
            intakeSubsystem.setPower(power);

            // Display Telemetry
            telemetry.addLine("=== Fast Intake Controls ===");
            telemetry.addLine("RT: Variable | LB + RT: Variable Reverse");
            telemetry.addLine("A: Max Intake | B: Max Outtake");
            telemetry.addLine("");
            telemetry.addLine("=== Subsystem State ===");
            telemetry.addData("Power Sent", "%.2f", power);
            intakeSubsystem.displayTelemetry(telemetry);
            telemetry.update();
        }
    }
}
