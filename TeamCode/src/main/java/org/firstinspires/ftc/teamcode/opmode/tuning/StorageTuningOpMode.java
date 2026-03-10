package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;

@TeleOp(name = "Storage Tuning", group = "Tuning")
public class StorageTuningOpMode extends LinearOpMode {

    private StorageSubsystem storageSubsystem;

    @Override
    public void runOpMode() {
        // Initialize the Storage Subsystem
        storageSubsystem = new StorageSubsystem(hardwareMap);
        storageSubsystem.InitStorage();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Reset To Intake (Uses Magnetic Sensor)");
        telemetry.addLine("B: Start Sorting");
        telemetry.addLine("X: Start Shooting");
        telemetry.addLine("Y: Abort / Stop");
        telemetry.addLine("Right Bumper: Open Gate");
        telemetry.addLine("Left Bumper: Close Gate");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Variables to handle toggle/presses
        boolean lastA = false;
        boolean lastX = false;
        boolean lastY = false;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Call update to handle the magnetic sensor resetting logic continuously
            storageSubsystem.update();

            // Handle Gamepad Input
            if (gamepad1.a && !lastA) {
                storageSubsystem.ResetToIntake();
            }
            lastA = gamepad1.a;

            if (gamepad1.x && !lastX) {
                storageSubsystem.StartShooting();
            }
            lastX = gamepad1.x;

            if (gamepad1.y && !lastY) {
                storageSubsystem.Abort();
            }
            lastY = gamepad1.y;

            if (gamepad1.dpadUpWasPressed()) {
                storageSubsystem.OpenGate();
            }

            if (gamepad1.dpadDownWasPressed()) {
                storageSubsystem.CloseGate();
            }

            // Display Telemetry
            telemetry.addLine("=== Commands ===");
            telemetry.addLine("A: Reset To Intake");
            telemetry.addLine("X: Start Shooting");
            telemetry.addLine("Y: Abort");
            telemetry.addLine("RB: Open Gate");
            telemetry.addLine("LB: Close Gate");
            telemetry.addLine("");

            telemetry.addLine("=== Subsystem State ===");
            storageSubsystem.displayTelemetry(telemetry);
            telemetry.update();
        }
    }
}
