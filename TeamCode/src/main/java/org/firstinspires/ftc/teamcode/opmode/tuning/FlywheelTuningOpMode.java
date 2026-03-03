package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;

@TeleOp(name = "Flywheel Tuning PIDF", group = "Tuning")
public class FlywheelTuningOpMode extends LinearOpMode {
    private OuttakeSubsystem outtake;
    private double[] stepSizes = { 10.0, 1.0, 0.1, 0.01, 0.001 };
    private int stepIndex = 1; // Default to 1.0

    private boolean lastY = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new OuttakeSubsystem(hardwareMap);
        outtake.InitOuttake();

        telemetry.addLine("Flywheel Tuning OpMode Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Edge detection for targetVelocity toggle
            if (gamepad1.y && !lastY) {
                outtake.targetVelocity = (outtake.targetVelocity == 1500) ? 900 : 1500;
            }
            lastY = gamepad1.y;

            // Edge detection for step size cycle
            if (gamepad1.b && !lastB) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            lastB = gamepad1.b;

            double currentStep = stepSizes[stepIndex];

            // P adjustment
            if (gamepad1.dpad_up) {
                outtake.flywheelP += currentStep;
            } else if (gamepad1.dpad_down) {
                outtake.flywheelP -= currentStep;
            }

            // F adjustment
            if (gamepad1.dpad_right) {
                outtake.flywheelF += currentStep;
            } else if (gamepad1.dpad_left) {
                outtake.flywheelF -= currentStep;
            }

            // Apply coefficients every loop
            outtake.setFlywheelPIDF(outtake.flywheelP, outtake.flywheelF);

            // Update motor power/velocity
            outtake.update();

            // Telemetry
            telemetry.addData("Target Velocity", outtake.targetVelocity);
            telemetry.addData("Current Velocity", outtake.getVelocity());
            telemetry.addData("Error", outtake.targetVelocity - outtake.getVelocity());
            telemetry.addData("P", outtake.flywheelP);
            telemetry.addData("F", outtake.flywheelF);
            telemetry.addData("Step Size", currentStep);
            telemetry.update();
        }
    }
}
