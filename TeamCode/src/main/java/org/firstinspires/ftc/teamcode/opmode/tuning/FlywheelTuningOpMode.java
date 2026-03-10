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
    private boolean lastX = false;
    private boolean lastB = false;
    private double lastHighVelocity = 3200;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new OuttakeSubsystem(hardwareMap);
        outtake.InitOuttake();

        telemetry.addLine("Flywheel Tuning OpMode Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // High Speed Toggle (3200 / 2000)
            if (gamepad1.y && !lastY) {
                outtake.targetVelocity = (outtake.targetVelocity == 2200) ? 1500 : 2200;
            }
            lastY = gamepad1.y;

            if (gamepad1.aWasPressed())
                outtake.ToggleShootMotor();

            // Low Speed Toggle (1000)
            if (gamepad1.x && !lastX) {
                if (outtake.targetVelocity != 100) {
                    lastHighVelocity = outtake.targetVelocity;
                    outtake.targetVelocity = 100;
                } else {
                    outtake.targetVelocity = lastHighVelocity;
                }
            }
            lastX = gamepad1.x;

            // Edge detection for step size cycle
            if (gamepad1.b && !lastB) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            lastB = gamepad1.b;

            double currentStep = stepSizes[stepIndex];

            // P adjustment (Dpad Up/Down)
            if (gamepad1.dpadUpWasPressed())
                outtake.flywheelP += currentStep;
            else if (gamepad1.dpadDownWasPressed())
                outtake.flywheelP -= currentStep;

            // Velocity F (kV) adjustment (Dpad Right/Left)
            if (gamepad1.dpadRightWasPressed())
                outtake.flywheelF += currentStep;
            else if (gamepad1.dpadLeftWasPressed())
                outtake.flywheelF -= currentStep;

            // Static F (kS) adjustment (Bumpers)
            if (gamepad1.rightBumperWasPressed())
                outtake.flywheelStaticF += currentStep;
            else if (gamepad1.leftBumperWasPressed())
                outtake.flywheelStaticF -= currentStep;

            // Apply coefficients every loop
            outtake.setFlywheelPIDF(outtake.flywheelP, outtake.flywheelF, outtake.flywheelStaticF);

            // Update motor power/velocity
            outtake.update();

            // Telemetry
            telemetry.addData("Target Velocity", outtake.targetVelocity);
            telemetry.addData("Current Velocity", outtake.getVelocity());
            telemetry.addData("Error", outtake.targetVelocity - outtake.getVelocity());
            telemetry.addLine("--- PIDF Coefficients ---");
            telemetry.addData("P (Dpad Up/Down)", "%.4f", outtake.flywheelP);
            telemetry.addData("Velocity F (Dpad R/L)", "%.6f", outtake.flywheelF);
            telemetry.addData("Static F (Bumpers)", "%.4f", outtake.flywheelStaticF);
            telemetry.addData("Step Size", currentStep);
            telemetry.update();
        }
    }
}
