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

    // Timing variables
    private double lastTarget = -1;
    private boolean lastEnabled = false;
    private com.qualcomm.robotcore.util.ElapsedTime spinUpTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private double spinUpTimeSeconds = -1;
    private boolean targetReached = true;

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
                outtake.targetVelocity = (outtake.targetVelocity == 2200) ? 1700 : 2200;
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

            // Apply coefficients every loop
            outtake.setFlywheelPIDF(outtake.flywheelP, outtake.flywheelF);

            // Spin-up timing logic
            boolean currentlyEnabled = outtake.getTargetVelocity() > 0; // Outtake sets enabled based on velocity in
                                                                        // some methods
            // Note: ToggleShootMotor changes internal state, but getTargetVelocity() might
            // be > 0.
            // Let's use a more direct way if possible, but let's stick to target and ready
            // checks.

            if (outtake.targetVelocity != lastTarget || currentlyEnabled != lastEnabled) {
                if (currentlyEnabled && outtake.targetVelocity > 0) {
                    spinUpTimer.reset();
                    targetReached = false;
                    spinUpTimeSeconds = -1;
                }
                lastTarget = outtake.targetVelocity;
                lastEnabled = currentlyEnabled;
            }

            if (!targetReached && Math.abs(outtake.getVelocity() - outtake.targetVelocity) <= 100) {
                spinUpTimeSeconds = spinUpTimer.seconds();
                targetReached = true;
            }

            // Update motor power/velocity
            outtake.update();

            // Telemetry
            telemetry.addData("Target Velocity", outtake.targetVelocity);
            telemetry.addData("Current Velocity", outtake.getVelocity());
            telemetry.addData("Error", outtake.targetVelocity - outtake.getVelocity());
            telemetry.addData("Motor Power", outtake.getPower());
            telemetry.addData("Spin-up Time",
                    spinUpTimeSeconds >= 0 ? String.format("%.2f s", spinUpTimeSeconds) : "Cranking...");
            telemetry.addLine("--- PIDF Coefficients ---");
            telemetry.addData("P (Dpad Up/Down)", "%.4f", outtake.flywheelP);
            telemetry.addData("Velocity F (Dpad R/L)", "%.6f", outtake.flywheelF);
            telemetry.addData("Step Size", currentStep);
            telemetry.update();
        }
    }
}
