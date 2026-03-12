package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Gate Servo Tuning", group = "Tuning")
public class GateServoTuningOpMode extends LinearOpMode {

    // Declare the servo object
    private Servo myServo;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        // Ensure the name "servoTest" matches your configuration on the Driver Station
        myServo = hardwareMap.get(Servo.class, "GateServo");
        myServo.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                // Move to position 1.0 when 'A' is pressed
                myServo.setPosition(0.23);
            } else {
                // Return to position 0.0 when 'A' is released
                myServo.setPosition(0.04);
            }

            // Send telemetry data to the Driver Station
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}