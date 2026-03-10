package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;

@TeleOp(name = "Tuning: Find Max Velocity", group = "Tuning")
public class MaxVelocityTuningOpMode extends LinearOpMode {
    private OuttakeSubsystem outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new OuttakeSubsystem(hardwareMap);

        // We initialize but we need to override the mode to RUN_WITHOUT_ENCODER
        // to see the raw max speed at 1.0 power.
        outtake.InitOuttake();

        // Accessing motors directly just for this raw test
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "ShootMotor2");

        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addLine("Max Velocity Tuning Initialized");
        telemetry.addLine("This will run motors at 1.0 POWER!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor1.setPower(1.0);
            motor2.setPower(1.0);

            telemetry.addData("Power", 1.0);
            telemetry.addData("Motor 1 Velocity", motor1.getVelocity());
            telemetry.addData("Motor 2 Velocity", motor2.getVelocity());
            telemetry.addData("Average Velocity", (motor1.getVelocity() + motor2.getVelocity()) / 2.0);
            telemetry.addLine("\nTake the Average Velocity value and put it in MAX_VELOCITY!");
            telemetry.update();
        }

        motor1.setPower(0);
        motor2.setPower(0);
    }
}
