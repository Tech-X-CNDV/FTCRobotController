package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class LearningOpMode extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        double tgtPower = 0;
        waitForStart();

        while(opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            rightMotor.setPower(tgtPower);
            leftMotor.setPower(tgtPower);

            telemetry.addData("target power", tgtPower);
            telemetry.addData("left motor power", leftMotor.getPower());
            telemetry.addData("right motor power", rightMotor.getPower());
            telemetry.update();
        }
    }
}
