package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: OPModeTabara", group = "Linear OpMode")
public class OPModeTabara extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    @Override
    public void init() {
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the runtime when the play button is pressed
        runtime.reset();
    }

    double maxMotorSpeed = 0.0;
    double rearLeftPower = 0.0;
    double rearRightPower = 0.0;
    double driveMotor = 0.0;
    double strafeMotor = 0.0;
    double turnMotor = 0.0;

    public void loop() {
        driveMotor = -gamepad1.left_stick_y;
        strafeMotor = gamepad1.left_stick_x;
        turnMotor = gamepad1.right_stick_x;

        // Calculate individual motor powers (adjust signs as needed)
        rearLeftPower = (driveMotor - strafeMotor + turnMotor);
        rearRightPower = (driveMotor + strafeMotor - turnMotor);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        maxMotorSpeed = Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower));

        if (maxMotorSpeed > 1.0) {
            rearLeftPower /= maxMotorSpeed;
            rearRightPower /= maxMotorSpeed;
        }
        leftRearMotor.setPower(rearLeftPower);
        rightRearMotor.setPower(rearRightPower);
    }
}