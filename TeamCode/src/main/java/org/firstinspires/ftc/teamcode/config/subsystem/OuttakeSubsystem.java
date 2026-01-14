package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSubsystem {
    private final HuskyLens hLens;
    private final DcMotorEx outtakeMotor, shootMotor;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        shootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
    }

    public void InitOuttake() {
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setPower(0);

        shootMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hLens.initialize();
        hLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public void OuttakeMotorControl(double power) {
        if (outtakeMotor.getCurrentPosition() > -500 && power < 0)
            outtakeMotor.setPower(power);
        else if (outtakeMotor.getCurrentPosition() < 700 && power > 0)
            outtakeMotor.setPower(power);
        else
            outtakeMotor.setPower(0);
    }

    public void ToggleShootMotor() {
        shootMotor.setPower(shootMotor.getPower() == 1 ? 0 : 1);
    }

    public HuskyLens.Block[] GetCameraFeed() {
        return hLens.blocks();
    }

    public int getOuttakeMotorPosition() {
        return outtakeMotor.getCurrentPosition();
    }
}
