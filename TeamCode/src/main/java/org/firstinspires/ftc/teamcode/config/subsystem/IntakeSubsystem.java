package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    public final DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "PrindMotor");
    }

    public void InitIntake() {
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("  Intake Power", intakeMotor.getPower());
    }
}
