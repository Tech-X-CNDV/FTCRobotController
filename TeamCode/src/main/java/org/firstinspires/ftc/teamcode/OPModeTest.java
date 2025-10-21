package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OPModeTest extends OpMode{

    @Override
    public void init(){
        telemetry.addData("Aprins", "sal");
        telemetry.update();
    }

    @Override
    public void loop(){

    }
}
