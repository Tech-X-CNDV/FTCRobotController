package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OPModeTest extends OpMode{
    DcMotorEx storageMotor;
    DcMotorEx prindMotor;
    float[] _green = {77, 162, 99};
    float[] _purple = {140, 50, 178};
    ColorSensor sensor = null;
    float[] hsvValues = new float[3];
    ElapsedTime runTime = new ElapsedTime();
    boolean sorting = false;
    boolean autoSort = false;
    int pos = 0;
    char curChar;
    double timer = 0;
    double timerServo = -3;
    int turns = 0;
    int thrown = 0;
    String[] patterns = {" ", "GPP ", "PGP ", "PPG ", " "};
    int curPatt;
    Servo servoArunc;
    boolean paused = false;
    boolean move = false;
    HuskyLens hLens;
    int lastId = 1;

    char idenColor(){
        float hue = hsvValues[0];
        float sat = hsvValues[1];
        if(sat < 0.35)
            return 'P';
        else if(hue > 120 && hue < 150)
            return 'G';
        else
            return 'N';
    }

    void PatternSortTeleop(String pattern){
        if(pos > 2) {
            sorting = false;
            pos = 0;
            turns = 0;
        }
        boolean found = false;
        char[] patt = pattern.toCharArray();
        curChar = patt[pos];
        telemetry.addData("CurrentChar", curChar);
        found = false;
        if(runTime.seconds() - timer > 1) {
            if (idenColor() == curChar) {
                turns = 0;
                pos++;
                sorting = false;
                // Funct de arunc
            }
            if(sorting) {
                MoveToPosition(storageMotor, 2550, 0.5);
                turns++;
                if (turns > 3)
                    sorting = false;
                timer = runTime.seconds();
            }
        }
    }

    void PatternSortAuto(String pattern){
        if(pos > 2 && !paused) {
            autoSort = false;
            pos = 0;
            turns = 0;
            servoArunc.setPosition(1);
        }
        char[] patt = pattern.toCharArray();
        curChar = patt[pos];
        telemetry.addData("CurrentChar", curChar);
        if(runTime.seconds() - timer > 1) {
            if(autoSort && idenColor() != curChar && !paused && !move) {
                servoArunc.setPosition(1);
                storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                move = true;
                turns++;
                if (turns > 3)
                    autoSort = false;
                timer = runTime.seconds();
            }
            if (idenColor() == curChar && !paused && !move) {
                turns = 0;
                pos++;
                servoArunc.setPosition(0.6);
                thrown++;
                timerServo = runTime.seconds();
                // Funct de arunc
            }
        }
    }

    void MoveToPosition(DcMotorEx motor, int target, double power){
        telemetry.addData("Target", target);
        if(storageMotor.getCurrentPosition() < target){
            motor.setPower(power);
        }else{
            motor.setPower(0);
            move = false;
        }
    }

    @Override
    public void init(){
        sensor = hardwareMap.get(ColorSensor.class, "ColorS");
        storageMotor = hardwareMap.get(DcMotorEx.class, "StorageMotor");
        storageMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storageMotor.setPower(0);
        storageMotor.setTargetPosition(0);
        storageMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        prindMotor = hardwareMap.get(DcMotorEx.class, "PrindMotor");
        prindMotor.setPower(0);
        prindMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servoArunc = hardwareMap.get(Servo.class, "ServoArunc");
        servoArunc.setPosition(1);
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
        hLens.initialize();
        hLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addData("Aprins", "sal");
        telemetry.update();
    }

    @Override
    public void start(){
        runTime.reset();
    }

    @Override
    public void loop(){
        HuskyLens.Block[] blocks = hLens.blocks();
        Color.RGBToHSV(sensor.red() * 8, sensor.green() * 8, sensor.blue() * 8, hsvValues);
        float hue = hsvValues[0];
        float sat = hsvValues[1];
        if(gamepad1.aWasPressed() && !storageMotor.isBusy())
            MoveToPosition(storageMotor,2500, 0.3);
        if(gamepad1.yWasPressed() && !sorting)
            sorting = true;
        if(gamepad1.bWasPressed() && !autoSort && !move) {
            storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pos = 0;
            turns = 0;
            autoSort = true;
        }
        prindMotor.setPower(gamepad1.right_trigger);
        if(gamepad1.dpadDownWasPressed()) {
            if(curPatt == 2)
                curPatt = 0;
            else
                curPatt++;
        }
        if(gamepad1.dpadUpWasPressed() && !storageMotor.isBusy() && !move) {
            storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            move = true;
        }
        if (gamepad1.dpadLeftWasPressed())
            servoArunc.setPosition(servoArunc.getPosition() == 0.7 ? 1 : 0.7);

        if(move)
            MoveToPosition(storageMotor, 2232, 0.6);
        paused = runTime.seconds() - timerServo < 1;

        if(sorting)
            PatternSortTeleop(patterns[curPatt]);
        if(autoSort)
            PatternSortAuto(patterns[lastId]);
//        if(!storageMotor.isBusy())
//            storageMotor.setPower(0);
        telemetry.addData("Hue: ", hue);
        telemetry.addData("Sat: ", sat);
        if(idenColor() == 'G')
            telemetry.addData("Artifact", "Green");
        else if(idenColor() == 'P')
            telemetry.addData("Artifact", "Purple");
        else
            telemetry.addData("Artifact", "None");
        telemetry.addData("runTime", runTime.seconds());
        telemetry.addData("Sorting", sorting);
        telemetry.addData("AutoSorting", autoSort);
        telemetry.addData("CharPos", pos);
        telemetry.addData("Turns", turns);
        telemetry.addData("Power", storageMotor.getPower());
        telemetry.addData("Busy", storageMotor.isBusy());
        telemetry.addData("CurrPatt", patterns[curPatt]);
        telemetry.addData("Paused", paused);
        telemetry.addData("Encoder", storageMotor.getCurrentPosition());
        telemetry.addData("Move", move);
        if(blocks.length > 0)
            lastId = blocks[0].id;
        telemetry.addData("Id", lastId + " " + patterns[lastId]);
    }
}
