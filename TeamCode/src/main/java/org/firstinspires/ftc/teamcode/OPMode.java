package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class OPMode extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    DcMotorEx storageMotor;
    DcMotorEx prindMotor;
    DcMotorEx outtakeMotor;
    DcMotorEx shootMotor;
    ColorSensor sensor = null;
    float[] hsvValues = new float[3];
    ElapsedTime runTime = new ElapsedTime();
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
    boolean smallMove = false;
    HuskyLens hLens;
    int lastId = 1;
    int outtakeDir = 1;
    boolean autoThrow = false;

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

    void PatternSortAuto(String pattern){
        if(pos > 2 && !paused) {
            pos = 0;
            turns = 0;
            servoArunc.setPosition(1);
            autoSort = false;
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

    void ThrowAll(){
        if(turns > 3) {
            turns = 0;
            thrown = 0;
            servoArunc.setPosition(1);
            autoThrow = false;
        }
        if(!move)
            servoArunc.setPosition(0.6);
        if(runTime.seconds() - timer > 1) {
            if(autoThrow && !paused && !move) {
                servoArunc.setPosition(1);
                storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                move = true;
                turns++;
                if (turns > 3)
                    autoThrow = false;
                timer = runTime.seconds();
            }
            thrown++;
        }
    }

    void MoveToPosition(DcMotorEx motor, int target, double power){
        telemetry.addData("Target", target);
        if(target - storageMotor.getCurrentPosition() > 485){
            motor.setPower(power);
        }else{
            motor.setPower(0);
            move = false;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        sensor = hardwareMap.get(ColorSensor.class, "ColorS");
        storageMotor = hardwareMap.get(DcMotorEx.class, "StorageMotor");
        storageMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storageMotor.setPower(0);
        storageMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        prindMotor = hardwareMap.get(DcMotorEx.class, "PrindMotor");
        prindMotor.setPower(0);
        prindMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setTargetPosition(0);
        outtakeMotor.setPower(0);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoArunc = hardwareMap.get(Servo.class, "ServoArunc");
        servoArunc.setPosition(1);
        hLens = hardwareMap.get(HuskyLens.class, "hLens");
        hLens.initialize();
        hLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addData("Aprins", "sal");
        telemetry.update();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        HuskyLens.Block[] blocks = hLens.blocks();
        Color.RGBToHSV(sensor.red() * 8, sensor.green() * 8, sensor.blue() * 8, hsvValues);
        float hue = hsvValues[0];
        float sat = hsvValues[1];
        if(gamepad2.bWasPressed() && !autoSort && !move) {
            storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pos = 0;
            turns = 0;
            autoSort = true;
        }
        prindMotor.setPower(gamepad1.right_trigger);
        if(gamepad2.xWasPressed()) {
            shootMotor.setPower(1);
            autoThrow = true;
        }
        if(gamepad2.dpadUpWasPressed() && !storageMotor.isBusy() && !move) {
            storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servoArunc.setPosition(1);
            move = true;
        }
        if (gamepad2.dpadLeftWasPressed() && !move){
            storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            smallMove = true;
        }
        if(gamepad1.left_trigger > 0 && !storageMotor.isBusy() && !move) {
            storageMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            storageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            move = true;
        }
        if(outtakeMotor.getCurrentPosition() > -500 && gamepad2.right_stick_x < 0)
            outtakeMotor.setPower(gamepad2.right_stick_x);
        else if(outtakeMotor.getCurrentPosition() < 700 && gamepad2.right_stick_x > 0)
            outtakeMotor.setPower(gamepad2.right_stick_x);
        else
            outtakeMotor.setPower(0);
//        if(gamepad2.left_trigger > 0) {
//            outtakeMotor.setPower(outtakeDir * 0.3);
//            outtakeMotor.setTargetPosition(outtakeMotor.getCurrentPosition() + outtakeDir * 100);
//        }else {
//            outtakeMotor.setPower(0);
//            outtakeMotor.setTargetPosition(outtakeMotor.getCurrentPosition());
//        }
        if(gamepad2.dpadRightWasPressed())
            servoArunc.setPosition(servoArunc.getPosition() == 1 ? 0.7 : 1);
        if(gamepad2.aWasPressed())
            shootMotor.setPower(shootMotor.getPower() > 0.0 ? 0.0 : 1);
        if(move)
            MoveToPosition(storageMotor, 2750, 0.5);
        if(smallMove)
            MoveToPosition(storageMotor, 700, 0.6);
        paused = runTime.seconds() - timerServo < 1;
        if(autoSort)
            PatternSortAuto(patterns[lastId]);
        if(autoThrow)
            ThrowAll();
        if (turns > 3)
            turns = 0;
        telemetry.addData("Hue: ", hue);
        telemetry.addData("Sat: ", sat);
        if(idenColor() == 'G')
            telemetry.addData("Artifact", "Green");
        else if(idenColor() == 'P')
            telemetry.addData("Artifact", "Purple");
        else
            telemetry.addData("Artifact", "None");
        telemetry.addData("runTime", runTime.seconds());
        telemetry.addData("AutoSorting", autoSort);
        telemetry.addData("Turns", turns);
        telemetry.addData("Power", storageMotor.getPower());
        telemetry.addData("Busy", storageMotor.isBusy());
        telemetry.addData("Paused", paused);
        telemetry.addData("EncoderStorage", storageMotor.getCurrentPosition());
        telemetry.addData("EncoderOuttake", outtakeMotor.getCurrentPosition());
        telemetry.addData("OuttakeTarget", outtakeMotor.getTargetPosition());
        telemetry.addData("OuttakeDir", outtakeDir);
        telemetry.addData("Move", move);
        telemetry.addData("ShooterServo", servoArunc.getPosition());
        if(blocks.length > 0)
            lastId = blocks[0].id;
        telemetry.addData("Id", lastId + " " + patterns[lastId]);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}