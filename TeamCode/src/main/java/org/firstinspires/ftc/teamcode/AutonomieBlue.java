package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "AutoBlue")
public class AutonomieBlue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;

//    public Outtake motorOuttake,servoPusher;

    // Pose Constants for the Red Side
    private final Pose startPose = new Pose(120.74971428571429, 126.49142857142861, Math.toRadians(-143));
    private final Pose scorePose = new Pose(93.10171428571429, 91.78971428571427, Math.toRadians(40));
    private final Pose pickup1 = new Pose(93.10171428571429, 78.75771428571429, Math.toRadians(0));
    private final Pose getPick1 = new Pose(128.39428571428572, 78.75771428571429, Math.toRadians(0));
    private final Pose pickup2 = new Pose(93.10171428571429, 55.325714285714284, Math.toRadians(0));
    private final Pose getPick2 = new Pose(135.39657142857143, 55.325714285714284, Math.toRadians(0));
    private final Pose getPick2Back = new Pose(128.39657142857143, 55.325714285714284, Math.toRadians(0));
    private final Pose parkPose = new Pose(128.39428571428572, 88.75771428571429, Math.toRadians(0));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, getPick1))
                .setConstantHeadingInterpolation(getPick1.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(getPick1, scorePose))
                .setLinearHeadingInterpolation(getPick1.getHeading(), scorePose.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, getPick2))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(getPick2, getPick2Back))
                .setConstantHeadingInterpolation(getPick2.getHeading())
                .addPath(new BezierLine(getPick2Back, scorePose)) // Corrected the jump from getPick2 to scorePose
                .setLinearHeadingInterpolation(getPick2.getHeading(), scorePose.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                outtakeSubsystem.ToggleShootMotorAuto();
                follower.setMaxPower(0.55);
                storageSubsystem.autoThrow = true;
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if(storageSubsystem.autoThrow){
                        storageSubsystem.ThrowAll();
                    }
                    if(!storageSubsystem.autoThrow) {
                        storageSubsystem.setServoPos(1);
                        follower.setMaxPower(1);
                        setPathState(2);
                    }
                }else{
                    storageSubsystem.servoTimer.reset();
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.85);
                    follower.followPath(path3);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(path4);
                    setPathState(5);
                }
                intakeSubsytem.setPower(1);
                storageSubsystem.MoveToPosition(475, 1);
                break;

            case 5:
                if(!follower.isBusy()){
                    intakeSubsytem.setPower(0);
                    storageSubsystem.autoThrow = true;
                    setPathState(6);
                }else{
                    intakeSubsytem.setPower(1);
                    storageSubsystem.MoveToPosition(475, 1);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    if(storageSubsystem.autoThrow){
                        storageSubsystem.ThrowAll();
                    }
                    if(!storageSubsystem.autoThrow) {
                        storageSubsystem.setServoPos(1);
                        follower.followPath(path5);
                        setPathState(7);
                    }
                }else{
                    storageSubsystem.servoTimer.reset();
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.85);
                    follower.followPath(path6);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(path7);
                    setPathState(9);
                }
                intakeSubsytem.setPower(1);
                storageSubsystem.MoveToPosition(475,1);
                break;

            case 9:
                if(!follower.isBusy()){
                    intakeSubsytem.setPower(0);
                    storageSubsystem.autoThrow = true;
                    setPathState(10);
                }else{
                    intakeSubsytem.setPower(1);
                    storageSubsystem.MoveToPosition(475,1);
                }
                break;

            case 10:
                if(!follower.isBusy()){
                    if(storageSubsystem.autoThrow){
                        storageSubsystem.ThrowAll();
                    }
                    if(!storageSubsystem.autoThrow) {
                        storageSubsystem.setServoPos(1);
                        follower.followPath(path8);
                        setPathState(-1);
                    }
                }else{
                    storageSubsystem.servoTimer.reset();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();
        outtakeSubsystem.AutoAngle();

        storageSubsystem = new StorageSubsystem(hardwareMap);
        storageSubsystem.InitStorage();

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("AutoThrow", storageSubsystem.autoThrow);
        telemetry.update();
    }
}
