package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "autoUnpunct")
public class Autounpunct extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;

//    public Outtake motorOuttake,servoPusher;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(89));
    private final Pose scorePose = new Pose(39, 103.5, Math.toRadians(89));
    private final Pose parkPose = new Pose(55.9, 7.9, Math.toRadians(89));
    private PathChain path1, path2;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(89))
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(58.7, 97.2), parkPose))
                .setConstantHeadingInterpolation(Math.toRadians(89))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
//                    motorOuttake.setPower(1);
//                    if (pathTimer.getElapsedTimeSeconds() > 1.0)
//                        servoPusher.up();
                    follower.followPath(path2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
//                    motorOuttake.setPower(0);
//                    servoPusher.down();
                    setPathState(-1);
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
        telemetry.update();
    }
}
