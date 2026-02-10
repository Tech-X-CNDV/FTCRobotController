package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoMicBlue")
public class AutoMic extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;

    // public Outtake motorOuttake,servoPusher;

    // Pose Constants for the Blue Side
    private final Pose startPose = new Pose(82.89828571428572, 9.152000000000037, Math.toRadians(90));
    private final Pose scorePose = new Pose(85.89571428571428, 23.147714285714294, Math.toRadians(67.5));
    private final Pose parkPose = new Pose(108.17028571428573, 10.630857142857149, Math.toRadians(0));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;
    boolean turned = false;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                // .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .addPath(new BezierLine(startPose, scorePose))
                // .setConstantHeadingInterpolation(Math.toRadians(89))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        path2 = follower.pathBuilder()
                // .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .addPath(new BezierLine(scorePose, parkPose))
                // .setConstantHeadingInterpolation(Math.toRadians(89))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds() > 13)
                    outtakeSubsystem.ToggleShootMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 18)
                    setPathState(1);
                break;
            case 1:
                outtakeSubsystem.SetAngle(0.7);
                follower.followPath(path1);
                follower.setMaxPower(0.9);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                        storageSubsystem.setServoPos(0.6);
                        setPathState(3);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5)
                    storageSubsystem.setServoPos(1);
                if (pathTimer.getElapsedTimeSeconds() > 1 && !turned) {
                    storageSubsystem.MoveRelative(475, 1);
                    turned = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5 && !storageSubsystem.isBusy()) {
                    storageSubsystem.setServoPos(0.6);
                    turned = false;
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5)
                    storageSubsystem.setServoPos(1);
                if (pathTimer.getElapsedTimeSeconds() > 1 && !turned) {
                    storageSubsystem.MoveRelative(475, 1);
                    turned = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5 && !storageSubsystem.isBusy()) {
                    storageSubsystem.setServoPos(0.6);
                    turned = false;
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    storageSubsystem.setServoPos(1);
                    outtakeSubsystem.ToggleShootMotorAuto();
                    follower.followPath(path2);
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
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();

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
        outtakeSubsystem.SetAutoAim(true);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();
        autonomousPathUpdate();
        // --- AUTO STATUS ---
        telemetry.addData("State", "%d (Time: %.2f s)", pathState, pathTimer.getElapsedTimeSeconds());

        // --- SUBSYSTEMS TELEMETRY ---
        intakeSubsytem.displayTelemetry(telemetry);
        storageSubsystem.displayTelemetry(telemetry);
        outtakeSubsystem.displayTelemetry(telemetry);

        telemetry.update();
    }
}
