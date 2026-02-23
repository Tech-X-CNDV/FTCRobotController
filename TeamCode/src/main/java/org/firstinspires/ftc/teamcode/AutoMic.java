package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoMicRed")
public class AutoMic extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;
    private final ElapsedTime matchTimer = new ElapsedTime();

    // public Outtake motorOuttake,servoPusher;

    // Pose Constants for the Blue Side
    private Pose startPose = new Pose(84.47085714285714, 11.45599999999999, Math.toRadians(69));
    private final Pose scorePose = new Pose(84.47085714285714, 13.05599999999999, Math.toRadians(69));
    private final Pose pickup3 = FieldPoses.PICKUP_3.mirror();
    private Pose getPickup3 = FieldPoses.GET_PICK_3.mirror();
    private final Pose parkPose = new Pose(108.17028571428573, 10.630857142857149, Math.toRadians(0));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;
    boolean turned = false;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                // .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .addPath(new BezierLine(startPose, pickup3))
                // .setConstantHeadingInterpolation(Math.toRadians(89))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup3.getHeading())
                .build();
        path2 = follower.pathBuilder()
                // .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .addPath(new BezierLine(pickup3, getPickup3))
                // .setConstantHeadingInterpolation(Math.toRadians(89))
                .setConstantHeadingInterpolation(pickup3.getHeading())
                .build();
        path3 = follower.pathBuilder()
                // .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .addPath(new BezierLine(getPickup3, scorePose))
                // .setConstantHeadingInterpolation(Math.toRadians(89))
                .setLinearHeadingInterpolation(getPickup3.getHeading(), scorePose.getHeading())
                .build();
        path4 = follower.pathBuilder()
                // .addPath(new BezierCurve(startPose, new Pose(58.5, 97.2), scorePose))
                .addPath(new BezierLine(scorePose, parkPose))
                // .setConstantHeadingInterpolation(Math.toRadians(89))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                outtakeSubsystem.SetShootMotorPower(0.8);
                outtakeSubsystem.SetAngle(1);
                storageSubsystem.autoThrow = true;
                if (pathTimer.getElapsedTimeSeconds() > 4)
                    setPathState(1);
                break;
            case 1:
                if (storageSubsystem.autoThrow) {
                    storageSubsystem.ThrowAll(0.6);
                } else {
                    storageSubsystem.setServoPos(1);
                    follower.setMaxPower(1);
                    follower.followPath(path1);
                    setPathState(2); // Move to Alignment
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    intakeSubsytem.setPower(1);
                    follower.followPath(path2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    startPose.setHeading(startPose.getHeading() - Math.toRadians(5));
                    buildPaths();
                    follower.followPath(path3, true);
                    setPathState(4);
                }
                storageSubsystem.MoveRelative(475, 1);
                break;
            case 4:
                if (!follower.isBusy()) {
                    storageSubsystem.autoThrow = true;
                    setPathState(5);
                } else {
                    storageSubsystem.MoveRelative(475, 1);
                }
                break;
            case 5:
                if (storageSubsystem.autoThrow) {
                    storageSubsystem.ThrowAll(0.6);
                } else {
                    storageSubsystem.setServoPos(1);
                    outtakeSubsystem.SetShootMotorPower(0);
                    intakeSubsytem.setPower(0);
                    follower.setMaxPower(1);
                    follower.followPath(path4, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        storageSubsystem.InitStorage();

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        getPickup3 = new Pose(getPickup3.getX(), getPickup3.getY() + 2, getPickup3.getHeading());
        pickup3.setHeading(Math.toRadians(50));
        PoseStorage.isRed = true;
        PoseStorage.allianceOffset = 0;
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        matchTimer.reset();
        outtakeSubsystem.SetAutoAim(true);
        PoseStorage.isRed = true; // AutoMic is currently Red
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();

        Pose currentPose = follower.getPose();
        PoseStorage.autoPoseRed = currentPose;

        // --- 30s FAILSAFE GUARDIAN ---
        if (matchTimer.seconds() > 29.8) {
            follower.breakFollowing();
            follower.setMaxPower(0);
            outtakeSubsystem.SetShootMotorPower(0);
            intakeSubsytem.setPower(0);
            PoseStorage.isRed = true;
            PoseStorage.autoPoseRed = currentPose;
            requestOpModeStop();
        }

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
