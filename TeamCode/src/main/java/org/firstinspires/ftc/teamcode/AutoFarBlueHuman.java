package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "09: AutoFarBlueHuman", group = "Active")
public class AutoFarBlueHuman extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsystem;
    private boolean shootingStarted = false;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Constant Poses
    private final Pose startPose = new Pose(59.52914285714286, 11.45599999999999, Math.toRadians(180));
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE;

    private final Pose pickup3 = FieldPoses.PICKUP_3;
    private final Pose getPickUp3 = FieldPoses.GET_PICK_3;

    private final Pose humanPickUp = new Pose(10.657059, 13.630857142857149, Math.toRadians(180));

    private final Pose parkPose = new Pose(35.829714285714275, 13.630857142857149, Math.toRadians(180));

    private PathChain startToPickup3Path, pickup3ToGetPickup3Path, getPickup3ToStartPath, startToHumanPath, humanToStartPath, parkPath;

    public void buildPaths() {
        startToPickup3Path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup3))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup3.getHeading())
                .build();
        pickup3ToGetPickup3Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, getPickUp3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), getPickUp3.getHeading())
                .build();
        getPickup3ToStartPath = follower.pathBuilder()
                .addPath(new BezierLine(getPickUp3, startPose))
                .setLinearHeadingInterpolation(getPickUp3.getHeading(), startPose.getHeading())
                .build();
        startToHumanPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, humanPickUp))
                .setLinearHeadingInterpolation(startPose.getHeading(), humanPickUp.getHeading())
                .build();
        humanToStartPath = follower.pathBuilder()
                .addPath(new BezierLine(humanPickUp, startPose))
                .setLinearHeadingInterpolation(humanPickUp.getHeading(), startPose.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate(boolean isBusy, Pose currentPose) {
        // Ensure shooting guard is reset once storage is idle
        if (storageSubsystem.isIdle()) {
            shootingStarted = false;
        }

        switch (pathState) {
            case 0: // Shoot from the Start position
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    if (!shootingStarted && storageSubsystem.isIdle()) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isDoneShooting()) {
                        follower.followPath(startToPickup3Path, true);
                        setPathState(1);
                    }
                }
                break;
            case 1: // Moving to pickup3
                if (!isBusy) {
                    follower.followPath(pickup3ToGetPickup3Path, true);
                    intakeSubsystem.setPower(1.0);
                    setPathState(2);
                }
                break;
            case 2: // Moving to getPickUp3
                if (!isBusy) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.8) { // Intake delay
                        follower.followPath(getPickup3ToStartPath, true);
                        intakeSubsystem.setPower(0);
                        setPathState(3);
                    }
                }
                break;
            case 3: // Returning to Start position for pickup3 shoot
                if (!isBusy) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        if (!shootingStarted && storageSubsystem.isIdle()) {
                            storageSubsystem.StartShooting();
                            shootingStarted = true;
                        }
                        if (storageSubsystem.isDoneShooting()) {
                            follower.followPath(startToHumanPath, true);
                            intakeSubsystem.setPower(1.0);
                            setPathState(4);
                        }
                    }
                }
                break;
            case 4: // Moving to human player zone
                if (!isBusy) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) { // Stay and intake
                        follower.followPath(humanToStartPath, true);
                        intakeSubsystem.setPower(0);
                        setPathState(5);
                    }
                }
                break;
            case 5: // Returning to Start position for human shoot
                if (!isBusy) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        if (!shootingStarted && storageSubsystem.isIdle()) {
                            storageSubsystem.StartShooting();
                            shootingStarted = true;
                        }
                        if (storageSubsystem.isDoneShooting()) {
                            follower.followPath(parkPath, true);
                            setPathState(6);
                        }
                    }
                }
                break;
            case 6: // Moving to Park position
                if (!isBusy) {
                    setPathState(-1); // Finished
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
        storageSubsystem.ResetToIntake(); // Home during init

        intakeSubsystem = new IntakeSubsytem(hardwareMap);
        intakeSubsystem.InitIntake();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pickup3.setHeading(Math.toRadians(180));
        getPickUp3.setHeading(Math.toRadians(180));
        buildPaths();
        follower.setStartingPose(startPose);

        PoseStorage.allianceOffset = Math.toRadians(180);
        PoseStorage.isRed = false;
        PoseStorage.autoPoseBlue = startPose;
    }

    @Override
    public void init_loop() {
        storageSubsystem.update();
        outtakeSubsystem.update();
        telemetry.addData("Storage State", storageSubsystem.getState());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        matchTimer.reset();
        outtakeSubsystem.setManualVelocityOffset(200);
        outtakeSubsystem.StartShootMotor();
        PoseStorage.isRed = false;
        PoseStorage.allianceOffset = Math.toRadians(180);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();

        Pose currentPose = follower.getPose();
        boolean isBusy = follower.isBusy();

        // Dynamic targeting using Lock Pose and start position
        double deltaX = LOCK_POSE.getX() - currentPose.getX();
        double deltaY = LOCK_POSE.getY() - currentPose.getY();
        outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
        outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);
        // Direct Lock on the high basket with a -5 degree offset
        outtakeSubsystem.updateTurretLock(currentPose, LOCK_POSE, Math.toRadians(-5));

        autonomousPathUpdate(isBusy, currentPose);

        // --- 30s FAILSAFE GUARDIAN ---
        if (matchTimer.seconds() > 29.8) {
            follower.breakFollowing();
            follower.setMaxPower(0);
            outtakeSubsystem.SetShootMotorPower(0);
            intakeSubsystem.setPower(0);
            PoseStorage.autoPoseBlue = currentPose;
            requestOpModeStop();
        }

        PoseStorage.autoPoseBlue = currentPose;

        telemetry.addData("Auto State", "%d (%.2f s)", pathState, pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Current Position", currentPose.toString());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
