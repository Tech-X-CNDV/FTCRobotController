package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "06: AutoPreloadParkRed", group = "Active")
public class AutoPreloadParkRed extends OpMode {
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
    private final Pose startPose = new Pose(59.52914285714286, 11.45599999999999, Math.toRadians(90)).mirror();
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE.mirror();
    private final Pose parkPose = new Pose(35.829714285714275, 13.630857142857149, Math.toRadians(90)).mirror();

    private PathChain parkPath;

    public void buildPaths() {
        // Path from Start to the Final Parking position
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
                // Give turret and flywheel a moment to align and spin up
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    if (!shootingStarted && storageSubsystem.isIdle()) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    // Once shot is fired, transition to wait state
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(1);
                    }
                }
                break;
            case 1: // Wait 1 second before parking
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(parkPath);
                    setPathState(2);
                }
                break;
            case 2: // Moving to Park position
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
        PoseStorage.autoPoseBlue = currentPose;

        telemetry.addData("Auto State", "%d (%.2f s)", pathState, pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Current Position", currentPose.toString());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
