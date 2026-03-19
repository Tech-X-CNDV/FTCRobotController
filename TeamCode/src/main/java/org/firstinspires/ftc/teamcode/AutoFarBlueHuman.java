package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "03: Blue Far (Human)", group = "Active")
public class AutoFarBlueHuman extends OpMode {
    private List<LynxModule> allHubs;

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsystem intakeSubsystem;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double loopTime;
    private double telemetryTimer = 0;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final double PATH_TIMEOUT = 3.2;

    // Constant Poses
    private final Pose startPose = new Pose(59.52914285714286, 11.45599999999999, Math.toRadians(180));
    private final Pose scorePose = new Pose(57.52914285714286, 13.55599999999999, Math.toRadians(180));
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE;

    public static final Pose pickup3 = new Pose(46.5110334, 34.558042, Math.toRadians(-180));
    public static final Pose getPickUp3 = new Pose(10.357059, 34.558042, Math.toRadians(-180));

    private final Pose humanPickUp = new Pose(10.057059, 33.558042, Math.toRadians(245));
    private final Pose humanPickUpGet = new Pose(10.057059, 12.030857142857149, Math.toRadians(245));

    private final Pose parkPose = new Pose(35.829714285714275, 13.630857142857149, Math.toRadians(180));

    // Paths
    private PathChain startToScorePath, scoreToPickup3Path, pickup3ToGetPickup3Path, getPickup3ToStartPath,
            startToHumanPath, humanPickupToHumanPickupGetPath, humanPickupGetToStartPath, parkPath;

    public void buildPaths() {
        startToScorePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToPickup3Path = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3.getHeading())
                .build();
        pickup3ToGetPickup3Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, getPickUp3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), getPickUp3.getHeading())
                .build();
        getPickup3ToStartPath = follower.pathBuilder()
                .addPath(new BezierLine(getPickUp3, scorePose))
                .setLinearHeadingInterpolation(getPickUp3.getHeading(), scorePose.getHeading())
                .build();
        startToHumanPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, humanPickUp))
                .setLinearHeadingInterpolation(scorePose.getHeading(), humanPickUp.getHeading())
                .build();
        humanPickupToHumanPickupGetPath = follower.pathBuilder()
                .addPath(new BezierLine(humanPickUp, humanPickUpGet))
                .setLinearHeadingInterpolation(humanPickUp.getHeading(), humanPickUpGet.getHeading())
                .build();
        humanPickupGetToStartPath = follower.pathBuilder()
                .addPath(new BezierLine(humanPickUpGet, scorePose))
                .setLinearHeadingInterpolation(humanPickUpGet.getHeading(), scorePose.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate(boolean isBusy, Pose currentPose) {
        switch (pathState) {
            case 0: // Command the first shot
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    storageSubsystem.StartShooting();
                    setPathState(1000); // Wait for the shot to finish
                }
                break;
            case 1000: // Await the shot completion solidly
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(100); // Your 1 Second Pause
                    }
                }
                break;
            case 100: // 1 Second Pause before moving
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(scoreToPickup3Path);
                    setPathState(1);
                }
                break;
            case 1: // Follow score to pickup 3 path
                if (!isBusy) {
                    intakeSubsystem.setPower(1);
                    follower.followPath(pickup3ToGetPickup3Path);
                    setPathState(2);
                }
                break;
            case 2: // Follow pickup 3 reach path
                if (!isBusy) {
                    intakeSubsystem.setPower(-1); // Pulse out to clear jam if any
                    follower.followPath(getPickup3ToStartPath);
                    setPathState(3);
                }
                break;
            case 3: // Return to Score
                if (pathTimer.getElapsedTimeSeconds() > 0.5)
                    intakeSubsystem.setPower(0);

                if (!isBusy) {
                    storageSubsystem.StartShooting();
                    setPathState(4000); // Wait for shooting
                }
                break;
            case 4000: // Await shooting completion
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(4); // Move toward Human Pickup
                    }
                }
                break;
            case 4: // Human Pickup Start
                if (!isBusy) {
                    follower.followPath(startToHumanPath);
                    intakeSubsystem.setPower(1);
                    setPathState(5);
                }
                break;
            case 5: // human pick up alignment
                if (!isBusy) {
                    follower.followPath(humanPickupToHumanPickupGetPath);
                    setPathState(6);
                }
                break;
            case 6: // human pick up get
                if (!isBusy) {
                    intakeSubsystem.setPower(-0.5); // Slow outtake for secure transfer
                    follower.followPath(humanPickupGetToStartPath);
                    setPathState(7);
                }
                break;
            case 7: // return to score Human
                if (pathTimer.getElapsedTimeSeconds() > 0.5)
                    intakeSubsystem.setPower(0);

                if (!isBusy) {
                    storageSubsystem.StartShooting();
                    setPathState(8000); // Wait for shot
                }
                break;

            case 8000:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    if (storageSubsystem.isDoneShooting()) {
                        follower.followPath(parkPath);
                        setPathState(8);
                    }
                }
                break;

            case 8: // Final Park
                if (!isBusy) {
                    setPathState(-1);
                }
                break;
            case -1: // OPMODE Completion
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    intakeSubsystem.setPower(0);
                    outtakeSubsystem.StopShootMotor();
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeSubsystem.InitIntake();
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();
        storageSubsystem = new StorageSubsystem(hardwareMap, outtakeSubsystem);
        storageSubsystem.InitStorage();

        buildPaths();
        follower.setStartingPose(startPose);
        PoseStorage.isRed = false;
        PoseStorage.allianceOffset = Math.toRadians(180);
        PoseStorage.autoPoseBlue = startPose;
    }

    @Override
    public void init_loop() {
        if (allHubs == null) {
            allHubs = hardwareMap.getAll(LynxModule.class);
        }
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        matchTimer.reset();
        outtakeSubsystem.StartShootMotor();
        setPathState(0);
    }

    @Override
    public void loop() {
        if (allHubs == null) {
            allHubs = hardwareMap.getAll(LynxModule.class);
        }
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        double currentTime = timer.milliseconds();
        loopTime = currentTime - lastTime;
        lastTime = currentTime;

        storageSubsystem.update();
        outtakeSubsystem.update();

        Pose currentPose = follower.getPose();
        boolean isBusy = follower.isBusy();

        // Dynamic targeting using Lock Pose and start position
        double deltaX = LOCK_POSE.getX() - currentPose.getX();
        double deltaY = LOCK_POSE.getY() - currentPose.getY();
        outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
        outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);
        // Direct Lock on the high basket with a -3 degree offset
        outtakeSubsystem.updateTurretLock(currentPose, LOCK_POSE, Math.toRadians(-3));

        autonomousPathUpdate(isBusy, currentPose);

        // --- 30s FAILSAFE GUARDIAN ---
        if (matchTimer.seconds() > 29.7) {
            follower.breakFollowing();
            follower.setMaxPower(0);
            outtakeSubsystem.SetShootMotorPower(0);
            intakeSubsystem.setPower(0);
            PoseStorage.autoPoseBlue = currentPose;
            requestOpModeStop();
        }

        PoseStorage.autoPoseBlue = currentPose;
        if (currentTime > telemetryTimer + 100) {
            loopTime = (currentTime - lastTime) / 1000;
            lastTime = currentTime;
            telemetry.addData("Loop Time", "%.2f ms", loopTime);
            telemetry.addData("State", "%d (Time: %.2f s)", pathState, pathTimer.getElapsedTimeSeconds());
            telemetry.addData("Current Position", currentPose.toString());
            telemetry.addData("Drive X", "%.2f", currentPose.getX());
            telemetry.addData("Drive Y", "%.2f", currentPose.getY());
            telemetry.addData("Drive Heading", "%.2f", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
            telemetryTimer = currentTime;
        }

        follower.update();
    }

    @Override
    public void stop() {
    }
}
