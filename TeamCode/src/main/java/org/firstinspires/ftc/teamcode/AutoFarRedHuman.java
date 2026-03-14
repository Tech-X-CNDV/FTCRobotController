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

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "10: AutoFarRedHuman", group = "Active")
public class AutoFarRedHuman extends OpMode {
    private List<LynxModule> allHubs;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsystem intakeSubsystem;
    private boolean shootingStarted = false;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double loopTime;
    private double telemetryTimer = 0;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final double PATH_TIMEOUT = 3.2;

    // --- STUCK FAILSAFE VARIABLES ---
    private final ElapsedTime stuckTimer = new ElapsedTime();
    private Pose lastFailsafePose = new Pose(0, 0, 0);
    private double lastDistToTarget = 0.0;
    private boolean isRecovering = false;
    private final double STUCK_THRESHOLD_INCHES = 0.5;
    private final double STUCK_CHECK_INTERVAL_MS = 250;

    // Constant Poses
    private final Pose startPose = new Pose(59.52914285714286, 11.45599999999999, Math.toRadians(180)).mirror();
    private final Pose scorePose = new Pose(57.52914285714286, 13.55599999999999, Math.toRadians(180)).mirror();
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE.mirror();

    public static final Pose pickup3 = new Pose(46.5110334, 35.058042, Math.toRadians(-180)).mirror();
    public static final Pose getPickUp3 = new Pose(10.357059, 35.058042, Math.toRadians(-180)).mirror();

    private final Pose humanPickUp = new Pose(10.057059, 33.558042, Math.toRadians(245)).mirror();
    private final Pose humanPickUpGet = new Pose(10.057059, 12.030857142857149, Math.toRadians(245)).mirror();

    private final Pose parkPose = new Pose(35.829714285714275, 13.630857142857149, Math.toRadians(180)).mirror();

    private PathChain startToPickup3Path, pickup3ToGetPickup3Path, getPickup3ToStartPath, startToHumanPath,
            humanPickupToHumanPickupGetPath, humanPickupGetToStartPath, parkPath;

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
        // The 'shootingStarted = false' guard has been completely removed
        // because it was resetting too quickly and causing infinite StartShooting()
        // loops.

        boolean timedOut = pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT;

        switch (pathState) {
            case 0: // Command the first shot
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    storageSubsystem.StartShooting();
                    setPathState(1000); // Wait for the shot to finish
                }
                break;
            case 1000: // Await the shot completion solidly
                // The delay guarantees the StorageSubsystem isn't IDLE momentarily at startup
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(100); // Your 1 Second Pause
                    }
                }
                break;
            case 100: // Wait 1 second after shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(startToPickup3Path, true);
                    setPathState(1);
                }
                break;
            case 1: // Moving to pickup3
                if (!isBusy || timedOut) {
                    follower.followPath(pickup3ToGetPickup3Path, true);
                    intakeSubsystem.setPower(1.0);
                    setPathState(2);
                }
                break;
            case 2: // Moving to getPickUp3
                if (!isBusy || timedOut) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.8) { // Intake delay
                        follower.followPath(getPickup3ToStartPath, true);
                        setPathState(3);
                    }
                }
                break;
            case 3: // Command the second shot for pickup3
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    intakeSubsystem.setPower(-1);
                }
                if (!isBusy) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        storageSubsystem.StartShooting();
                        setPathState(1003); // Wait for shot
                    }
                }
                break;
            case 1003: // Await shot completion
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(103); // Your 1 Second Pause
                    }
                }
                break;
            case 103: // Wait 1 second after shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(startToHumanPath, true);
                    intakeSubsystem.setPower(1.0);
                    setPathState(4);
                }
                break;
            case 4: // Moving to human player alignment
                if (!isBusy || timedOut) {
                    follower.followPath(humanPickupToHumanPickupGetPath, true);
                    setPathState(5);
                }
                break;
            case 5: // Moving to human player GET position
                if (!isBusy || timedOut) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) { // Intake delay
                        follower.followPath(humanPickupGetToStartPath, true);
                        setPathState(6);
                    }
                }
                break;
            case 6: // Command final human shoot
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    intakeSubsystem.setPower(-1);
                }
                if (!isBusy) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        storageSubsystem.StartShooting();
                        setPathState(1006); // Wait for shot
                    }
                }
                break;
            case 1006: // Await shot completion solidly
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(106); // Your 1 Second Pause
                    }
                }
                break;
            case 106: // Wait 1 second after shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (matchTimer.seconds() < 20) {
                        intakeSubsystem.setPower(1.0);
                        follower.followPath(startToHumanPath, true);
                        setPathState(4);
                    } else {
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(parkPath, true);
                    setPathState(8);
                }
                break;
            case 8: // Moving to Park position
                if (!isBusy || timedOut) {
                    outtakeSubsystem.StopShootMotor();
                    intakeSubsystem.setPower(0);
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();

        storageSubsystem = new StorageSubsystem(hardwareMap, outtakeSubsystem);
        storageSubsystem.InitStorage();
        storageSubsystem.ResetToIntake(); // Home during init

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeSubsystem.InitIntake();

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        PoseStorage.allianceOffset = 0;
        PoseStorage.isRed = true;
        PoseStorage.autoPoseRed = startPose;
    }

    @Override
    public void init_loop() {
        if (allHubs == null) {
            allHubs = hardwareMap.getAll(LynxModule.class);
        }
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        storageSubsystem.update();
        outtakeSubsystem.update();
        telemetry.addData("Storage State", storageSubsystem.getState());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        matchTimer.reset();
        outtakeSubsystem.setManualVelocityOffset(165);
        outtakeSubsystem.StartShootMotor();
        PoseStorage.isRed = true;
        PoseStorage.allianceOffset = 0;
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

        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();

        Pose currentPose = follower.getPose();
        boolean isBusy = follower.isBusy();

        // // --- GLOBAL STUCK FAILSAFE (Improved for Slippage) ---

        // // 1. Current distance to the goal
        // Pose target = follower.getPose();
        // double currentDistToTarget = Math.hypot(
        // target.getX() - currentPose.getX(),
        // target.getY() - currentPose.getY());

        // // 2. The "Hold Point" Safety Check
        // boolean isActuallyMovingToTarget = follower.isBusy() && currentDistToTarget >
        // 1.2;

        // if (isActuallyMovingToTarget && !isRecovering) {
        // if (stuckTimer.milliseconds() > STUCK_CHECK_INTERVAL_MS) {

        // // CHECK: How much did our progress toward the target improve?
        // // Positive value = we got closer. Negative = we drifted away.
        // double progressMade = lastDistToTarget - currentDistToTarget;

        // // NEW THRESHOLD: If we haven't closed the gap by at least 0.25 inches
        // if (progressMade < STUCK_THRESHOLD_INCHES) {
        // isRecovering = true;
        // actionTimer.resetTimer();
        // follower.breakFollowing();
        // }

        // // Update tracking variables for the next interval
        // lastDistToTarget = currentDistToTarget;
        // stuckTimer.reset();
        // }
        // } else {
        // // If we aren't "busy" or are within the 1.2" deadzone,
        // // keep the progress tracker synced so it doesn't "jump" when a new path
        // starts.
        // lastDistToTarget = currentDistToTarget;
        // }

        // if (isRecovering) {
        // if (actionTimer.getElapsedTimeSeconds() < 0.5) {
        // // Use the 'target' variable declared at the top
        // double angleToTarget = Math.atan2(target.getY() - currentPose.getY(),
        // target.getX() - currentPose.getX());

        // double escapeAngle = angleToTarget + Math.PI;
        // double escapeX = Math.cos(escapeAngle) * 0.5;
        // double escapeY = Math.sin(escapeAngle) * 0.5;

        // follower.setTeleOpDrive(escapeX, escapeY, 0.0, false, 0.0);
        // } else {
        // follower.setTeleOpDrive(0.0, 0.0, 0.0, false, 0.0);
        // isRecovering = false;
        // stuckTimer.reset();
        // retriggerCurrentPath();
        // }
        // } else {
        // // Only update the path follower if we aren't nudging
        // follower.update();
        // }

        // Dynamic targeting using Lock Pose and start position
        double deltaX = LOCK_POSE.getX() - currentPose.getX();
        double deltaY = LOCK_POSE.getY() - currentPose.getY();
        outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
        outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);
        // Direct Lock on the high basket with a -5 degree offset
        outtakeSubsystem.updateTurretLock(currentPose, LOCK_POSE, Math.toRadians(-3));

        autonomousPathUpdate(isBusy, currentPose);

        // --- 30s FAILSAFE GUARDIAN ---
        if (matchTimer.seconds() > 29.7) {
            follower.breakFollowing();
            follower.setMaxPower(0);
            outtakeSubsystem.SetShootMotorPower(0);
            intakeSubsystem.setPower(0);
            PoseStorage.autoPoseRed = currentPose;
            requestOpModeStop();
        }

        PoseStorage.autoPoseRed = currentPose;
        if (currentTime > telemetryTimer + 100) {
            loopTime = (currentTime - lastTime) / 1000;
            lastTime = currentTime;
            telemetry.addData("Loop Time", "%.2f ms", loopTime);
            telemetry.addData("State", "%d (Time: %.2f s)", pathState, pathTimer.getElapsedTimeSeconds());
            telemetry.addData("Current Position", currentPose.toString());
            telemetry.addData("Drive X", "%.2f", currentPose.getX());
            telemetry.addData("Drive Y", "%.2f", currentPose.getY());
            telemetry.addData("Drive Heading", "%.2f", Math.toDegrees(currentPose.getHeading()));
            // --- SUBSYSTEMS TELEMETRY ---
            intakeSubsystem.displayTelemetry(telemetry);
            storageSubsystem.displayTelemetry(telemetry);
            outtakeSubsystem.displayTelemetry(telemetry);
            telemetry.update();
            telemetryTimer = currentTime;
        }
    }

    @Override
    public void stop() {
    }

    private void retriggerCurrentPath() {
        // 1. Refresh path definitions to ensure everything is up to date
        buildPaths();

        // 2. Simply 'reset' the current state.
        // This re-enters the current case in autonomousPathUpdate()
        // and re-triggers the followPath() command.
        setPathState(pathState);
    }
}
