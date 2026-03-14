package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "02: AutoBlueV2", group = "Active")
public class AutonomieBlueV2 extends OpMode {
    private List<LynxModule> allHubs;

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsystem intakeSubsystem;
    private boolean shootingStarted = false; // guards StartShooting() so it is only called once per state
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private double lastTime = 0;
    private double loopTime;
    private double telemetryTimer = 0;
    private double manualTurretOffset = Math.toRadians(-3);
    private boolean turretLockEnabled = true;
    private final double PATH_TIMEOUT = 5.0;

    // --- STUCK FAILSAFE VARIABLES ---
    private final ElapsedTime stuckTimer = new ElapsedTime();
    private Pose lastFailsafePose = new Pose(0, 0, 0);
    private double lastDistToTarget = 0.0;
    private boolean isRecovering = false;
    private final double STUCK_THRESHOLD_INCHES = 0.25;
    private final double STUCK_CHECK_INTERVAL_MS = 500;

    // Pose Constants
    private final Pose startPose = FieldPoses.START;
    private final Pose scorePose = FieldPoses.SCORE;
    private final Pose scoreFinal = FieldPoses.SCORE_FINAL;
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE;
    private Pose targetPose;

    // Pickup 1
    private final Pose pickup1 = FieldPoses.PICKUP_1;
    private final Pose getPick1 = FieldPoses.GET_PICK_1;
    private final Pose posGate = FieldPoses.POS_GATE;
    private final Pose openGate = FieldPoses.OPEN_GATE;

    // Pickup 2
    private final Pose pickup2 = FieldPoses.PICKUP_2;
    private final Pose getPick2 = FieldPoses.GET_PICK_2;
    // private final Pose getPick2Back = FieldPoses.GET_PICK_2_BACK;

    // Pickup 3
    private final Pose pickup3 = FieldPoses.PICKUP_3;
    private final Pose getPick3 = FieldPoses.GET_PICK_3;

    private final Pose cycle = FieldPoses.cycle;
    private final Pose cycle2 = FieldPoses.cycle2;
    private final Pose cyclePoint = FieldPoses.cyclePoint;

    // Parking
    private final Pose parkPose = FieldPoses.PARK;
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path8_2, path9, path10, path11;

    public void buildPaths() {
        // path1: Start to Preload Score
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // path2: Score to Pickup 1 Alignment
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1.getHeading())
                .build();

        // path3: Intake Reach 1
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, getPick1))
                .setConstantHeadingInterpolation(getPick1.getHeading())
                .build();

        // path4: Return to Score 1 (Direct)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(getPick1, scoreFinal))
                .setLinearHeadingInterpolation(getPick1.getHeading(), scoreFinal.getHeading())
                .build();

        // path5: Score to Pickup 2 Alignment
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2.getHeading())
                .build();

        // path6: Intake Reach 2
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, getPick2))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();

        // Path 7: Swing out to avoid the obstacle on the left
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(getPick2, new Pose(65, 65), scorePose))
                .setLinearHeadingInterpolation(getPick2.getHeading(), scorePose.getHeading())
                .build();

        // path8: Score to Cycle Alignment P1
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cyclePoint, cycle))
                .setLinearHeadingInterpolation(scorePose.getHeading(), cycle.getHeading())
                .build();

        // path8_2: Score to Cycle Alignment P2
        path8_2 = follower.pathBuilder()
                .addPath(new BezierLine(cycle, cycle2))
                .setLinearHeadingInterpolation(cycle.getHeading(), cycle2.getHeading())
                .build();

        // path9: Cycle Return to Score
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(cycle2, cyclePoint, scorePose))
                .setLinearHeadingInterpolation(cycle2.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate(boolean isBusy, Pose currentPose) {
        // Global Guard: Only allow the shooting variable to reset when homing is 100%
        // finished
        if (storageSubsystem.isIdle()) {
            shootingStarted = false;
        }

        boolean timedOut = pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT;

        switch (pathState) {
            case 0: // Move to Preload Score
                follower.followPath(path1, true);
                follower.setMaxPower(1);
                setPathState(1);
                break;
            case 1: // SHOOTING: Preload
                if (!isBusy) {
                    if (!shootingStarted && storageSubsystem.isIdle()) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    // Wait until the shot has left (even if still homing) before moving on
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(2);
                    }
                }
                break;
            // ================= PICKUP 2 SEQUENCE =================
            case 2: // ALIGN to Pickup 2 (Path 5)
                if (!isBusy) {
                    follower.followPath(path5); // Correctly call alignment path
                    setPathState(3);
                }
                break;
            case 3: // STAB/INTAKE 2 (Path 6)
                if (!isBusy && storageSubsystem.isIdle()) {
                    intakeSubsystem.setPower(1);
                    follower.followPath(path6);
                    setPathState(4);
                }
                break;
            case 4: // RETURN to Score (Path 7)
                if (!isBusy) {
                    follower.followPath(path7, true);
                    setPathState(5);
                }
                // storageSubsystem.MoveRelative(475, 1);
                break;
            case 5: // ARRIVED Score
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSubsystem.setPower(-1);
                }
                if (!isBusy) {
                    setPathState(6);
                }
                break;
            case 6: // SHOOTING 1
                if (!isBusy) {
                    // intakeSubsystem.setPower(0);
                    if (!shootingStarted && storageSubsystem.isIdle()) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isDoneShooting()) {
                        follower.setMaxPower(0.9);
                        follower.followPath(path8); // Align to Cycle P1
                        intakeSubsystem.setPower(1);
                        setPathState(7);
                    }
                }
                break;
            // ================= CYCLE SEQUENCE =================
            case 7: // Finish Cycle Alignment
                if (!isBusy) {
                    follower.setMaxPower(1);
                    follower.followPath(path8_2); // Align to Cycle P2
                    setPathState(8);
                }
                break;
            case 8: // Wait for fragments and return
                if ((!isBusy && storageSubsystem.isIdle()) || timedOut) {
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(path9);
                        setPathState(9);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;
            case 9: // Shooting Cycle
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSubsystem.setPower(-1);
                }
                if (!isBusy) {
                    if (!shootingStarted && storageSubsystem.isIdle()) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isDoneShooting()) {
                        // if (matchTimer.seconds() > 17) {
                        follower.followPath(path2);
                        setPathState(10);
                        // }
                        // } else {
                        // follower.followPath(path8); // Loop back to start alignment
                        // intakeSubsystem.setPower(1);
                        // setPathState(7);
                        // }
                    }
                }
                break;
            case 10: // ARRIVED intake 1
                if (!isBusy) {
                    intakeSubsystem.setPower(1);
                    follower.followPath(path3);
                    setPathState(11);
                }
                break;
            case 11: // STAB/INTAKE 1
                if (!isBusy && storageSubsystem.isIdle()) {
                    follower.followPath(path4);
                    setPathState(12);
                }
                break;
            case 12: // Final Shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSubsystem.setPower(-1);
                }
                if (!isBusy) {
                    // intakeSubsystem.setPower(0);
                    if (!shootingStarted && storageSubsystem.isIdle()) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isDoneShooting()) {
                        setPathState(-1);
                    }
                }
                break;
            case -1:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    outtakeSubsystem.StopShootMotor();
                    intakeSubsystem.setPower(0);
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
        storageSubsystem.ResetToIntake(); // Start homing during init
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeSubsystem.InitIntake();

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        PoseStorage.allianceOffset = Math.toRadians(180);
        PoseStorage.isRed = false;
        PoseStorage.autoPoseBlue = startPose;
        targetPose = LOCK_POSE;
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
        outtakeSubsystem.StartShootMotor();
        PoseStorage.isRed = false;
        PoseStorage.allianceOffset = Math.toRadians(180);
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

        boolean isBusy = follower.isBusy();
        Pose currentPose = follower.getPose();

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
        // if (actionTimer.getElapsedTimeSeconds() < 0.4) {
        // // Use the 'target' variable declared at the top
        // double angleToTarget = Math.atan2(target.getY() - currentPose.getY(),
        // target.getX() - currentPose.getX());

        // double escapeAngle = angleToTarget + Math.PI;
        // double escapeX = Math.cos(escapeAngle) * 0.25;
        // double escapeY = Math.sin(escapeAngle) * 0.25;

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

        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();

        // Centralized Dynamic Aim (Power & Angle) â€” always active
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
        outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);

        if (turretLockEnabled) {
            outtakeSubsystem.updateTurretLock(currentPose, targetPose, manualTurretOffset);
        }

        // --- 30s FAILSAFE GUARDIAN ---
        if (matchTimer.seconds() > 29.7) {
            follower.breakFollowing();
            follower.setMaxPower(0);
            outtakeSubsystem.SetShootMotorPower(0);
            intakeSubsystem.setPower(0);
            PoseStorage.autoPoseBlue = currentPose;
            requestOpModeStop();
        }

        autonomousPathUpdate(isBusy, currentPose);
        PoseStorage.autoPoseBlue = currentPose;

        /*
         * if (storageSubsystem.recoveryState ==
         * StorageSubsystem.RecoveryState.WAITING_FOR_RETRY
         * || storageSubsystem.recoveryState ==
         * StorageSubsystem.RecoveryState.RETURNING)
         * intakeSubsystem.setPower(1);
         */
        if (currentTime > telemetryTimer + 100) {
            // --- AUTO STATUS ---
            telemetry.addData("Loop Time", "%.2f ms", loopTime);
            telemetry.addData("State", "%d (Time: %.2f s)", pathState, pathTimer.getElapsedTimeSeconds());

            // --- DRIVE / POSITION ---
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
        // PoseStorage.autoPoseBlue = follower.getPose();
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
