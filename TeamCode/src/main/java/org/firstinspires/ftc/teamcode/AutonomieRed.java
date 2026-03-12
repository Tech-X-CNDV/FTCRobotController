package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "04: AutoRed", group = "Active")
public class AutonomieRed extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;
    private boolean shootingStarted = false; // guards StartShooting() so it is only called once per state
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private double lastTime = 0;
    private double loopTime;

    // Pose Constants
    private final Pose startPose = FieldPoses.START.mirror();
    private final Pose scorePose = FieldPoses.SCORE.mirror();
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE.mirror();
    private Pose targetPose;

    // Pickup 1
    private final Pose pickup1 = FieldPoses.PICKUP_1.mirror();
    private final Pose getPick1 = FieldPoses.GET_PICK_1.mirror();
    private final Pose posGate = FieldPoses.POS_GATE.mirror();
    private final Pose openGate = FieldPoses.OPEN_GATE.mirror();

    // Pickup 2
    private final Pose pickup2 = FieldPoses.PICKUP_2.mirror();
    private final Pose getPick2 = FieldPoses.GET_PICK_2.mirror();
    private final Pose swing = new Pose(65, 65).mirror();
    // private final Pose getPick2Back = FieldPoses.GET_PICK_2_BACK;

    // Pickup 3
    private final Pose pickup3 = FieldPoses.PICKUP_3.mirror();
    private final Pose getPick3 = FieldPoses.GET_PICK_3.mirror();

    // Parking
    private final Pose parkPose = FieldPoses.PARK.mirror();
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

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
                .addPath(new BezierLine(getPick1, posGate))
                .setConstantHeadingInterpolation(getPick1.getHeading())
                .addPath(new BezierLine(posGate, openGate))
                .setConstantHeadingInterpolation(getPick1.getHeading())
                .build();

        // path4: Return to Score 1 (Direct)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(openGate, scorePose))
                .setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading())
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
                .addPath(new BezierCurve(getPick2, swing, scorePose))
                .setLinearHeadingInterpolation(getPick2.getHeading(), scorePose.getHeading())
                .build();

        // path9: Score to Pickup 3 Alignment
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3.getHeading())
                .build();

        // path10: Intake Reach 3
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, getPick3))
                .setConstantHeadingInterpolation(getPick3.getHeading())
                .build();

        // path11: Return to Score 3 (Direct)
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(getPick3, scorePose))
                .setLinearHeadingInterpolation(getPick3.getHeading(), scorePose.getHeading())
                .build();

        // path8: Final Park
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate(boolean isBusy, Pose currentPose) {
        switch (pathState) {
            case 0: // Move to Preload Score
                follower.followPath(path1, true);
                follower.setMaxPower(0.85);
                // storageSubsystem.autoThrow = true;
                setPathState(1);
                break;
            case 1: // SHOOTING: Preload
                if (!isBusy) {
                    if (!shootingStarted) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    // Wait until the full shot sequence has completed before moving on
                    if (storageSubsystem.isIdle()) {
                        shootingStarted = false;
                        follower.setMaxPower(1);
                        setPathState(2);
                    }
                }
                break;
            // ================= PICKUP 1 SEQUENCE =================
            case 2: // ALIGN to Pickup 1 (Path 2)
                if (!isBusy) {
                    follower.followPath(path2); // Correctly call alignment path
                    setPathState(3);
                }
                break;
            case 3: // STAB/INTAKE 1 (Path 3)
                if (!isBusy) {
                    follower.setMaxPower(0.9);
                    intakeSubsytem.setPower(1);
                    follower.followPath(path3);
                    setPathState(4);
                }
                break;
            case 4: // RETURN to Score 1 (Path 4)
                if (!isBusy) {
                    follower.setMaxPower(1.0);
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                // storageSubsystem.MoveRelative(475, 1);
                break;
            case 5: // ARRIVED Score 1
                if (!isBusy) {
                    // storageSubsystem.autoThrow = true;
                    setPathState(6);
                }
                // Secure intake during travel
                // if (pathTimer.getElapsedTimeSeconds() < 0.5 && isBusy)
                // storageSubsystem.MoveRelative(475, 1);
                break;
            case 6: // SHOOTING 1
                if (!isBusy) {
                    intakeSubsytem.setPower(0);
                    if (!shootingStarted) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isIdle()) {
                        shootingStarted = false;
                        follower.followPath(path5); // Align to Pickup 2
                        setPathState(7);
                    }
                }
                break;
            // ================= PICKUP 2 SEQUENCE =================
            case 7: // STAB/INTAKE 2 (Path 6)
                if (!isBusy) {
                    follower.setMaxPower(0.9);
                    intakeSubsytem.setPower(1);
                    follower.followPath(path6);
                    setPathState(8);
                }
                break;
            case 8: // RETURN Score 2 (Path 7 - Bezier)
                if (!isBusy) {
                    follower.setMaxPower(1.0);
                    follower.followPath(path7, true);
                    setPathState(9);
                }
                // storageSubsystem.MoveRelative(475, 1);
                break;
            case 9: // ARRIVED Score 2
                if (!isBusy) {
                    // storageSubsystem.autoThrow = true;
                    setPathState(10);
                }
                // if (pathTimer.getElapsedTimeSeconds() < 0.5 && isBusy)
                // storageSubsystem.MoveRelative(475, 1);
                break;
            case 10: // SHOOTING 2
                if (!isBusy) {
                    intakeSubsytem.setPower(0);
                    if (!shootingStarted) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isIdle()) {
                        shootingStarted = false;
                        follower.followPath(path9); // Align to Pickup 3
                        setPathState(11);
                    }
                }
                break;
            // ================= PICKUP 3 SEQUENCE =================
            case 11: // STAB/INTAKE 3 (Path 10)
                if (!isBusy) {
                    follower.setMaxPower(1);
                    intakeSubsytem.setPower(1);
                    follower.followPath(path10);
                    setPathState(12);
                }
                break;
            case 12: // RETURN Score 3 (Path 11)
                if (!isBusy) {
                    follower.setMaxPower(1.0);
                    follower.followPath(path11, true);
                    setPathState(13);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    // storageSubsystem.MoveRelative(475, 1);
                }
                break;
            case 13: // ARRIVED Score 3
                if (!isBusy) {
                    // storageSubsystem.autoThrow = true;
                    setPathState(14);
                }
                // if (pathTimer.getElapsedTimeSeconds() < 0.5 && isBusy)
                // storageSubsystem.MoveRelative(475, 1);
                break;
            case 14: // SHOOTING 3
                if (!isBusy) {
                    intakeSubsytem.setPower(0);
                    if (!shootingStarted) {
                        storageSubsystem.StartShooting();
                        shootingStarted = true;
                    }
                    if (storageSubsystem.isIdle()) {
                        shootingStarted = false;
                        outtakeSubsystem.SetShootMotorPower(0);
                        follower.followPath(path8, true); // Park
                        setPathState(15);
                    }
                }
                break;
            case 15: // PARK COMPLETION
                if (!isBusy) {
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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        PoseStorage.allianceOffset = 0;
        PoseStorage.isRed = true;
        PoseStorage.autoPoseRed = startPose;
        targetPose = LOCK_POSE.mirror();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        matchTimer.reset();
        PoseStorage.isRed = true;
        PoseStorage.allianceOffset = 0;
        setPathState(0);
    }

    @Override
    public void loop() {
        double currentTime = timer.milliseconds();
        loopTime = currentTime - lastTime;
        lastTime = currentTime;

        boolean isBusy = follower.isBusy();
        Pose currentPose = follower.getPose();

        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();

        // Centralized Dynamic Aim (Power & Angle) — always active
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
        outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);

        // --- 30s FAILSAFE GUARDIAN ---
        if (matchTimer.seconds() > 29.8) {
            follower.breakFollowing();
            follower.setMaxPower(0);
            outtakeSubsystem.SetShootMotorPower(0);
            intakeSubsytem.setPower(0);
            PoseStorage.autoPoseRed = currentPose;
            requestOpModeStop();
        }

        autonomousPathUpdate(isBusy, currentPose);
        PoseStorage.autoPoseRed = currentPose;

        /*
         * if (storageSubsystem.recoveryState ==
         * StorageSubsystem.RecoveryState.WAITING_FOR_RETRY
         * || storageSubsystem.recoveryState ==
         * StorageSubsystem.RecoveryState.RETURNING)
         * intakeSubsytem.setPower(1);
         */
        // --- AUTO STATUS ---
        telemetry.addData("Loop Time", "%.2f ms", loopTime);
        telemetry.addData("State", "%d (Time: %.2f s)", pathState, pathTimer.getElapsedTimeSeconds());

        // --- DRIVE / POSITION ---
        telemetry.addData("Drive X", "%.2f", currentPose.getX());
        telemetry.addData("Drive Y", "%.2f", currentPose.getY());
        telemetry.addData("Drive Heading", "%.2f", Math.toDegrees(currentPose.getHeading()));

        // --- SUBSYSTEMS TELEMETRY ---
        intakeSubsytem.displayTelemetry(telemetry);
        storageSubsystem.displayTelemetry(telemetry);
        outtakeSubsystem.displayTelemetry(telemetry);

        telemetry.update();
    }

    @Override
    public void stop() {
        // PoseStorage.autoPoseBlue = follower.getPose();
    }
}
