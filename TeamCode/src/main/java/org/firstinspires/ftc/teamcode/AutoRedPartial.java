package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoRedPartial")
public class AutoRedPartial extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    OuttakeSubsystem outtakeSubsystem;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;
    private boolean dynamicAimStarted = false;
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
    private final Pose getPick2Back = FieldPoses.GET_PICK_2_BACK.mirror();

    // Parking
    private final Pose parkPose = FieldPoses.PARK.mirror();
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

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
                .addPath(new BezierLine(getPick1, scorePose))
                .setLinearHeadingInterpolation(getPick1.getHeading(), scorePose.getHeading())
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
                .addPath(new BezierCurve(getPick2, new Pose(79, 65), scorePose))
                .setLinearHeadingInterpolation(getPick2.getHeading(), scorePose.getHeading())
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
                outtakeSubsystem.AutoAngle();
                outtakeSubsystem.SetShootMotorPower(1.0);
                follower.setMaxPower(0.8);
                storageSubsystem.autoThrow = true;
                setPathState(1);
                break;

            case 1: // SHOOTING: Preload
                if (!isBusy) {
                    dynamicAimStarted = true;
                    if (storageSubsystem.autoThrow) {
                        storageSubsystem.ThrowAll();
                    } else {
                        storageSubsystem.setServoPos(1);
                        follower.setMaxPower(1);
                        setPathState(2); // Move to Alignment
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
                    follower.setMaxPower(0.8);
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
                // Delayed Conveyor during stab
                // if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                storageSubsystem.MoveRelative(475, 1);
                // }
                break;

            case 5: // ARRIVED Score 1
                if (!isBusy) {
                    storageSubsystem.autoThrow = true;
                    setPathState(6);
                }
                // Secure intake during travel
                if (pathTimer.getElapsedTimeSeconds() < 0.5 && isBusy)
                    storageSubsystem.MoveRelative(475, 1);
                break;

            case 6: // SHOOTING 1
                if (!isBusy) {
                    if (storageSubsystem.autoThrow) {
                        storageSubsystem.ThrowAll();
                    } else {
                        intakeSubsytem.setPower(0);
                        storageSubsystem.setServoPos(1);
                        follower.followPath(path5); // Align to Pickup 2
                        setPathState(7);
                    }
                }
                break;

            // ================= PICKUP 2 SEQUENCE =================
            case 7: // STAB/INTAKE 2 (Path 6)
                if (!isBusy) {
                    follower.setMaxPower(0.8);
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
                // if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                storageSubsystem.MoveRelative(475, 1);
                // }
                break;

            case 9: // ARRIVED Score 2
                if (!isBusy) {
                    storageSubsystem.autoThrow = true;
                    setPathState(10);
                }
                if (pathTimer.getElapsedTimeSeconds() < 0.5 && isBusy)
                    storageSubsystem.MoveRelative(475, 1);
                break;

            case 10: // SHOOTING 2
                if (!isBusy) {
                    if (storageSubsystem.autoThrow) {
                        storageSubsystem.ThrowAll();
                    } else {
                        intakeSubsytem.setPower(0);
                        outtakeSubsystem.SetShootMotorPower(0);
                        storageSubsystem.setServoPos(1);
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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        storageSubsystem.InitStorage();

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        targetPose = LOCK_POSE;
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        matchTimer.reset();
        outtakeSubsystem.AutoAngle();
        outtakeSubsystem.SetShootMotorPower(0.75);
        dynamicAimStarted = false;
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

        if (dynamicAimStarted) {
            // Calculate Distance to Bucket (LOCK_POSE mirror for Red)
            double deltaX = targetPose.getX() - currentPose.getX();
            double deltaY = targetPose.getY() - currentPose.getY();

            outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
            outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);
        } else {
            // High-power spin-up while traveling to first position
            outtakeSubsystem.SetShootMotorPower(0.72);
            outtakeSubsystem.AutoAngle();
        }

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
        PoseStorage.autoPoseRed = follower.getPose();
    }
}
