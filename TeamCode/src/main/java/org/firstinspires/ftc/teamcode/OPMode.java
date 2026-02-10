package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp
public class OPMode extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;
    OuttakeSubsystem outtakeSubsystem;
    ElapsedTime runTime = new ElapsedTime();
    String[] patterns = { " ", "GPP ", "PGP ", "PPG ", " " };
    int lastColorId = 0;
    int outtakeDir = 1;
    boolean foundPattern = false;
    char[] charPattern;
    boolean turretLockEnabled = false;
    boolean chassisLockEnabled = false;
    int targetTagId = 7; // Default tag to track, can be adjusted
    public static double kP_CHASSIS_TURN = -0.035;

    private final Pose SCORE_POSE_RED = new Pose(87.8480, 87.8788, Math.toRadians(40));
    private final Pose SCORE_POSE_BLUE = new Pose(56.15201428571429, 87.87884285714283, Math.toRadians(143));

    public void driveToPose(Pose targetPose) {
        // Build the path using CURRENT position at this exact millisecond
        PathChain dynamicPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(dynamicPath, true);
        automatedDrive = true;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.isRed ? PoseStorage.autoPoseRed : PoseStorage.autoPoseBlue);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        storageSubsystem = new StorageSubsystem(hardwareMap);
        storageSubsystem.InitStorage();

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();

        telemetry.addData("Status", "Initialized");
        storageSubsystem.colorSensingEnabled = true; // Enable color sensing for TeleOp telemetry
        telemetry.update();
    }

    @Override
    public void start() {
        // The parameter controls whether the Follower should use break mode on the
        // motors (using it is recommended).
        // In order to use float mode, add .useBrakeModeInTeleOp(true); to your
        // Drivetrain Constants in Constant.java (for Mecanum)
        // If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
        outtakeSubsystem.SetAngle(OuttakeSubsystem.INITIAL_ANGLE);
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private boolean manual = false;
    boolean reverseIntake = false;

    @Override
    public void loop() {
        double currentTime = timer.milliseconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        // Call Updates once per loop
        follower.update();
        storageSubsystem.update();
        outtakeSubsystem.update();
        telemetryM.update();

        // Handle Inputs
        handleDriverControls();
        handleOperatorControls();

        // Update Auto Systems
        if (storageSubsystem.autoSort)
            storageSubsystem.PatternSortAuto(charPattern);
        else if (!manual && storageSubsystem.autoThrow)
            storageSubsystem.ThrowAll();

        if (turretLockEnabled)
            outtakeSubsystem.updateTurretLock(targetTagId);

        displayTelemetry(loopTime);
    }

    private void handleDriverControls() {
        // Driving Logic
        if (!automatedDrive) {
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (chassisLockEnabled) {
                double errorDegrees = outtakeSubsystem.getTagCenterError(targetTagId);
                if (Math.abs(errorDegrees) > 1)
                    turn = errorDegrees * kP_CHASSIS_TURN;
            }

            if (slowMode) {
                drive *= slowModeMultiplier;
                strafe *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }
            follower.setTeleOpDrive(drive, strafe, turn, true);
        }

        // Feature Toggles
        if (gamepad1.rightBumperWasPressed())
            slowMode = !slowMode;
        if (gamepad1.yWasPressed())
            reverseIntake = !reverseIntake;

        if (gamepad1.xWasPressed()) {
            chassisLockEnabled = !chassisLockEnabled;
            outtakeSubsystem.SetAutoAim(chassisLockEnabled);
        }

        // Intake Power
        if (storageSubsystem.recoveryState == StorageSubsystem.RecoveryState.WAITING_FOR_RETRY
                || storageSubsystem.recoveryState == StorageSubsystem.RecoveryState.RETURNING)
            intakeSubsytem.setPower(1);
        else
            intakeSubsytem.setPower(reverseIntake ? -gamepad1.right_trigger : gamepad1.right_trigger);

        // Automation Controls
        if (gamepad1.aWasPressed() && !automatedDrive)
            driveToPose(PoseStorage.isRed ? SCORE_POSE_RED : SCORE_POSE_BLUE);

        double STICK_THRESHOLD = 0.15;
        boolean driverInput = Math.abs(gamepad1.left_stick_x) > STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > STICK_THRESHOLD ||
                Math.abs(gamepad1.right_stick_x) > STICK_THRESHOLD;

        if (automatedDrive && (driverInput || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Helper Overrides
        if (gamepad1.left_trigger > 0)
            storageSubsystem.MoveRelative(475, 1);
        if (gamepad1.dpadUpWasPressed()) {
            charPattern = patterns[0].toCharArray();
            foundPattern = true;
        }
    }

    private void handleOperatorControls() {
        // Storage & Indexing
        if (gamepad2.dpadDownWasPressed())
            storageSubsystem.ResetStuck();
        if (gamepad2.dpadRightWasPressed())
            storageSubsystem.setServoPos(storageSubsystem.getServoPos() == 1 ? 0.7 : 1);
        if (gamepad2.bWasPressed() && foundPattern) {
            storageSubsystem.checkTimer.reset();
            storageSubsystem.autoSort = true;
        }
        if (gamepad2.xWasPressed()) {
            storageSubsystem.servoTimer.reset();
            storageSubsystem.autoThrow = true;
        }
        if (gamepad2.dpadUpWasPressed())
            storageSubsystem.MoveRelative(475, 1);

        // Manual Storage Override
        if (gamepad2.left_trigger > 0) {
            if (storageSubsystem.autoThrow)
                storageSubsystem.Abort();
            storageSubsystem.ManualMove(gamepad2.right_stick_x * 0.4);
            manual = true;
        } else if (manual) {
            storageSubsystem.RestoreAuto();
            manual = false;
        }

        // Outtake & Turret
        if (gamepad2.aWasPressed())
            outtakeSubsystem.ToggleShootMotor();
        if (gamepad2.yWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
            outtakeSubsystem.SetAutoAim(turretLockEnabled);
            if (!turretLockEnabled)
                outtakeSubsystem.resetTurret();
        }

        // Outtake Servo Angle
        if (!storageSubsystem.autoSort && !storageSubsystem.autoThrow) {
            if (gamepad2.leftBumperWasReleased())
                outtakeSubsystem.IncreaseAngle();
            else if (gamepad2.rightBumperWasReleased())
                outtakeSubsystem.DecreaseAngle();
        }
    }

    private void displayTelemetry(double loopTime) {
        Pose currentPose = follower.getPose();
        // --- GAMEPAD 1: DRIVER ---
        telemetry.addLine("=== GAMEPAD 1: DRIVER ===");
        telemetry.addData("> Drive Mode", slowMode ? "SLOW (x" + slowModeMultiplier + ")" : "NORMAL");
        telemetry.addData("> Chassis Lock", chassisLockEnabled ? "ACTIVE (Tag: " + targetTagId + ")" : "OFF");
        intakeSubsytem.displayTelemetry(telemetry);
        telemetry.addData("> Drive Pos", "X:%.1f Y:%.1f H:%.1f", currentPose.getX(), currentPose.getY(),
                Math.toDegrees(currentPose.getHeading()));

        // --- GAMEPAD 2: OPERATOR ---
        telemetry.addLine("\n=== GAMEPAD 2: OPERATOR ===");
        telemetry.addData("> Turret Lock", turretLockEnabled ? "ACTIVE (Target: " + targetTagId + ")" : "OFF");
        outtakeSubsystem.displayTelemetry(telemetry);
        storageSubsystem.displayTelemetry(telemetry);

        // --- SENSORS & PATTERNS ---
        telemetry.addLine("\n=== SENSORS & LOGIC ===");
        telemetry.addData("Pattern", patterns[lastColorId] + " (ID: " + lastColorId + ")");
        if (foundPattern)
            telemetry.addData("Active Char", charPattern);
        telemetry.addData("Loop Time", loopTime + "ms");
        telemetry.addData("Runtime", "%.1f s", runTime.seconds());

        telemetry.update();
        telemetryM.debug("position", currentPose);
        telemetryM.debug("velocity", follower.getVelocity());
    }
}