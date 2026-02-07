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
    int targetTagId = 7; // Default tag to track, can be adjusted

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
        outtakeSubsystem.SetAngle(0.9);
    }

    private com.qualcomm.robotcore.util.ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();
    private double lastTime = 0;
    private boolean manual = false;
    boolean reverseIntake = false;

    @Override
    public void loop() {
        double currentTime = timer.milliseconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        // Store frequently accessed properties
        Pose currentPose = follower.getPose();

        // Call this once per loop
        follower.update();
        storageSubsystem.update();
        telemetryM.update();

        if (!automatedDrive) {
            // Make the last parameter false for field-centric
            // In case the drivers want to use a "slowMode" you can scale the vectors

            // This is the normal version to use in the TeleOp
            if (!slowMode)
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );

            // This is how it looks with slowMode on
            else
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true // Robot Centric
                );
        }

        // Automated PathFollowing
        if (gamepad1.aWasPressed() && !automatedDrive) {
            driveToPose(PoseStorage.isRed ? SCORE_POSE_RED : SCORE_POSE_BLUE);
        }

        double STICK_THRESHOLD = 0.15;

        boolean driverInput = Math.abs(gamepad1.left_stick_x) > STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > STICK_THRESHOLD ||
                Math.abs(gamepad1.right_stick_x) > STICK_THRESHOLD;

        // Stop automated following if the follower is done
        if (automatedDrive && (driverInput || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if(storageSubsystem.recoveryState == StorageSubsystem.RecoveryState.WAITING_FOR_RETRY || storageSubsystem.recoveryState == StorageSubsystem.RecoveryState.RETURNING)
            intakeSubsytem.setPower(1);
        else
            intakeSubsytem.setPower(reverseIntake ? -gamepad1.right_trigger : gamepad1.right_trigger);

        // outtakeSubsystem.OuttakeMotorControl(gamepad2.right_stick_x);
        if (gamepad2.dpadDownWasPressed())
            storageSubsystem.ResetStuck();
        if (gamepad2.dpadRightWasPressed())
            storageSubsystem.setServoPos(storageSubsystem.getServoPos() == 1 ? 0.7 : 1);
        if (gamepad2.aWasPressed())
            outtakeSubsystem.ToggleShootMotor();

        if (gamepad2.yWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
            if (!turretLockEnabled)
                outtakeSubsystem.resetTurret();
        }

        if (gamepad2.left_trigger > 0) {
            if (storageSubsystem.autoThrow)
                storageSubsystem.Abort();
            storageSubsystem.ManualMove(gamepad2.right_stick_x * 0.4);
            manual = true;
        } else if (manual) {
            storageSubsystem.RestoreAuto();
            manual = false;
        }

        if (gamepad1.yWasPressed())
            reverseIntake = !reverseIntake;


        if (storageSubsystem.autoSort)
            storageSubsystem.PatternSortAuto(charPattern);
        else if (!manual && storageSubsystem.autoThrow)
            storageSubsystem.ThrowAll();
        else {
            if (gamepad2.leftBumperWasReleased())
                outtakeSubsystem.IncreaseAngle();
            else if (gamepad2.rightBumperWasReleased())
                outtakeSubsystem.DecreaseAngle();
            if (gamepad2.bWasPressed() && foundPattern) {
                storageSubsystem.checkTimer.reset();
                storageSubsystem.autoSort = true;
            }
            if (gamepad2.xWasPressed()) {
                // outtakeSubsystem.ToggleShootMotor();
                storageSubsystem.servoTimer.reset();
                storageSubsystem.autoThrow = true;
            }
            if (gamepad2.dpadUpWasPressed())
                storageSubsystem.MoveRelative(475, 1);

            if (gamepad1.left_trigger > 0)
                storageSubsystem.MoveRelative(475, 1);
        }

        if (turretLockEnabled) {
            outtakeSubsystem.updateTurretLock(targetTagId);
        }
        // if (lastColorId == 0 && !foundPattern) {
        // outtakeSubsystem.InitVision(); // Lazy init HuskyLens
        // blocks = outtakeSubsystem.GetCameraFeed();
        // if (blocks.length > 0 && blocks[0].id == 21) {
        // lastColorId = blocks[0].id;
        // charPattern = patterns[lastColorId].toCharArray();
        // foundPattern = true;
        // }
        // }

        if (gamepad1.dpadUpWasPressed()) {
            charPattern = patterns[0].toCharArray();
            foundPattern = true;
        }
        // --- SYSTEM STATUS ---
        telemetry.addData(">> MODE", slowMode ? "SLOW (Multiplier: " + slowModeMultiplier + ")" : "NORMAL");
        telemetry.addData(">> AUTOMATED : isBusy?", automatedDrive + " : " + follower.isBusy());
        telemetry.addData(">> LOOP TIME (ms)", loopTime);

        // --- DRIVE / POSITION ---
        telemetry.addData("Drive X", "%.2f", currentPose.getX());
        telemetry.addData("Drive Y", "%.2f", currentPose.getY());
        telemetry.addData("Drive Heading", "%.2f", Math.toDegrees(currentPose.getHeading()));

        // --- INTAKE ---
        telemetry.addData("Intake Power", (reverseIntake ? -gamepad1.right_trigger : gamepad1.right_trigger));
        telemetry.addData("Intake Reverse", reverseIntake);

        // --- STORAGE ---
        telemetry.addData("Storage Status",
                storageSubsystem.isStuck ? "STUCK (" + storageSubsystem.recoveryState + ")" : "OK");
        telemetry.addData("StorageVelocity", storageSubsystem.ReturnVelocity());
        telemetry.addData("Storage AutoSort", storageSubsystem.autoSort);
        telemetry.addData("Storage AutoThrow", storageSubsystem.autoThrow);
        telemetry.addData("Storage Target Progress",
                storageSubsystem.getTurns() + "/3 (" + storageSubsystem.getPos() + ")");
        telemetry.addData("Storage Pos", storageSubsystem.getPosition());
        telemetry.addData("Shooter Servo", storageSubsystem.getServoPos());

        // --- OUTTAKE ---
        telemetry.addData("Outtake Pos", outtakeSubsystem.getOuttakeMotorPosition());
        telemetry.addData("Turret Lock", turretLockEnabled ? "ACTIVE (Target: " + targetTagId + ")" : "OFF");
        telemetry.addData("Turret Target", outtakeSubsystem.getTurretTargetPos());
        telemetry.addData("Outtake Dir", outtakeDir);

        // --- SENSORS & PATTERN ---
        if (storageSubsystem.idenColor() == 'G')
            telemetry.addData("Detected Artifact", "GREEN");
        else if (storageSubsystem.idenColor() == 'P')
            telemetry.addData("Detected Artifact", "PURPLE");
        else
            telemetry.addData("Detected Artifact", "NONE");

        telemetry.addData("Hue/Sat", "%.2f / %.2f", storageSubsystem.hue, storageSubsystem.sat);
        telemetry.addData("Pattern", patterns[lastColorId] + " (ID: " + lastColorId + ")");
        if (foundPattern)
            telemetry.addData("Active Pattern Char", String.valueOf(charPattern));

        // --- DEBUG / MISC ---
        telemetry.addData("Runtime", "%.1f s", runTime.seconds());
        telemetryM.debug("position", currentPose);
        telemetryM.debug("velocity", follower.getVelocity());
    }
}