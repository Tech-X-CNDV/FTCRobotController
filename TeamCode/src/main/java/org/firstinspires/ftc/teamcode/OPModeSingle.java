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
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.PoseStorage;
import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.FieldPoses;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "OPMode Single Gamepad", group = "TeleOp")
public class OPModeSingle extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    StorageSubsystem storageSubsystem;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    ElapsedTime runTime = new ElapsedTime();
    String[] patterns = { " ", "GPP ", "PGP ", "PPG ", " " };
    int lastColorId = 0;
    boolean foundPattern = false;
    char[] charPattern;
    boolean turretLockEnabled = false;
    boolean chassisLockEnabled = false;
    boolean smallBasketLockEnabled = false;
    private double manualHeadingOffset = 0;
    private double manualTurretOffset = 0;

    private final Pose SCORE_POSE_RED = FieldPoses.SCORE.mirror();
    private final Pose SCORE_POSE_BLUE = FieldPoses.SCORE;
    private final Pose LOCK_POSE = FieldPoses.LOCK_POSE;
    private final Pose LOW_BASKET_POSE = FieldPoses.LOW_BASKET_POSE;
    Pose targetPose;

    private List<LynxModule> allHubs;

    public void driveToPose(Pose targetPose) {
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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeSubsystem.InitIntake();

        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();
        storageSubsystem = new StorageSubsystem(hardwareMap, outtakeSubsystem);
        storageSubsystem.InitStorage();

        telemetry.addData("Status", "Initialized (Single Gamepad Mode)");
        telemetry.update();
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
        follower.startTeleopDrive(true);
        targetPose = PoseStorage.isRed ? LOCK_POSE.mirror() : LOCK_POSE;
        outtakeSubsystem.SetAngle(0.15); // Safe compact starting position (avoids mechanical bottoming/binding)
        outtakeSubsystem.resetTurret();
        outtakeSubsystem.StartShootMotor();
        outtakeSubsystem.setTargetVelocity(OuttakeSubsystem.IDLE_SHOOT_VELOCITY);
        storageSubsystem.ResetToIntake(true);
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    boolean reverseIntake = false;
    double loopTime;
    private double telemetryTimer = 0;

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
        telemetryM.update();

        handleControls();

        if (turretLockEnabled)
            outtakeSubsystem.updateTurretLock(follower.getPose(), targetPose, manualTurretOffset);

        if (currentTime > telemetryTimer + 100) {
            displayTelemetry(loopTime);
            telemetryTimer = currentTime;
        }
    }

    private void handleControls() {
        // 1. DRIVING INPUTS
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // 2. MODIFIER (Shift Key) - Using Back Button for manual overrides
        boolean isShift = gamepad1.back; 

        // 3. TOGGLES & AUTOMATION
        
        // Slow Mode Toggle (L3 - Left Stick Button)
        if (gamepad1.leftStickButtonWasPressed()) slowMode = !slowMode;
        
        // Field Centric Reset (R3 - Right Stick Button)
        if (gamepad1.rightStickButtonWasPressed()) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), PoseStorage.allianceOffset));
            gamepad1.rumbleBlips(2);
        }

        // Intake Reverse Toggle (Y)
        if (gamepad1.yWasPressed()) reverseIntake = !reverseIntake;

        // Drive to Pose (A)
        if (gamepad1.aWasPressed() && !automatedDrive)
            driveToPose(PoseStorage.isRed ? SCORE_POSE_RED : SCORE_POSE_BLUE);

        // LOCKS (Using D-Pad)
        
        // Chassis Lock Toggle (D-Pad Left)
        if (gamepad1.dpadLeftWasPressed()) {
            chassisLockEnabled = !chassisLockEnabled;
            if (chassisLockEnabled) smallBasketLockEnabled = false;
            outtakeSubsystem.SetAutoAim(chassisLockEnabled || smallBasketLockEnabled);

            if (chassisLockEnabled) {
                targetPose = PoseStorage.isRed ? LOCK_POSE.mirror() : LOCK_POSE;
                gamepad1.rumble(300);
            } else if (!smallBasketLockEnabled) {
                Pose current = follower.getPose();
                follower.setPose(new Pose(current.getX(), current.getY(), current.getHeading()));
            }
        }

        // Small Basket Lock Toggle (D-Pad Right)
        if (gamepad1.dpadRightWasPressed()) {
            smallBasketLockEnabled = !smallBasketLockEnabled;
            if (smallBasketLockEnabled) chassisLockEnabled = false;
            outtakeSubsystem.SetAutoAim(chassisLockEnabled || smallBasketLockEnabled);

            if (smallBasketLockEnabled) {
                targetPose = PoseStorage.isRed ? LOW_BASKET_POSE.mirror() : LOW_BASKET_POSE;
                gamepad1.rumble(300);
            } else if (!chassisLockEnabled) {
                Pose current = follower.getPose();
                follower.setPose(new Pose(current.getX(), current.getY(), current.getHeading()));
            }
        }

        // Turret Lock Toggle (D-Pad Up)
        if (gamepad1.dpadUpWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
            if (!turretLockEnabled) outtakeSubsystem.resetTurret();
            else gamepad1.rumble(300);
        }

        // Gate Toggle (D-Pad Down)
        if (gamepad1.dpadDownWasPressed()) {
            if (storageSubsystem.isGateOpen()) storageSubsystem.CloseGate();
            else storageSubsystem.OpenGate();
        }

        // 4. SHOOTING & STORAGE
        
        // Start Shooting Sequence (X)
        if (gamepad1.xWasPressed()) {
            outtakeSubsystem.StartShootMotor();
            storageSubsystem.StartShooting();
        }

        // Toggle Shoot Motor (B)
        if (gamepad1.bWasPressed()) {
            outtakeSubsystem.ToggleShootMotor();
        }

        // Reset Storage to Intake (LT) or Manual Abort (LT + Stick)
        if (gamepad1.left_trigger_pressed) {
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                outtakeSubsystem.SetShootMotorPower(0);
                storageSubsystem.Abort(gamepad1.left_stick_x);
            } else {
                storageSubsystem.ResetToIntake(true);
            }
        }

        // Intake Power (Right Trigger)
        intakeSubsystem.setPower(reverseIntake ? -gamepad1.right_trigger : gamepad1.right_trigger);

        // 5. MANUAL ADJUSTMENTS (Bumpers & Shift)
        
        // Outtake Servo Angle (Bumpers)
        if (gamepad1.leftBumperWasReleased()) {
            if (chassisLockEnabled || smallBasketLockEnabled || turretLockEnabled) outtakeSubsystem.IncreaseAngleOffset();
            else outtakeSubsystem.IncreaseDirectAngle();
        } else if (gamepad1.rightBumperWasReleased()) {
            if (chassisLockEnabled || smallBasketLockEnabled || turretLockEnabled) outtakeSubsystem.DecreaseAngleOffset();
            else outtakeSubsystem.DecreaseDirectAngle();
        }

        // Manual Offsets (Using Shift + Sticks)
        if (isShift) {
            // Manual storage movement override
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                outtakeSubsystem.SetShootMotorPower(0);
                storageSubsystem.Abort(gamepad1.left_stick_x);
            } else if (storageSubsystem.getState().equals("MANUAL")) {
                storageSubsystem.Abort(0);
            }
            
            // Manual shooter velocity offset
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                double currentOffset = outtakeSubsystem.getManualVelocityOffset();
                outtakeSubsystem.setManualVelocityOffset(currentOffset - (gamepad1.left_stick_y * 5.0));
            }

            // Manual turret offset
            if (turretLockEnabled && Math.abs(gamepad1.right_stick_x) > 0.1) {
                manualTurretOffset -= gamepad1.right_stick_x * 0.01;
            }
            
            // CRITICAL: Disable all driving immediately
            drive = 0;
            strafe = 0;
            turn = 0;
            follower.setTeleOpDrive(0,0,0); 
        }

        // 6. DRIVING & AIM LOGIC
        if (!automatedDrive) {
            if (slowMode) {
                drive *= slowModeMultiplier;
                strafe *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }

            if (chassisLockEnabled || smallBasketLockEnabled || turretLockEnabled) {
                double deltaX = targetPose.getX() - follower.getPose().getX();
                double deltaY = targetPose.getY() - follower.getPose().getY();

                if (smallBasketLockEnabled) {
                    outtakeSubsystem.updateFixedPower(0.75);
                    outtakeSubsystem.SetAngle(Math.max(0, Math.min(1.0, 1.0 + outtakeSubsystem.getManualAngleOffset())));
                } else {
                    outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
                    outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);
                }

                if ((chassisLockEnabled || smallBasketLockEnabled) && !isShift) {
                    if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                        manualHeadingOffset -= gamepad1.right_stick_x * 0.015;
                    }

                    double angleToScore = Math.atan2(deltaY, deltaX) + Math.toRadians(5) + manualHeadingOffset;
                    double currentHeading = follower.getPose().getHeading();
                    double headingError = angleToScore - currentHeading;

                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    double autoTurnPower = headingError * 1.0;
                    follower.setTeleOpDrive(drive, strafe, autoTurnPower, false, angleToScore);
                } else {
                    follower.setTeleOpDrive(drive, strafe, turn, false, PoseStorage.allianceOffset);
                }
            } else {
                follower.setTeleOpDrive(drive, strafe, turn, false, PoseStorage.allianceOffset);
                if (storageSubsystem.isIdle()) {
                    outtakeSubsystem.setTargetVelocity(OuttakeSubsystem.IDLE_SHOOT_VELOCITY);
                } else {
                    // Dynamically scale flywheel velocity based on distance to targetPose when shooting in manual
                    double deltaX = targetPose.getX() - follower.getPose().getX();
                    double deltaY = targetPose.getY() - follower.getPose().getY();
                    outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
                }
                outtakeSubsystem.SetAngle(OuttakeSubsystem.INITIAL_ANGLE);
            }
        }

        // 7. AUTOMATION INTERRUPT
        double STICK_THRESHOLD = 0.15;
        boolean driverInput = Math.abs(gamepad1.left_stick_x) > STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > STICK_THRESHOLD ||
                Math.abs(gamepad1.right_stick_x) > STICK_THRESHOLD;

        if (automatedDrive && (driverInput || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    private void displayTelemetry(double loopTime) {
        Pose currentPose = follower.getPose();
        telemetry.addLine("=== SINGLE PILOT CONTROLS ===");
        telemetry.addData("> Drive Mode", slowMode ? "SLOW (x" + slowModeMultiplier + ")" : "NORMAL");
        telemetry.addData("> Chassis Lock", chassisLockEnabled ? "ACTIVE" : "OFF");
        telemetry.addData("> Basket Lock", smallBasketLockEnabled ? "ACTIVE" : "OFF");
        telemetry.addData("> Turret Lock", turretLockEnabled ? "ACTIVE" : "OFF");
        telemetry.addData("> Angle Offset", outtakeSubsystem.getManualAngleOffset());
        telemetry.addData("> Drive Pos", "X:%.1f Y:%.1f H:%.1f", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        
        telemetry.addLine("\n=== SUBSYSTEMS ===");
        intakeSubsystem.displayTelemetry(telemetry);
        outtakeSubsystem.displayTelemetry(telemetry);
        storageSubsystem.displayTelemetry(telemetry);

        telemetry.addLine("\n=== SYSTEM ===");
        telemetry.addData("Loop Time", loopTime + "ms");
        telemetry.addData("Runtime", "%.1f s", runTime.seconds());

        telemetry.update();
        telemetryM.debug("position", currentPose);
        telemetryM.debug("velocity", follower.getVelocity());
    }
}
