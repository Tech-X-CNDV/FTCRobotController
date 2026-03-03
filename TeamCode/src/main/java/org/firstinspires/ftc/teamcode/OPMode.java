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
import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;
import org.firstinspires.ftc.teamcode.config.FieldPoses;

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
        // targetTagId = PoseStorage.isRed ? 2 : 1;
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        storageSubsystem = new StorageSubsystem(hardwareMap);
        // storageSubsystem.InitStorage();

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();

        telemetry.addData("Status", "Initialized");
        // storageSubsystem.colorSensingEnabled = false; // Enable color sensing for
        // TeleOp telemetry
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
        targetPose = PoseStorage.isRed ? LOCK_POSE.mirror() : LOCK_POSE;
        outtakeSubsystem.SetAngle(0);
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private boolean manual = false;
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

        // Call Updates once per loop
        follower.update();
        // storageSubsystem.update();
        outtakeSubsystem.update();
        telemetryM.update();

        // Handle Inputs
        handleDriverControls();
        handleOperatorControls();

        // Update Auto Systems
        /*
         * if (storageSubsystem.autoSort)
         * storageSubsystem.PatternSortAuto(charPattern);
         * else if (!manual && storageSubsystem.autoThrow)
         * storageSubsystem.ThrowAll(0.32);
         */

        if (turretLockEnabled)
            outtakeSubsystem.updateTurretLock(follower.getPose(), targetPose, manualTurretOffset);

        if (currentTime > telemetryTimer + 100) {
            displayTelemetry(loopTime);
            telemetryTimer = currentTime;
        }
    }

    private void handleDriverControls() {
        // 1. INPUT GATHERING
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // 2. FEATURE TOGGLES & RESET LOGIC
        if (gamepad1.rightBumperWasPressed())
            slowMode = !slowMode;
        if (gamepad1.yWasPressed())
            reverseIntake = !reverseIntake;
        if (gamepad1.aWasPressed() && !automatedDrive)
            driveToPose(PoseStorage.isRed ? SCORE_POSE_RED : SCORE_POSE_BLUE);

        // CHASSIS LOCK TOGGLE
        if (gamepad1.xWasPressed()) {
            chassisLockEnabled = !chassisLockEnabled;
            if (chassisLockEnabled)
                smallBasketLockEnabled = false;
            outtakeSubsystem.SetAutoAim(chassisLockEnabled || smallBasketLockEnabled);

            if (chassisLockEnabled) {
                targetPose = PoseStorage.isRed ? LOCK_POSE.mirror() : LOCK_POSE;
                gamepad1.rumble(300); // Confirmation buzz
            } else if (!smallBasketLockEnabled) {
                // SYNC HEADING on release to prevent the "Joystick Jump"
                Pose current = follower.getPose();
                follower.setPose(new Pose(current.getX(), current.getY(), current.getHeading()));
            }
        }

        // SMALL BASKET LOCK TOGGLE
        if (gamepad1.bWasPressed()) {
            smallBasketLockEnabled = !smallBasketLockEnabled;
            if (smallBasketLockEnabled)
                chassisLockEnabled = false;
            outtakeSubsystem.SetAutoAim(chassisLockEnabled || smallBasketLockEnabled);

            if (smallBasketLockEnabled) {
                targetPose = PoseStorage.isRed ? LOW_BASKET_POSE.mirror() : LOW_BASKET_POSE;
                gamepad1.rumble(300); // Confirmation buzz
            } else if (!chassisLockEnabled) {
                // SYNC HEADING on release to prevent the "Joystick Jump"
                Pose current = follower.getPose();
                follower.setPose(new Pose(current.getX(), current.getY(), current.getHeading()));
            }
        }

        // EMERGENCY FIELD-CENTRIC RESET (Start Button)
        if (gamepad1.startWasPressed()) {
            Pose currentPose = follower.getPose();
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), PoseStorage.allianceOffset));
            gamepad1.rumbleBlips(2);
        }

        // SNAP TO DRIVER FORWARD (D-Pad Up)
        /*
         * Doesn't work rn
         * if (gamepad1.dpadUpWasPressed()) {
         * follower.holdPoint(
         * new Pose(follower.getPose().getX(), follower.getPose().getY(),
         * PoseStorage.allianceOffset), true);
         * }
         */

        // 3. DRIVING & AIM LOGIC
        if (!automatedDrive) {
            if (slowMode) {
                drive *= slowModeMultiplier;
                strafe *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }

            // SHARED DYNAMIC AIM (Triggers for Turret or Chassis Lock)
            if (chassisLockEnabled || smallBasketLockEnabled || turretLockEnabled) {
                double deltaX = targetPose.getX() - follower.getPose().getX();
                double deltaY = targetPose.getY() - follower.getPose().getY();

                if (smallBasketLockEnabled) {
                    outtakeSubsystem.updateFixedPower(0.75);
                    outtakeSubsystem.SetAngle(1);
                } else {
                    outtakeSubsystem.updateAutoAimPower(deltaX, deltaY);
                    outtakeSubsystem.updateAutoAimAngle(deltaX, deltaY);
                }

                // Specific Chassis Lock Driving
                if (chassisLockEnabled || smallBasketLockEnabled) {
                    if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                        manualHeadingOffset -= gamepad1.right_stick_x * 0.015; // Tunable sensitivity
                    }

                    double angleToScore = Math.atan2(deltaY, deltaX) + Math.toRadians(5) + manualHeadingOffset;

                    // Calculate Heading Error
                    double currentHeading = follower.getPose().getHeading();
                    double headingError = angleToScore - currentHeading;

                    // Normalize the error so the robot takes the shortest path
                    while (headingError > Math.PI)
                        headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI)
                        headingError += 2 * Math.PI;

                    // Force the Power (Manual P-Loop)
                    double autoTurnPower = headingError * 1.0;

                    // "POWER STEERING" LOCK:
                    follower.setTeleOpDrive(drive, strafe, autoTurnPower, false, angleToScore);
                } else {
                    // Manual Driving but with Dynamic Aim active (Turret Lock is on)
                    follower.setTeleOpDrive(drive, strafe, turn, false, PoseStorage.allianceOffset);
                }

            } else {
                // STANDARD FIELD-CENTRIC (No Locks Active)
                follower.setTeleOpDrive(drive, strafe, turn, false, PoseStorage.allianceOffset);
            }
        }

        // 4. SUBSYSTEMS (Intake & Recovery Logic)
        /*
         * if (storageSubsystem.recoveryState ==
         * StorageSubsystem.RecoveryState.WAITING_FOR_RETRY
         * || storageSubsystem.recoveryState ==
         * StorageSubsystem.RecoveryState.RETURNING) {
         * intakeSubsytem.setPower(1);
         * } else {
         * intakeSubsytem.setPower(reverseIntake ? -gamepad1.right_trigger :
         * gamepad1.right_trigger);
         * }
         */
        intakeSubsytem.setPower(reverseIntake ? -gamepad1.right_trigger : gamepad1.right_trigger);

        /*
         * if (gamepad1.left_trigger_pressed) {
         * storageSubsystem.MoveRelative(475, 1);
         * }
         */

        if (gamepad1.dpadUpWasPressed()) {
            charPattern = patterns[0].toCharArray();
            foundPattern = true;
        }

        // 5. AUTOMATION INTERRUPT (Safety)
        double STICK_THRESHOLD = 0.15;
        boolean driverInput = Math.abs(gamepad1.left_stick_x) > STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > STICK_THRESHOLD ||
                Math.abs(gamepad1.right_stick_x) > STICK_THRESHOLD;

        if (automatedDrive && (driverInput || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    private void handleOperatorControls() {
        // Storage & Indexing
        /*
         * if (gamepad2.dpadDownWasPressed())
         * storageSubsystem.ResetStuck();
         * if (gamepad2.dpadRightWasPressed())
         * storageSubsystem.setServoPos(storageSubsystem.getServoPos() > 0.8 ? 0.65 :
         * 0.97);
         */
        /*
         * Not used at the moment
         * if (gamepad2.bWasPressed() && foundPattern) {
         * storageSubsystem.checkTimer.reset();
         * storageSubsystem.autoSort = true;
         * }
         */
        /*
         * if (gamepad2.xWasPressed()) {
         * storageSubsystem.servoTimer.reset();
         * storageSubsystem.autoThrow = true;
         * }
         * if (gamepad2.dpadUpWasPressed())
         * storageSubsystem.MoveRelative(475, 1);
         */

        // Manual Storage Override
        if (gamepad2.left_trigger_pressed) {
            // storageSubsystem.Abort(); // Master Override: Stops everything (Recovery,
            // Auto-Throw, Auto-Sort)
            // storageSubsystem.ManualMove(gamepad2.right_stick_x * 0.4);
            manual = true;
        } else {
            if (manual) {
                // storageSubsystem.RestoreAuto();
                manual = false;
            }
            if (turretLockEnabled && Math.abs(gamepad2.right_stick_x) > 0.1) {
                manualTurretOffset -= gamepad2.right_stick_x * 0.01; // Tunable sensitivity
            }
        }

        // Outtake & Turret
        if (gamepad2.aWasPressed())
            outtakeSubsystem.ToggleShootMotor();
        if (gamepad2.yWasPressed()) {
            turretLockEnabled = !turretLockEnabled;
            if (!turretLockEnabled)
                outtakeSubsystem.resetTurret();
            else
                gamepad2.rumble(300);
        }
        if (outtakeSubsystem.isReadyToFire())
            gamepad2.rumble(100);

        // Outtake Servo Angle
        // if (!storageSubsystem.autoSort && !storageSubsystem.autoThrow) {
        if (true) { // Storage retired
            if (gamepad2.leftBumperWasReleased()) {
                if (chassisLockEnabled || smallBasketLockEnabled) {
                    outtakeSubsystem.IncreaseAngleOffset();
                } else {
                    outtakeSubsystem.IncreaseDirectAngle();
                }
            } else if (gamepad2.rightBumperWasReleased()) {
                if (chassisLockEnabled || smallBasketLockEnabled) {
                    outtakeSubsystem.DecreaseAngleOffset();
                } else {
                    outtakeSubsystem.DecreaseDirectAngle();
                }
            }
        }

        // Manual Shooter Power Offset (Persistent)
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            double currentOffset = outtakeSubsystem.getManualPowerOffset();
            // Up = Positive Power Boost (gamepad Y is reversed)
            outtakeSubsystem.setManualPowerOffset(currentOffset - (gamepad2.left_stick_y * 0.002));
        }
    }

    private void displayTelemetry(double loopTime) {
        Pose currentPose = follower.getPose();
        // --- GAMEPAD 1: DRIVER ---
        telemetry.addLine("=== GAMEPAD 1: DRIVER ===");
        telemetry.addData("> Drive Mode", slowMode ? "SLOW (x" + slowModeMultiplier + ")" : "NORMAL");
        telemetry.addData("> Chassis Lock", chassisLockEnabled ? "ACTIVE (Low Basket: OFF)"
                : (smallBasketLockEnabled ? "ACTIVE (Low Basket: ON)" : "OFF"));
        telemetry.addData("> Lock Offset", "%.1f°", Math.toDegrees(manualHeadingOffset));
        intakeSubsytem.displayTelemetry(telemetry);
        telemetry.addData("> Drive Pos", "X:%.1f Y:%.1f H:%.1f", currentPose.getX(), currentPose.getY(),
                Math.toDegrees(currentPose.getHeading()));

        // --- GAMEPAD 2: OPERATOR ---
        telemetry.addLine("\n=== GAMEPAD 2: OPERATOR ===");
        telemetry.addData("> Turret Lock", turretLockEnabled ? "ACTIVE" : "OFF");
        outtakeSubsystem.displayTelemetry(telemetry);
        telemetry.addData("> Power Offset", "%.3f (L-Stick Y)", outtakeSubsystem.getManualPowerOffset());
        telemetry.addData("> Angle Offset", "%.3f (Bumpers)", outtakeSubsystem.getManualAngleOffset());
        telemetry.addData("> Turret Offset", "%.1f°", Math.toDegrees(manualTurretOffset));
        // storageSubsystem.displayTelemetry(telemetry);

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