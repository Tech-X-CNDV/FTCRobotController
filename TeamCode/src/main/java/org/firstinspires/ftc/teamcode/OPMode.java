package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystem.IntakeSubsytem;
import org.firstinspires.ftc.teamcode.config.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.StorageSubsystem;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class OPMode extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    StorageSubsystem storageSubsystem;
    IntakeSubsytem intakeSubsytem;
    OuttakeSubsystem outtakeSubsystem;
    ElapsedTime runTime = new ElapsedTime();
    String[] patterns = {" ", "GPP ", "PGP ", "PPG ", " "};
    int lastColorId = 1;
    int outtakeDir = 1;
    HuskyLens.Block[] blocks;
    boolean foundPattern = false;
    char[] charPattern;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();


        storageSubsystem = new StorageSubsystem(hardwareMap);
        storageSubsystem.InitStorage();

        intakeSubsytem = new IntakeSubsytem(hardwareMap);
        intakeSubsytem.InitIntake();

        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        outtakeSubsystem.InitOuttake();

        telemetry.addData("Aprins", "sal");
        telemetry.update();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(false);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        outtakeSubsystem.OuttakeMotorControl(gamepad2.right_stick_x);
        if (gamepad2.dpadRightWasPressed())
            storageSubsystem.setServoPos(storageSubsystem.getServoPos() == 1 ? 0.7 : 1);
        if (gamepad2.aWasPressed())
            outtakeSubsystem.ToggleShootMotor();

        intakeSubsytem.setPower(gamepad1.right_trigger);

        if (storageSubsystem.autoSort)
            storageSubsystem.PatternSortAuto(charPattern);
        else if (storageSubsystem.autoThrow)
            storageSubsystem.ThrowAll();
        else {
            if (gamepad2.bWasPressed() && foundPattern)
                storageSubsystem.autoSort = true;
            if (gamepad2.xWasPressed()) {
                outtakeSubsystem.ToggleShootMotor();
                storageSubsystem.autoThrow = true;
            }
            if (gamepad2.dpadUpWasPressed())
                storageSubsystem.MoveToPosition(475, 1);

            if (gamepad1.left_trigger > 0)
                storageSubsystem.MoveToPosition(475, 1);
        }

        if (lastColorId == 0 && !foundPattern) {
            blocks = outtakeSubsystem.GetCameraFeed();
            if (blocks.length > 0 && blocks[0].id == 21) { //TODO schimba la id-ul corect
                lastColorId = blocks[0].id;
                charPattern = patterns[lastColorId].toCharArray();
                foundPattern = true;
            }
        }

        if (storageSubsystem.idenColor() == 'G')
            telemetry.addData("Artifact", "Green");
        else if (storageSubsystem.idenColor() == 'P')
            telemetry.addData("Artifact", "Purple");
        else
            telemetry.addData("Artifact", "None");
        telemetry.addData("runTime", runTime.seconds());
        telemetry.addData("AutoSorting", storageSubsystem.autoSort);
        telemetry.addData("Turns", storageSubsystem.getTurns());
        telemetry.addData("Power", storageSubsystem.getPower());
        telemetry.addData("EncoderStorage", storageSubsystem.getPosition());
        telemetry.addData("EncoderOuttake", outtakeSubsystem.getOuttakeMotorPosition());
        telemetry.addData("OuttakeDir", outtakeDir);
        telemetry.addData("ShooterServo", storageSubsystem.getServoPos());
        telemetry.addData("Id", lastColorId + " " + patterns[lastColorId]);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}