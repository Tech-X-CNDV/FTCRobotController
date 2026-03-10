package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
                        .mass(14.5)
                        .forwardZeroPowerAcceleration(-34.90381022599874)
                        .lateralZeroPowerAcceleration(-65.65430289749332)
                        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.035, 0.02))
                        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.05, 0))
                        .useSecondaryTranslationalPIDF(true)
                        .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.02))
                        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.1, 0))
                        .useSecondaryHeadingPIDF(true)
                        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015))
                        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.00035, 0.6, 0.02))
                        .useSecondaryDrivePIDF(true)
                        .centripetalScaling(0.0005);

        public static MecanumConstants driveConstants = new MecanumConstants()
                        .maxPower(1)
                        .rightFrontMotorName("rightFrontMotor")
                        .rightRearMotorName("rightRearMotor")
                        .leftRearMotorName("leftRearMotor")
                        .leftFrontMotorName("leftFrontMotor")
                        .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
                        .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
                        .xVelocity(77.73205229)
                        .yVelocity(60.44209493)
                        .useBrakeModeInTeleOp(true);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                        .forwardPodY(-0.5)
                        .strafePodX(0)
                        .distanceUnit(DistanceUnit.INCH)
                        .hardwareMapName("pinpoint")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // public static PathConstraints pathConstraints = new PathConstraints(0.99,
        // 100, 1, 1);
        public static PathConstraints pathConstraints = new PathConstraints(0.97, 100, 1, 0.8);

        public static Follower createFollower(HardwareMap hardwareMap) {
                return new FollowerBuilder(followerConstants, hardwareMap)
                                .pathConstraints(pathConstraints)
                                .mecanumDrivetrain(driveConstants)
                                .pinpointLocalizer(localizerConstants)
                                .build();
        }
}