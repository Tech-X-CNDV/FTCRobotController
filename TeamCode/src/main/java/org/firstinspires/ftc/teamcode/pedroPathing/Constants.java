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
                        .mass(11.2)
                        .forwardZeroPowerAcceleration(-34.20381022599874)
                        .lateralZeroPowerAcceleration(-70.18430289749332)
                        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.03, 0.03))
                        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.03, 0.01))
                        .useSecondaryTranslationalPIDF(true)
                        .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.03))
                        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.1, 0.01))
                        .useSecondaryHeadingPIDF(true)
                        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0001, 0.1, 0.01))
                        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, 0.003, 0.8, 0.003))
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
                        .xVelocity(76.8154037355)
                        .yVelocity(58.709159310408474)
                        .useBrakeModeInTeleOp(true);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                        .forwardPodY(-1.3)
                        .strafePodX(0)
                        .distanceUnit(DistanceUnit.INCH)
                        .hardwareMapName("pinpoint")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // public static PathConstraints pathConstraints = new PathConstraints(0.99,
        // 100, 1, 1);
        public static PathConstraints pathConstraints = new PathConstraints(0.995, 200, 1, 1);

        public static Follower createFollower(HardwareMap hardwareMap) {
                return new FollowerBuilder(followerConstants, hardwareMap)
                                .pathConstraints(pathConstraints)
                                .mecanumDrivetrain(driveConstants)
                                .pinpointLocalizer(localizerConstants)
                                .build();
        }
}