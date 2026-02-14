package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static double allianceOffset = 0;
    public static boolean isRed = false;
    public static Pose autoPoseBlue = FieldPoses.START;
    public static Pose autoPoseRed = FieldPoses.START.mirror();
}
