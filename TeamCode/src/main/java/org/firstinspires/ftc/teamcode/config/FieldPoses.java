package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

public class FieldPoses {
    // Blue Alliance Poses (Source of Truth)
    public static final Pose START = new Pose(23.25028571428571, 126.49142857142861, Math.toRadians(143));
    public static final Pose SCORE = new Pose(48.72801428571427, 91.61141428571426, Math.toRadians(136));
    public static final Pose LOCK_POSE = new Pose(7.765714285714282, 135.07942857142854);

    // Pickup 1
    public static final Pose PICKUP_1 = new Pose(47.0110334, 80.3317188, Math.toRadians(-177));
    public static final Pose GET_PICK_1 = new Pose(16.908908, 80.3317188, Math.toRadians(-177));
    public static final Pose POS_GATE = new Pose(20.452620, 80.272503, Math.toRadians(-177));
    public static final Pose OPEN_GATE = new Pose(16.5, 73.0695589, Math.toRadians(-177));

    // Pickup 2
    public static final Pose PICKUP_2 = new Pose(46.5110334, 57.334464, Math.toRadians(-177));
    public static final Pose GET_PICK_2 = new Pose(10.547059, 56.334464, Math.toRadians(-177));
    public static final Pose GET_PICK_2_BACK = new Pose(30.0, 55.929082, Math.toRadians(190));

    // Pickup 3
    public static final Pose PICKUP_3 = new Pose(46.5110334, 33.558042, Math.toRadians(-177));
    public static final Pose GET_PICK_3 = new Pose(10.87840, 33.558042, Math.toRadians(-177));

    // Parking
    public static final Pose PARK = new Pose(24.06171428571428, 89.74514285714287, Math.toRadians(141));
}
