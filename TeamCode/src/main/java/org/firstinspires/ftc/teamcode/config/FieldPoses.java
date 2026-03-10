package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

public class FieldPoses {
    // Blue Alliance Poses (Source of Truth)
    public static final Pose START = new Pose(23.25028571428571, 126.49142857142861, Math.toRadians(143));
    public static final Pose SCORE = new Pose(48.10058571428572, 102.75198571428571, Math.toRadians(145));
    public static final Pose LOCK_POSE = new Pose(7.765714285714282, 135.07942857142854);
    public static final Pose LOW_BASKET_POSE = new Pose(5.572620000000042, 121.41536014285717);

    // Pickup 1
    public static final Pose PICKUP_1 = new Pose(47.0110334, 80.3317188, Math.toRadians(-177));
    public static final Pose GET_PICK_1 = new Pose(16.908908, 80.3317188, Math.toRadians(-177));
    public static final Pose POS_GATE = new Pose(22.552620, 80.272503, Math.toRadians(-177));
    public static final Pose OPEN_GATE = new Pose(16.6, 72.8695589, Math.toRadians(-177));

    // Pickup 2
    public static final Pose PICKUP_2 = new Pose(46.5110334, 57.334464, Math.toRadians(-177));
    public static final Pose GET_PICK_2 = new Pose(10.557059, 56.334464, Math.toRadians(-177));
    public static final Pose GET_PICK_2_POINT = new Pose(41.30171428571428, 70.59542857142856);

    // Pickup 3
    public static final Pose PICKUP_3 = new Pose(46.5110334, 33.558042, Math.toRadians(-177));
    public static final Pose GET_PICK_3 = new Pose(10.657059, 33.558042, Math.toRadians(-177));

    public static final Pose cycle = new Pose(10.654285714285749, 57.704, Math.toRadians(30));
    public static final Pose cycle2 = new Pose(10.654285714285749, 54.07714285714288, Math.toRadians(30));
    public static final Pose cyclePoint = new Pose(44.674571428571454, 66.20285714285717);

    // Parking
    public static final Pose PARK = new Pose(24.06171428571428, 89.74514285714287, Math.toRadians(141));
}
