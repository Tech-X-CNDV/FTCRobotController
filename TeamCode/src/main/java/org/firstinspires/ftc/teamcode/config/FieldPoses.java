package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

public class FieldPoses {
    // Blue Alliance Poses (Source of Truth)
    public static final Pose START = new Pose(23.25028571428571, 126.49142857142861, Math.toRadians(143));
    public static final Pose SCORE = new Pose(48.74629999999999, 91.66398571428567, Math.toRadians(136));
    public static final Pose LOCK_POSE = new Pose(7.780571428571426, 135.07199999999997);

    // Pickup 1
    public static final Pose PICKUP_1 = new Pose(46.5110334, 80.3317188, Math.toRadians(-177));
    public static final Pose GET_PICK_1 = new Pose(14.008908, 80.3317188, Math.toRadians(-177));
    public static final Pose POS_GATE = new Pose(25.452620, 80.272503, Math.toRadians(-177));
    public static final Pose OPEN_GATE = new Pose(15.7, 73.0695589, Math.toRadians(-177));

    // Pickup 2
    public static final Pose PICKUP_2 = new Pose(46.5110334, 56.334464, Math.toRadians(-177));
    public static final Pose GET_PICK_2 = new Pose(7.847059, 56.334464, Math.toRadians(-177));
    public static final Pose GET_PICK_2_BACK = new Pose(30.0, 55.929082, Math.toRadians(190));

    // Pickup 3
    public static final Pose PICKUP_3 = new Pose(46.5110334, 32.558042, Math.toRadians(-177));
    public static final Pose GET_PICK_3 = new Pose(7.87840, 32.558042, Math.toRadians(-177));

    // Parking
    public static final Pose PARK = new Pose(24.06171428571428, 89.74514285714287, Math.toRadians(141));
}
