package org.firstinspires.ftc.teamcode.utils;

public class AutoRobotPos {
    public static float rotation = 0;
    public static boolean autoRun = false;

    public static void store (double pose) {
        rotation = (float) pose;
        autoRun = true;
    }

    public static void reset () {
        rotation = 0;
        autoRun = false;
    }

    public static float getRotation () {
        return rotation;
    }
}
