package org.firstinspires.ftc.teamcode.Autonomous;

public final class Constants {
    // Declare your constants as public, static, and final
    //Griffin says PPR (ticks/pulses per revolution) for our drive motors is 384.5 PPR
    public static final double TICKS_PER_INCH = 384.5 / 11.88;
    public static final double ROBOT_WIDTH = 18.0; // inches
    public static final double WHEEL_DIAMETER = 3.7795; // inches
    public static final String TEAM_NAME = "Dragons Anonymous";
    public static boolean Forward = true;
    public static final boolean Reverse = false;
    public static boolean Right = true;
    public static final boolean Left = false;


    // Private constructor to prevent instantiation
    private Constants() {}

}
