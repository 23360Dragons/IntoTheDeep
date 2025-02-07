package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Global {
    public static StringBuilder exceptions  = new StringBuilder("The following were not found:\n");
    public static boolean exceptionOccurred = false;

    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection  usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static final int BLUE   = 0;
    public static final int RED    = 1;
    public static final int YELLOW = 2;

    public static final double TICKS_PER_INCH = 384.5 / 11.88;
    public static final double ROBOT_WIDTH = 18.0; // inches
    public static final double WHEEL_DIAMETER = 3.7795; // inches
    public static final String TEAM_NAME = "Dragons Anonymous";
    public static boolean Forward = true;
    public static final boolean Backward = false;
    public static boolean Right = true;
    public static final boolean Left = false;
    public static final boolean Clockwise = true;
    public static final boolean CounterClockwise = false;

    public enum ControlState {
        MANUAL,
        AUTO
    }

    public static Global.ControlState controlState = ControlState.MANUAL;

    public static void toggleControlState () {
        if (controlState == ControlState.MANUAL)
            controlState = ControlState.AUTO;
        else
            controlState = ControlState.MANUAL;
    }

    private Global () {

    }
}