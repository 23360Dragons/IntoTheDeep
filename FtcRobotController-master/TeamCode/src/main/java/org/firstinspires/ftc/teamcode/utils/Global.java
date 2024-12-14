package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Global {
    public static StringBuilder exceptions = new StringBuilder("The following were not found:\n");
    public static boolean exceptionOccurred = false;

    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection  usbFacingDirection   = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    public static final int BLUE   = 0;
    public static final int RED    = 1;
    public static final int YELLOW = 2;

    public static final int LEFT  = 0;
    public static final int RIGHT = 1;
}