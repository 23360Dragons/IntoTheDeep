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
}