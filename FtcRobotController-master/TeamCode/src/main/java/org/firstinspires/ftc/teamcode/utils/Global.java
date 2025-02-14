package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class Global {

    // ROBOT PARAMS
    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection  usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // SPEEDS
    public static double SSSpeed             = 0.8; // todo finetune this. Maybe different values for auto and manual?
    public static double SSCreepSpeed        = 0.5;
    public static double extSpeed            = 0.8; // todo finetune this. Maybe different values for auto and manual?
    public static double twistSpeed          = 50;
    public static double tiltSpeed           = 30;
    public static double armSpeed            = 5;
    public static double normalDriveSpeed    = 0.8;
    public static double alternateDriveSpeed = 1;

    //<editor-fold desc="Consts (Will literally never need to be edited)">
    public static StringBuilder exceptions  = new StringBuilder("The following were not found:\n");
    public static boolean exceptionOccurred = false;

    public static final int BLUE   = 0;
    public static final int RED    = 1;
    public static final int YELLOW = 2;

    public static final boolean Forward          = true;
    public static final boolean Backward         = false;
    public static final boolean Right            = true;
    public static final boolean Left             = false;
    public static final boolean Clockwise        = true;
    public static final boolean CounterClockwise = false;

    public static final double WHEEL_DIAMETER = 3.7795; // inches
    public static final double ROBOT_WIDTH    = 18.0; // inches
    public static final double TICKS_PER_INCH = 384.5 / (Math.PI * 2 * (WHEEL_DIAMETER / 2));
    public static final String TEAM_NAME = "Dragons Anonymous";
    //</editor-fold>

    //<editor-fold desc=" Control State Handling ">
    public enum ControlState {
        MANUAL,
        AUTO
    }

    public static ControlState controlState = ControlState.MANUAL;

    public static void toggleControlState () {
        if (controlState == ControlState.MANUAL)
            controlState = ControlState.AUTO;
        else
            controlState = ControlState.MANUAL;
    }

    public static void switchToManual () {
        controlState = ControlState.MANUAL;
    }

    public static void switchToAuto () {
        controlState = ControlState.AUTO;
    }
    //</editor-fold>

    private Global () {}
}