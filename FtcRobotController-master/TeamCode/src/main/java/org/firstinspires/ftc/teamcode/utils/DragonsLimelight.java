package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;

import java.util.List;

public class DragonsLimelight {
    public static boolean isValid = false;
    private static int currentPipeline;
    private static LLResult result;

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            telemetry.addLine("Configuring limelight...");
            telemetry.update();

            Global.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            isValid          = true;
            Global.limelight.start();

            telemetry.addLine("Limelight Configured!");
            telemetry.update();
        } catch (IllegalArgumentException ex) {
            Global.exceptions.append("Configuration Error: ").append("limelight").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    public static void update (Telemetry telemetry) {
            result = Global.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                if (DragonsLights.isValid) {
                    switch (currentPipeline) {
                        case 0:
                            DragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                            break;
                        case 1:
                            DragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            break;
                        case 2:
                            DragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            break;
                    }
                }

                telemetry.addLine().addData("tx", result.getTx());
                telemetry.addLine().addData("ty", result.getTy());

                // getting rotation of the result. TODO: make this work

            } else {
                if (result == null) {
                    telemetry.addLine("Limelight result is null");

                    if (DragonsLights.isValid)
                        DragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                } else if (!result.isValid()) {
                    telemetry.addLine("Limelight result is not valid");

                    if (DragonsLights.isValid)
                        DragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }
    }

    public static void setPipeline (int targetPipeline) {
        Global.limelight.pipelineSwitch(targetPipeline);
        currentPipeline = targetPipeline;
    }

    public static int getPipeline () {
        return currentPipeline;
    }

    public static double getTx () {
}
