package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.utils.init.Consts;

public class DragonsLimelight {
    public static boolean isValid = false;
    public static int currentPipeline;


    public static void initialize(HardwareMap hardwareMap) {
        try {
            Consts.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            isValid   = true;
            Consts.limelight.start();
        } catch (IllegalArgumentException ex) {
            Consts.exceptions.append("Configuration Error: ").append("limelight").append(" does not exist").append("\n");
            Consts.exceptionOccurred = true;
            isValid = false;
        }
    }

    public static void update (Telemetry telemetry) {
            LLResult result = Consts.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();

                switch (currentPipeline) {
                    case 0:
                        Consts.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        break;
                    case 1:
                        Consts.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        break;
                    case 2:
                        Consts.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                        break;
                    case 3:
                        break;

                }

                telemetry.addLine().addData("tx", result.getTx());
                telemetry.addLine().addData("ty", result.getTy());
                telemetry.addLine().addData("Botpose", botpose.toString());

            } else {
                if (result == null) {
                    telemetry.addLine("Limelight result is null");

                } else if (!result.isValid()) {
                    telemetry.addLine("Limelight result is REALLY not valid");
                    Consts.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }
    }

    public static void setPipeline (int targetPipeline) {
        Consts.limelight.pipelineSwitch(targetPipeline);
        currentPipeline = targetPipeline;
    }
}
