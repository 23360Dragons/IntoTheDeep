package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class DragonsLimelight {
    public static Limelight3A limelight;
    public static boolean isValid = false;

    public static Limelight3A initialize(HardwareMap hardwareMap, int pipeline) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            isValid = true;
            limelight.pipelineSwitch(pipeline);
            limelight.start();
            return limelight;
        } catch (IllegalArgumentException ex) {
            InitInfo.exceptions.append("Configuration Error: ").append("limelight").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
            isValid = false;
            return null;
        }
    }

    public static void update(Limelight3A limelight, Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            telemetry.addLine("blue sample detected");
            telemetry.addLine().addData("tx", result.getTx());
            telemetry.addLine().addData("ty", result.getTy());
            telemetry.addLine().addData("Botpose", botpose.toString());
        }
    }
}
