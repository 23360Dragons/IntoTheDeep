package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.net.InetAddress;

public class DragonsLimelight {
    public static boolean isValid = false;

    public static void initialize(HardwareMap hardwareMap, int pipeline) {
        try {
            InitInfo.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            isValid   = true;
            InitInfo.limelight.pipelineSwitch(pipeline);
            InitInfo.limelight.start();
        } catch (IllegalArgumentException ex) {
            InitInfo.exceptions.append("Configuration Error: ").append("limelight").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
            isValid = false;
        }
    }

    public static void update (Telemetry telemetry, int pipeline, RevBlinkinLedDriver light) {
        limelight.pipelineSwitch(pipeline);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            telemetry.addLine("blue sample detected");
            light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            telemetry.addLine().addData("tx", result.getTx());
            telemetry.addLine().addData("ty", result.getTy());
            telemetry.addLine().addData("Botpose", botpose.toString());
        }
    }
}
