package org.firstinspires.ftc.teamcode.TeleOp.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class DragonsLimelight
{
    public static Limelight3A initialize (Limelight3A limelight, HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        return limelight;
    }

    public static void update (Limelight3A limelight, Telemetry telemetry)
    {
        LLResult result = limelight.getLatestResult();
        if (result != null) {

            if (result.isValid()) {

                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

            }

        }
    }
}
