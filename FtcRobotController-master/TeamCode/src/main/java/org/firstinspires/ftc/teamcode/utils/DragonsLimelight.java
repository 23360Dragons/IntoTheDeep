package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    public static double update (Telemetry telemetry) {
            result = Global.limelight.getLatestResult();
            double angle = 0;

            if (result != null && result.isValid()) {
                // Get the pose of the robot
                Pose3D botpose = result.getBotpose(); // Extract orientation (yaw, pitch, roll
                Position pos = botpose.getPosition();
                double x = pos.x; // Display orientation on telemetry
                double y = pos.y;
                double z = pos.z;
                telemetry.addData("Yaw (Z)",   z);
                telemetry.addData("Pitch (Y)", y);
                telemetry.addData("Roll (X)",  x);

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

                if (getPipeline() == 3) {
                    double[] pythonOutputs = result.getPythonOutput();

                    if (pythonOutputs != null && pythonOutputs.length > 0) {
                        double firstOutput = pythonOutputs[0];
                        telemetry.addData("Python output:", firstOutput);
                    }
                }

                // getting rotation of the result. TODO: make this work

                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                if (!colorResults.isEmpty()) {
                    telemetry.addLine("true");

                    for (LLResultTypes.ColorResult cr : colorResults) {
                        List<List<Double>> la = cr.getTargetCorners(); // should return {{0,0}, {1,0}, {1,1}, {0,1}} or something like that

                        angle = rotateClaw(la);
                        telemetry.addData("CR target corners", la.get(0).toString());
                        telemetry.addData("CR target corners", la.get(1).toString());
                        telemetry.addData("Rotate to angle", angle);
                    }
                }

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
            return angle;
    }

    public static void setPipeline (int targetPipeline) {
        Global.limelight.pipelineSwitch(targetPipeline);
        currentPipeline = targetPipeline;
    }

    public static double rotateClaw (List<List<Double>> cr) {
        int offset = 30;

        List<Double> tl = cr.get(0),
                     tr = cr.get(1);

        boolean isFacingLeft = tl.get(1) - tr.get(1) < 0;

        double  rise = Math.abs(tl.get(1) - tr.get(1)),
                run  = Math.abs(tl.get(0) - tr.get(0));

        double target = Math.toDegrees(Math.tan(Math.toRadians(rise/run)));
        if (isFacingLeft) {
            return target + offset;
        } else {
            return target - offset;
        }
    }

    public static int getPipeline () {
        return currentPipeline;
    }

    public static double getTx () {
        return result.getTx();
    }
}
