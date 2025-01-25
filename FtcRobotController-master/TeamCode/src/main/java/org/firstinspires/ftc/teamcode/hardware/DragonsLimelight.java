package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.TeleOp.DragonsDriver;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.LLPipeline;

import java.util.List;

public class DragonsLimelight {
    public boolean isValid = true;
    public Limelight3A limelight;
    private int currentPipeline;
    private LLResult result;

    public DragonsLimelight (LinearOpMode opmode) {
        try {
            opmode.telemetry.addLine("Configuring limelight...");
            opmode.telemetry.update();

            limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();

            opmode.telemetry.addLine("Limelight Configured!");
            opmode.telemetry.update();
        } catch (Exception ex) {
            Global.exceptions.append("limelight\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    public double update (DragonsDriver opmode) {
        DragonsLights dragonsLights = opmode.dragonsLights;
            result = limelight.getLatestResult();
            double angle = 0;

            if (result != null && result.isValid()) {
                // Get the pose of the robot
                Pose3D botpose = result.getBotpose(); // Extract orientation (yaw, pitch, roll
                Position pos = botpose.getPosition();
                double x = pos.x; // Display orientation on telemetry
                double y = pos.y;
                double z = pos.z;
                opmode.telemetry.addData("Yaw (Z)",   z);
                opmode.telemetry.addData("Pitch (Y)", y);
                opmode.telemetry.addData("Roll (X)",  x);

                if (dragonsLights.isValid) {
                    switch (currentPipeline) {
                        case 0:
                            dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                            break;
                        case 1:
                            dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            break;
                        case 2:
                            dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            break;
                    }
                }

                if (getPipeline().num == 3) {
                    double[] pythonOutputs = result.getPythonOutput();

                    if (pythonOutputs != null && pythonOutputs.length > 0) {
                        double firstOutput = pythonOutputs[0];
                        opmode.telemetry.addData("Python output:", firstOutput);
                    }
                }

                // getting rotation of the result. TODO: make this work

//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//
//                if (!colorResults.isEmpty()) {
//                    opmode.telemetry.addLine("true");
//
//                    for (LLResultTypes.ColorResult cr : colorResults) {
//                        List<List<Double>> la = cr.getTargetCorners(); // should return {{0,0}, {1,0}, {1,1}, {0,1}} or something like that
//
//                        angle = rotateClaw(la);
//                        opmode.telemetry.addData("CR target corners", la.get(0).toString());
//                        opmode.telemetry.addData("CR target corners", la.get(1).toString());
//                        opmode.telemetry.addData("Rotate to angle", angle);
//                    }
//                }

            } else {
                if (result == null) {
                    opmode.telemetry.addLine("Limelight result is null");

                    if (dragonsLights.isValid)
                        dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                } else if (!result.isValid()) {
                    opmode.telemetry.addLine("Limelight result is not valid");

                    if (dragonsLights.isValid)
                        dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }
            return angle;
    }

    public void setPipeline (int targetPipeline) {
        limelight.pipelineSwitch(targetPipeline);
        currentPipeline = targetPipeline;
    }

    public double rotateClaw (List<List<Double>> cr) {
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

    public LLPipeline getPipeline () {
        return new LLPipeline(currentPipeline);
    }

    public double getTx () {
        return result.getTx();
    }
}
