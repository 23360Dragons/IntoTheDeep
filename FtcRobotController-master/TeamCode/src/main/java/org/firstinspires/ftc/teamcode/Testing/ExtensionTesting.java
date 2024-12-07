//package org.firstinspires.ftc.teamcode.Testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.SuperStructure;
//import org.firstinspires.ftc.teamcode.utils.Global;
//
//public class ExtensionTesting extends LinearOpMode {
//    public static double p = 0,
//                         i = 0,
//                         d = 0;
//
//    public static int target = 0; // in degrees
//
//    public static double ticks_per_degree;
//
//    PIDController controller;
//    SuperStructure superStructure;
//
//    @Override
//    public void runOpMode () throws InterruptedException {
//        controller = new PIDController(p,i,d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        superStructure = new SuperStructure(hardwareMap, telemetry);
//
//        if (!superStructure.isValid) {
//            telemetry.addLine(Global.exceptions.toString());
//            telemetry.update();
//            sleep (5000);
//            telemetry.addLine("Super Structure is invalid. Exiting...");
//            telemetry.update();
//            sleep (2000);
//            requestOpModeStop();
//        }
//
//        ticks_per_degree = superStructure.extension.getTPD();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            controller.setPID(p,i,d);
//
//            double  armPosition = superStructure.extension.getPosition().right / ticks_per_degree,
//                    pid = controller.calculate(armPosition, target);
//
//            superStructure.extension.setPower(pid);
//
//            telemetry.addData("Position", armPosition);
//            telemetry.addData("Target", target);
//            telemetry.addData("ticks per degree", superStructure.articulation.getTPD());
//            telemetry.addLine();
//
//            // TODO: tune this using      192.168.43.1:8080/dash
//        }
//    }
//}
