package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.TeleOp.utils.DriveMotor;

@TeleOp
public class DragonsDriver extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    IMU imu;

    private Limelight3A limelight;

    @Override
    public void init() {

        leftFront  = DriveMotor.createNewMotor(hardwareMap, "leftFront");  // returns DcMotorEx
        rightFront = DriveMotor.createNewMotor(hardwareMap, "rightFront"); // editable in
        leftBack   = DriveMotor.createNewMotor(hardwareMap, "leftBack");   // utils\DriveMotor.java
        rightBack  = DriveMotor.createNewMotor(hardwareMap, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    @Override
    public void loop() {

        if(gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);



        double y, x, rightX;
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;

        moveRobot(botHeading,x, y, rightX);

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

    private void moveRobot(double botHeading, double x, double y, double rightX)
    {

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //we have to initialize the variable to control speed percentage
        //because y on the stick is negative, speed must be negative
        rotX *= 1.1; // Counteract imperfect strafing
        double speed = -0.5; // change to go faster or slower --- check to see if negative is required

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);
        leftFront.setPower (((rotY + rotX + rightX) / denominator) * speed);
        leftBack.setPower  (((rotY - rotX + rightX) / denominator) * speed);
        rightFront.setPower(((rotY - rotX - rightX) / denominator) * speed);
        rightBack.setPower (((rotY + rotX - rightX) / denominator) * speed);

    }
}
