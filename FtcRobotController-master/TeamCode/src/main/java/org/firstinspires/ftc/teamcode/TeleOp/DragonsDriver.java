package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.TeleOp.utils.DriveMotor;

@TeleOp(name = "Dragons Driver", group = "TeleOp")
public class DragonsDriver extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    IMU imu;

    // TODO: change these values based on robot construction
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private Limelight3A limelight;

    @Override
    public void init() {

        leftFront  = DriveMotor.newMotor(hardwareMap, "leftFront");  // returns DcMotorEx - editable in utils\DriveMotor.java
        rightFront = DriveMotor.newMotor(hardwareMap, "rightFront");
        leftBack   = DriveMotor.newMotor(hardwareMap, "leftBack");
        rightBack  = DriveMotor.newMotor(hardwareMap, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection,
                usbFacingDirection));

        imu.initialize(parameters);

        limelight = DragonsLimelight.initialize(limelight, hardwareMap);

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

        moveRobot(botHeading, x, y, rightX); // x, y, and rightX are the gamepad inputs

        DragonsLimelight.update(limelight, telemetry);

        telemetry.update();

    }

    private void moveRobot(double botHeading, double x, double y, double rightX)
    {

        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1; // Counteract imperfect strafing
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        /*
        these are the rotated x and y (i.e. the vector relative to the field rather than to the robot)

        x * Math.cos and Math.sin are using trigonometry to find the values of the distances between
        the robot-centric vector and the field-centric vector (the target vector). rotY does the same

        in essence, x and y are the values of the initial vector, and rotX and rotY are the values
        of the field-centric vector (the vector after it has been transformed by the botHeading)

        see the Mecanum Drive Tutorial on gm0.org for more info
         */

        //we have to initialize the variable to control speed percentage
        //because y on the stick is negative, speed must be negative
        double speed = -0.5;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1); //makes sure values don't get scaled up by dividing by a decimal
        //takes the sum of all inputs so the total motor power is within the range -1 to 1
        // TODO: possibly swap speed and denominator - test
        leftFront.setPower (((rotY + rotX + rightX) * speed) / denominator);
        leftBack.setPower  (((rotY - rotX + rightX) * speed) / denominator);
        rightFront.setPower(((rotY - rotX - rightX) * speed) / denominator);
        rightBack.setPower (((rotY + rotX - rightX) * speed) / denominator);


        //telemetry placeholder code
        DcMotor[] driveMotors = {leftFront, leftBack, rightFront, rightBack};
        String[] driveMotorNames = {"leftFront", "leftBack", "rightFront", "rightBack"};

        int i = 0;
        for (DcMotor motor : driveMotors) {
            telemetry.addData(driveMotorNames[i] + " power", motor.getPower());
            telemetry.addLine();
            i++;
        }
        telemetry.addData("botHeading", botHeading);

    }
}
