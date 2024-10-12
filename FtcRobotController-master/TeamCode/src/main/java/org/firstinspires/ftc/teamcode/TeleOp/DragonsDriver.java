package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.TeleOp.utils.DriveMotor;
import org.firstinspires.ftc.teamcode.TeleOp.utils.MoveRobot;

@TeleOp(name = "Dragons Driver", group = "TeleOp")
public class DragonsDriver extends OpMode
{

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
    public void init()
    {

        leftFront  = DriveMotor.newMotor(hardwareMap, "leftFront");  // returns DcMotor - editable in utils\DriveMotor.java
        rightFront = DriveMotor.newMotor(hardwareMap, "rightFront");
        leftBack   = DriveMotor.newMotor(hardwareMap, "leftBack");
        rightBack  = DriveMotor.newMotor(hardwareMap, "rightBack");

        //reverse the right motors due to the direction they rotate being flipped on the right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection,
                usbFacingDirection));

        imu.initialize(parameters);

//        limelight = DragonsLimelight.initialize(hardwareMap, 0);

    }

    @Override
    public void loop()
    {

        if(gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y, x, rightX;
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;

        MoveRobot.moveRobotFC(botHeading, x, y, rightX, -0.5, leftFront, leftBack, rightFront, rightBack); // x, y, and rightX are the gamepad inputs
        //speed should be negative because gamepad y is negative

        //telemetry placeholder code
        {
            DcMotor[] driveMotors = {leftFront, leftBack, rightFront, rightBack};
            String[] driveMotorNames = {"leftFront", "leftBack", "rightFront", "rightBack"};

            int i = 0;
            for (DcMotor motor : driveMotors) {
                telemetry.addLine();
                telemetry.addData(driveMotorNames[i] + " power", motor.getPower());
                i++;
            }
            telemetry.addLine();
            telemetry.addData("botHeading", botHeading);
        }

//        DragonsLimelight.update(limelight, telemetry);

//        telemetry.update();

    }
}
