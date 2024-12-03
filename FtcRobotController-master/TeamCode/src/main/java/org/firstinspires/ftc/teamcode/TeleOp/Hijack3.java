package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "LinearTest3")
public class Hijack3 extends OpMode
{
    DcMotor linearMotor;
    DcMotor _rightFront;
    DcMotor _leftFront;
// DcMotor _rightBack;
// DcMotor _leftBack;


    @Override
    public void init()
    {
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        _leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    }
  // _leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
    // _rightBack = hardwareMap.get(DcMotor.class, "RightBack");

    @Override
    public void loop()
    {
        // if (gamepad2.left_trigger!=0)
        // {
        //_intake.setPosition(90);
        //     linearMotor.setPower(.25);
        // }  else if (gamepad2.right_trigger!=0)
        // {
        // _intake.setPosition(0);
        //     linearMotor.setPower(-.25);
        // }
        // code for liner motor
       if (gamepad2.y) {
            linearMotor.setPower(-.65);
       } else if (gamepad2.x) {
            linearMotor.setPower(.65);
        } else if (gamepad2.a) {
            linearMotor.setPower(0);
        }

        //code for wheels
        if (gamepad1.dpad_up)
        {
            _leftFront.setPower(.25);
            _rightFront.setPower(.25);
        } else if (gamepad1.dpad_down)
        {
            _rightFront.setPower(-.25);
            _leftFront.setPower(-.25);
        }else if (gamepad1.a)
        {
            _leftFront.setPower(0);
            _rightFront.setPower(0);
        }


        }

    }
