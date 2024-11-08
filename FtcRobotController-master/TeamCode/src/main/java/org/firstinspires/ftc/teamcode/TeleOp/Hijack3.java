package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "LinearTest3")
public class Hijack3 extends OpMode
{


    DcMotor linearMotor;


    @Override
    public void init()
    {
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");

    }

    @Override
    public void loop() {
        // if (gamepad2.left_trigger!=0)
        // {
        //_intake.setPosition(90);
        //     linearMotor.setPower(.25);
        // }  else if (gamepad2.right_trigger!=0)
        // {
        // _intake.setPosition(0);
        //     linearMotor.setPower(-.25);
        // }

        if (gamepad2.y) {
            linearMotor.setPower(-.75);
        } else if (gamepad2.x) {
            linearMotor.setPower(.75);
        }

    }
}