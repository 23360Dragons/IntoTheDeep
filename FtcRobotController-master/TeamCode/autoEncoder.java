package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    @Autonomous(name = "autoEncoder")
    public class auto extends LinearOpmode {

        private DcMotor left;
        private DcMotor right;

        private int leftPos;
        private int rightPos;

        @Override
        public void runOpMode() {
            left = hardwareMap.get(DcMotor.class, "leftMotor");
            right = hardwareMap.get(DcMotor.class, "leftMotor");

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);

            leftPos = 0;
            rightPos = 0;

            waitForStart();

            drive (1000, 1000, 0.25);
            drive (1000, -1000, 0.25);
        }

        private void drive(int leftTarget, int rightTarget, double speed) {
            leftPos  +=  leftTarget;
            rightPos += rightTarget;

            left.setTargetPosition(leftPos);
            right.setTargetPosition(rightPos);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setPower(speed);
            right.setPower(speed);
            while(opModeIsActive() && left.isBusy() && right.isBusy()) {
                idle();
            }
        }


    }

