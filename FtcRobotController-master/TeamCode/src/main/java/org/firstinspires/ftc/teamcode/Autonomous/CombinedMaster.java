package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class
CombinedMaster extends LinearOpMode{
    IMU _orient;
    DcMotor _leftFront;//Port 0
    DcMotor _leftBack;//Port 1
    DcMotor _rightFront;//Port2
    DcMotor _rightBack;//Port3
    //@Override

    // private DcMotor         leftDrive   = null;
    // private DcMotor         rightDrive  = null;

    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.3;

    //@Override
    //public void runOpMode()
    public void runOpMode() throws InterruptedException {

        // _intake = hardwareMap.get(DcMotor.class, "Intake");
        _orient = hardwareMap.get(IMU.class, "imu");
        _leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        _leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        _rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //assumption - right side reversed
        _leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the drive system variables.
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //
        // rightDrive.setDirection(DcMotor.Direction.FORWARD);

        _leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                _leftFront.getCurrentPosition(),
                _rightFront.getCurrentPosition(),
                _leftBack.getCurrentPosition(),
                _rightBack.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        ////////////////////////////////////////////////////////////////////////////
        //strafe right four tiles
        //_leftFront.setPower  (-0.75);
        //_leftBack.setPower    (0.75);
        //_rightFront.setPower  (0.75);
        //_rightBack.setPower  (-0.75);

        //turn right
        //_leftFront.setPower (0.75);
        //_leftBack.setPower  (0.75);
        //_rightFront.setPower(-0.75);
        //_rightBack.setPower (-0.75);
        /////////////////////////////////////////////////////////////////////////////

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        // LF LB   RF RB
//strafe right 2 feet
        encoderDrive(DRIVE_SPEED,  -12,  12, 12, -12, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//go foward 4 feet
        encoderDrive(DRIVE_SPEED,  24,  24, 24, 24, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//strafe right 1 foot
        encoderDrive(DRIVE_SPEED,  -6,  6, 6, -6, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//go backwards 4 feet
        encoderDrive(DRIVE_SPEED,  -24,  -24, -24, -24, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//go foward 4 feet
        encoderDrive(DRIVE_SPEED,  48,  48, 48, 48, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//strafe right 1 foot
        encoderDrive(DRIVE_SPEED,  -12,  12, 12, -12, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//go backwards 4 feet
        encoderDrive(DRIVE_SPEED,  -48,  -48, -48, -48, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//go fowards 4 feet
        encoderDrive(DRIVE_SPEED,  48,  48, 48, 48, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

// strafe right 1 foot
        encoderDrive(DRIVE_SPEED,  -12,  12, 12, -12, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

//go backwards 4 feet
        encoderDrive(DRIVE_SPEED,  -48,  -48, -48, -48, 5.0);  // S1: Forward 48 Inches with 5 Sec timeou

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void encoderDrive(double speed, double leftFrontInches,double leftBackInches,
                             double rightFrontInches, double rightBackInches, double timeoutS) {
        int newLeftfrontTarget;
        int newRightfrontTarget;
        int newLeftbackTarget;
        int newRightbackTarget;
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = _leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightfrontTarget = _rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftbackTarget = _leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightbackTarget = _rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            _leftFront.setTargetPosition(newLeftfrontTarget);
            _rightFront.setTargetPosition(newRightfrontTarget);
            _leftBack.setTargetPosition(newLeftbackTarget);
            _rightBack.setTargetPosition(newRightbackTarget);
            // Turn On RUN_TO_POSITION
            _leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            _leftFront.setPower(Math.abs(speed));
            _rightFront.setPower(Math.abs(speed));
            _leftBack.setPower(Math.abs(speed));
            _rightBack.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (_leftFront.isBusy() && _rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftfrontTarget,  newRightfrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        _leftFront.getCurrentPosition(), _rightFront.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            _leftFront.setPower(0);
            _rightFront.setPower(0);
            _leftBack.setPower(0);
            _rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            _leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}




