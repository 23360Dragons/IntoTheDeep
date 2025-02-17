package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

public class AutoRobotMovement {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private LinearOpMode opmode;

    public AutoRobotMovement(AutonomousOpMode opmode) {
        new AutoRobotMovement(opmode.drivetrain, opmode);
    }
    
    public AutoRobotMovement(Drivetrain drivetrain, LinearOpMode opMode) {
        frontLeft  = drivetrain.leftFront;
        frontRight = drivetrain.rightFront;
        backLeft   = drivetrain.leftBack;
        backRight  = drivetrain.rightBack;
        opmode     = opMode;
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Moves the robot forward or backward for the specified distance.
     *
     * @param distance The distance to move in inches.
     * @param forward   True to move forward, false to move backward.
     */
    public void moveForward(double distance, boolean forward, double speed) {
        double targetTicks = distance * Global.TICKS_PER_INCH;
        targetTicks *= 1;

        resetEncoders();

        frontLeft.setTargetPosition ((int) (forward ? targetTicks : -targetTicks));
        frontRight.setTargetPosition((int) (forward ? targetTicks : -targetTicks));
        backLeft.setTargetPosition  ((int) (forward ? targetTicks : -targetTicks));
        backRight.setTargetPosition ((int) (forward ? targetTicks : -targetTicks));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        frontLeft.setPower(forward ?  speed : -speed);
//        frontRight.setPower(forward ? speed : -speed);
//        backLeft.setPower(forward ?   speed : -speed);
//        backRight.setPower(forward ?  speed : -speed);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opmode.opModeIsActive()) {
            // Do nothing, just wait
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Moves the robot to the right or left for the specified distance.
     *
     * @param distance The distance to move in inches.
     * @param right    True to move right, false to move left.
     */
    public void strafe(double distance, boolean right, double speed) {
        double targetTicks = distance * Global.TICKS_PER_INCH;
        targetTicks*=1.74;

        resetEncoders();

        frontLeft.setTargetPosition( (int) (right ?  targetTicks  : -targetTicks));
        frontRight.setTargetPosition((int) (right ? -targetTicks  :  targetTicks));
        backLeft.setTargetPosition(  (int) (right ? -targetTicks  :  targetTicks));
        backRight.setTargetPosition( (int) (right ?  targetTicks  : -targetTicks));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        frontLeft.setPower(right ?   speed : -speed);
//        frontRight.setPower(right ? -speed :  speed);
//        backLeft.setPower(right ?   -speed :  speed);
//        backRight.setPower(right ?   speed : -speed);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opmode.opModeIsActive()) {
            // Do nothing, just wait
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    /**
     * Rotates the robot by the specified number of degrees.
     *
     * @param degrees The number of degrees to rotate.
     * @param clockwise True to rotate clockwise, false to rotate counterclockwise.
     */
    public void rotate(double degrees, boolean clockwise, double speed) {
        // Calculate the distance traveled by each wheel during rotation
        double wheelCircumference = 2 * Math.PI * Global.ROBOT_WIDTH;
        double distancePerDegree = wheelCircumference / 360.0;
        double rotationDistance = degrees * distancePerDegree;

        // Calculate target encoder counts
        double targetTicks = rotationDistance * Global.TICKS_PER_INCH;
        targetTicks *= (195.0 /180.0);

        resetEncoders();

        // Set motor target positions
        frontLeft.setTargetPosition( (int) (clockwise ? targetTicks  : -targetTicks));
        frontRight.setTargetPosition((int) (clockwise ? -targetTicks : targetTicks));
        backLeft.setTargetPosition(  (int) (clockwise ? targetTicks  : -targetTicks));
        backRight.setTargetPosition( (int) (clockwise ? -targetTicks : targetTicks));

        // Set motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
//        frontLeft.setPower(clockwise ?  speed : -speed);
//        frontRight.setPower(clockwise ? -speed :  speed);
//        backLeft.setPower(clockwise ?   speed : -speed);
//        backRight.setPower(clockwise ?  -speed :  speed);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Wait for motors to reach target position
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opmode.opModeIsActive()) {
            // Do nothing, just wait
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
