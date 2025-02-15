package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

public class AutoRobotMovement {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public AutoRobotMovement(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
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

        frontLeft.setPower(forward ?  speed : -speed);
        frontRight.setPower(forward ? speed : -speed);
        backLeft.setPower(forward ?   speed : -speed);
        backRight.setPower(forward ?  speed : -speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
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

        frontLeft.setPower(right ?   speed : -speed);
        frontRight.setPower(right ? -speed :  speed);
        backLeft.setPower(right ?   -speed :  speed);
        backRight.setPower(right ?   speed : -speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
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
    public void rotate(double degrees, boolean clockwise) {
        // Calculate the distance traveled by each wheel during rotation
        double wheelCircumference = Math.PI * Global.ROBOT_WIDTH;
        double distancePerDegree = wheelCircumference / 360.0;
        double rotationDistance = degrees * distancePerDegree;

        // Calculate target encoder counts
        double targetTicks = rotationDistance * Global.TICKS_PER_INCH;

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
        frontLeft.setPower(clockwise ?   0.5 : -0.5);
        frontRight.setPower(clockwise ? -0.5 :  0.5);
        backLeft.setPower(clockwise ?    0.5 : -0.5);
        backRight.setPower(clockwise ?  -0.5 :  0.5);

        // Wait for motors to reach target position
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            // Do nothing, just wait
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Moves the robot diagonally for the specified distance and angle.
     *
     * @param distance The distance to move in inches.
     * @param angle The angle of the diagonal movement in degrees (0-360).
     */
    public void moveDiagonallyRight(double distance, double angle) {
        double angleRadians = Math.toRadians(angle);

        double frontLeftPower = Math.cos(angleRadians);
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = Math.cos(angleRadians);

        // Adjust power ratios for mecanum drive (simplified)
//        frontLeftPower *= -1;
//        backRightPower *= -1;

        resetEncoders();

        double targetTicks = distance * Global.TICKS_PER_INCH;

        frontLeft.setTargetPosition((int) (targetTicks * frontLeftPower));
        frontRight.setTargetPosition((int) (targetTicks * frontRightPower));
        backLeft.setTargetPosition((int) (targetTicks * backLeftPower));
        backRight.setTargetPosition((int) (targetTicks * backRightPower));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower((frontLeftPower));
        frontRight.setPower((frontRightPower));
        backLeft.setPower((backLeftPower));
        backRight.setPower((backRightPower));

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            // Do nothing, just wait
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void moveDiagonallyLeft(double distance, double angle, LinearOpMode opmode) {
//        double angleRadians = Math.toRadians(angle);

        double[] drivePowers = MoveRobot.moveRobotAngle(angle, 0.5);

        double frontLeftPower  = drivePowers[0];
        double frontRightPower = drivePowers[1];
        double backLeftPower   = drivePowers[2];
        double backRightPower  = drivePowers[3];

        resetEncoders();

        opmode.telemetry.addLine(Arrays.toString(drivePowers));
        opmode.telemetry.update();
        opmode.sleep(1000);


        // Adjust power ratios for mecanum drive (simplified)
//        frontLeftPower *= -1;
//        backRightPower *= -1;

        double targetTicks = distance * Global.TICKS_PER_INCH;

        opmode.telemetry.addData("Target ticks", targetTicks);
        opmode.telemetry.update();

        //todo look at this output
        opmode.sleep (1000);

        frontLeft.setTargetPosition((int) (targetTicks * frontLeftPower));
        frontRight.setTargetPosition((int) (targetTicks * frontRightPower));
        backLeft.setTargetPosition((int) (targetTicks * backLeftPower));
        backRight.setTargetPosition((int) (targetTicks * backRightPower));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower((frontLeftPower));
        frontRight.setPower((frontRightPower));
        backLeft.setPower((backLeftPower));
        backRight.setPower((backRightPower));

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            // Do nothing, just wait
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
