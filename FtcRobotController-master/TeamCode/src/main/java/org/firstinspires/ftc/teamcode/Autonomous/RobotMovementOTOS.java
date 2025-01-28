package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotMovementOTOS {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Declare OTOS sensor
    private SparkFunOTOS otos;

    public RobotMovementOTOS(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, SparkFunOTOS otos) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        this.otos = otos;
        configureOtos();
    }

    /**
     * Moves the robot forward or backward for the specified distance using OTOS.
     *
     * @param distance The distance to move in inches.
     * @param forward  True to move forward, false to move backward.
     */
    public void moveForward(double distance, boolean forward) {
        // Get initial robot position
        double initialX = otos.getPosition().x;

        // Set target position
        double targetX = initialX + (forward ? distance : -distance);

        // Set motor power
        double power = forward ? 0.5 : -0.5;

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Move until target position is reached
        while ((forward && otos.getPosition().x < targetX) || (!forward && otos.getPosition().x > targetX)) {
            // Continue moving
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
     /**
     * Moves the robot to the right or left for the specified distance using OTOS.
     *
     * @param distance The distance to move in inches.
     * @param right     True to move right, false to move left.
     */
    public void strafe(double distance, boolean right) {
        // Get initial robot position
        double initialY = otos.getPosition().y; 

        // Set target position
        double targetY = initialY + (right ? distance : -distance);

        // Set motor power (adjust as needed)
        double power = right ? 0.5 : -0.5;

        frontLeft.setPower(right ? -power : power); 
        frontRight.setPower(right ? power : -power);
        backLeft.setPower(right ? power : -power);
        backRight.setPower(right ? -power : power);

        // Move until target position is reached
        while ((right && otos.getPosition().y < targetY) || (!right && otos.getPosition().y > targetY)) {
            // Continue moving
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
    /**
     * Moves the robot diagonally for the specified distance and angle using OTOS.
     *
     * @param distance The distance to move in inches.
     * @param angle    The angle of the diagonal movement in degrees (0-360).
     */
    public void moveDiagonally(double distance, double angle) {
        // Get initial robot position
        double initialX = otos.getPosition().x;
        double initialY = otos.getPosition().y;

        // Calculate target position based on distance and angle
        double targetX = initialX + (distance * Math.cos(Math.toRadians(angle)));
        double targetY = initialY + (distance * Math.sin(Math.toRadians(angle)));

        // Calculate and set motor powers (simplified)
        double angleRadians = Math.toRadians(angle);
        double frontLeftPower = Math.cos(angleRadians);
        double frontRightPower = Math.sin(angleRadians);
        double backLeftPower = Math.sin(angleRadians);
        double backRightPower = Math.cos(angleRadians);

        // Adjust power ratios for mecanum drive (simplified)
        frontLeftPower *= -1;
        backRightPower *= -1;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Move until target position is reached (simplified)
        while (Math.sqrt(Math.pow(otos.getPosition().x - targetX, 2) + Math.pow(otos.getPosition().y - targetY, 2)) > 0.1) { 
            // Continue moving
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

        /**
     * Rotates the robot by the specified number of degrees using OTOS.
     *
     * @param degrees The number of degrees to rotate.
     * @param clockwise True to rotate clockwise, false to rotate counterclockwise.
     */
    public void rotate(double degrees, boolean clockwise) {
        // Get initial robot heading (You might need to implement a method to get robot heading from OTOS data)
        double initialHeading = getRobotHeadingFromOTOS(); 

        // Calculate target heading
        double targetHeading = initialHeading + (clockwise ? degrees : -degrees);

        // Normalize target heading to be within 0-360 degrees
        targetHeading = normalizeAngle(targetHeading); 

        // Set motor powers for rotation (adjust as needed)
        double power = 0.5; 
        frontLeft.setPower(clockwise ? power : -power);
        frontRight.setPower(clockwise ? -power : power);
        backLeft.setPower(clockwise ? power : -power);
        backRight.setPower(clockwise ? -power : power);

        // Rotate until target heading is reached
        while (Math.abs(getRobotHeadingFromOTOS() - targetHeading) > 1.0) { // Adjust tolerance as needed
            // Continue rotating
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Helper method to normalize an angle to be within 0-360 degrees.
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    private double normalizeAngle(double angle) {
        while (angle < 0) {
            angle += 360;
        }
        while (angle >= 360) {
            angle -= 360;
        }
        return angle;
    }

		//TODO
    /*
     * Helper method to get the robot's current heading from OTOS data.
     * 
     * **This method needs to be implemented based on how your OTOS sensor provides heading information.** 
     * 
     * For example:
     * 
     * 1. If OTOS provides a direct heading value:
     * 
     *     ```java
     *     public double getRobotHeadingFromOTOS() {
     *         return otos.getHeading(); 
     *     }
     *     ```
     * 
     * 2. If OTOS provides X and Y coordinates and you need to calculate heading:
     * 
     *     ```java
     *     public double getRobotHeadingFromOTOS() {
     *         double robotX = otos.getPosition().x;
     *         double robotY = otos.getPosition().y;
     *         double heading = Math.atan2(robotY, robotX) * 180 / Math.PI; 
     *         return normalizeAngle(heading); 
     *     }
     *     ```
     *
     * @return The robot's current heading in degrees.
     */
    private double getRobotHeadingFromOTOS() {
        // Implement this method based on your specific OTOS sensor and robot configuration
        return 0.0; // Placeholder
    }
    
    
    //using configuration from DragonsOTOS 
    //how confident are we in those???
    private void configureOtos() {
    	// myOtos.setLinearUnit(DistanceUnit.METER);
        otos.setLinearUnit(DistanceUnit.INCH);
        // otos.setAngularUnit(AnguleUnit.RADIANS);
        otos.setAngularUnit(AngleUnit.DEGREES);
        
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(0.992);
        
        otos.calibrateImu();
        otos.resetTracking();
        
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
        
        //why are these lines needed?
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
        
    }
  }