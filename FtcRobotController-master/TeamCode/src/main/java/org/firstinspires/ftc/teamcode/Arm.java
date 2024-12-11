package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;

public class Arm {
    public boolean isValid;
    public Tilt  tilt;
    public Twist twist;
    public Claw  claw;

    public Arm (HardwareMap hardwareMap, Telemetry telemetry) {
        telemetry.addLine("Configuring Arm Elbow!");
        telemetry.update();

        this.tilt = new Tilt (hardwareMap);

        telemetry.addData("Arm Elbow configured", tilt.isValid);
        telemetry.addLine("Configuring Arm Wrist!");
        telemetry.update();

        this.twist = new Twist (hardwareMap);

        telemetry.addData("Arm Wrist configured", twist.isValid);
        telemetry.addLine("Configuring Arm Claw!");
        telemetry.update();

        this.claw  = new Claw (hardwareMap);

        telemetry.addData("Arm Claw configured", claw.isValid);
        telemetry.update();
    }

    public static class Tilt {
        private Servo servo;
        public boolean isValid;
        Tilt (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "tilt");
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("Elbow Servo").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

        public void setPosition (double position) {
            servo.setPosition(position);
        }

        public double getPosition () {
            return servo.getPosition();
        }
    }

    public static class Twist {
        public boolean isValid;
        private Servo servo;

        Twist (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "twist");
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("Wrist pitch").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

        public void setRotation (double rotation) {
            servo.setPosition(rotation);
        }
    }

    public static class Claw {
        public boolean isValid;
        private Servo claw;

        private final double openRotation   = 1;
        private final double closedRotation = 0;

        Claw (HardwareMap hardwareMap) {
            try {
                claw = hardwareMap.get(Servo.class, "claw");
                claw.scaleRange(0.45, 1);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("Claw").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

        public void open  ()  {
            claw.setPosition(openRotation);
        }

        public void close () {
            claw.setPosition(closedRotation);
        }
    }
}
