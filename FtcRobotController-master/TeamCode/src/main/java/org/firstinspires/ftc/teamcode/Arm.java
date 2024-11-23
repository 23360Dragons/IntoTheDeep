package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;

public class Arm {
    public boolean isValid;
    public Elbow   elbow;
    public Wrist   wrist;
    public Claw    claw;

    public Arm (HardwareMap hardwareMap, Telemetry telemetry) {
        telemetry.addLine("Configuring Arm Elbow!");
        telemetry.update();

        this.elbow = new Elbow(hardwareMap);

        telemetry.addData("Arm Elbow configured", elbow.isValid);
        telemetry.addLine("Configuring Arm Wrist!");
        telemetry.update();

        this.wrist = new Wrist(hardwareMap);

        telemetry.addData("Arm Wrist configured", wrist.isValid);
        telemetry.addLine("Configuring Arm Claw!");
        telemetry.update();

        this.claw  = new Claw (hardwareMap);

        telemetry.addData("Arm Claw configured", claw.isValid);
        telemetry.update();
    }

    public static class Elbow {
        private Servo servo;
        public boolean isValid;
        Elbow (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "");
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

    public static class Wrist {
        public boolean isValid;
        private Servo servo;

        Wrist (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, ""); // todo: set name
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

        private double openRotation   = 90;
        private double closedRotation = 0;

        Claw (HardwareMap hardwareMap) {
            try {
                claw = hardwareMap.get(Servo.class, ""); // todo: set name
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
