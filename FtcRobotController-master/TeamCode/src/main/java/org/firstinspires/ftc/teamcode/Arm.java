package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.Positions;

import java.util.Arrays;

public class Arm {
    public boolean isValid;
    public Tilt  tilt;
    public Twist twist;
    public Claw  claw;
    public Artie artie;

    public Arm (LinearOpMode opmode) {
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
        telemetry.addLine("Configuring Arm Artie!");
        telemetry.update();

        this.artie = new Artie(hardwareMap);

        telemetry.addData("Arm Artie configured", artie.isValid);
        telemetry.update();
    }

    public static class Tilt {
        public boolean isValid = true;
        private Servo servo;
        Tilt (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "tilt");
                servo.scaleRange(0,1);
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
        public boolean isValid = true;
        private Servo servo;

        Twist (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "twist");
                servo.scaleRange(0.167, 0.833);

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
        public boolean isValid = true;
        private Servo claw;

        private final double openRotation   = 1;
        private final double closedRotation = 0;

        Claw (HardwareMap hardwareMap) {
            try {
                claw = hardwareMap.get(Servo.class, "claw");
                claw.scaleRange(0.47, 0.9);
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

    public static class Artie {
        public boolean isValid = true;
        private Servo left;
        private Servo right;
        private ArtiePos artiePos;
        private ArtiePos lastArtiePos;

        private double down = 0.6; // todo : update these through testing
        private double up   = 1;
        private double back = 0.8;

        Artie (HardwareMap hardwareMap) {
            try {
                left = hardwareMap.get(Servo.class, "leftArtie");
                left.scaleRange(0,1);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("leftArtie").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                right = hardwareMap.get(Servo.class, "rightArtie");
                right.setDirection(Servo.Direction.REVERSE);
                right.scaleRange(0,1);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("rightArtie").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            artiePos = ArtiePos.DOWN;
            lastArtiePos = artiePos;
        }

        public enum ArtiePos {
            UP,
            DOWN,
            BACK
        }

        public void setPosition (ArtiePos pos) {
            artiePos = pos;
        }

        private void moveServos (double pos) {
            left.setPosition(pos);
            right.setPosition(pos);
        }

        public void updatePosition () {
            if (artiePos != lastArtiePos) {
                moveServos(artiePos == ArtiePos.UP   ? up
                         : artiePos == ArtiePos.DOWN ? down
                         : artiePos == ArtiePos.BACK ? back
                         : left.getPosition());
            }

            lastArtiePos = artiePos;
        }

        public ArtiePos getPosition () {
            return artiePos;
        }
    }
}
