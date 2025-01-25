package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.Positions;

public class MiniStructure {
    public boolean isValid;
    public Tilt  tilt;
    public Twist twist;
    public Claw  claw;
    public Artie artie;

    public MiniStructure(LinearOpMode opmode) {
        opmode.telemetry.addLine("Configuring MiniStructure Elbow!");
        opmode.telemetry.update();

        this.tilt = new Tilt (opmode.hardwareMap);

        opmode.telemetry.addData("MiniStructure Elbow configured", tilt.isValid);
        opmode.telemetry.addLine("Configuring MiniStructure Wrist!");
        opmode.telemetry.update();

        this.twist = new Twist (opmode.hardwareMap);

        opmode.telemetry.addData("MiniStructure Wrist configured", twist.isValid);
        opmode.telemetry.addLine("Configuring MiniStructure Claw!");
        opmode.telemetry.update();

        this.claw  = new Claw (opmode.hardwareMap);

        opmode.telemetry.addData("MiniStructure Claw configured", claw.isValid);
        opmode.telemetry.addLine("Configuring MiniStructure Artie!");
        opmode.telemetry.update();

        this.artie = new Artie(opmode.hardwareMap);

        opmode.telemetry.addData("MiniStructure Artie configured", artie.isValid);
        opmode.telemetry.update();
    }

    public static class Tilt {
        public boolean isValid = true;
        private Servo servo;
        private final double up = 0.8;
        private final double down = 0.2;

        Tilt (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "tilt");
                servo.scaleRange(0,0.6);
            } catch (Exception e) {
                Global.exceptions.append("Tilt\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

//         0 tilts it back, 1 tilts it forward
        public void setPosition (double position) {
            servo.setPosition(position);
        }

//        public void setPower (double power){
//            servo.setPower(power);
//        }

        public double getPosition () {
            return servo.getPosition();
        }

        public void up () {
            servo.setPosition(up);
        }

        public void down () {
            servo.setPosition(down);
        }
    }

    public static class Twist {
        public boolean isValid = true;
        private Servo servo;

        Twist (HardwareMap hardwareMap) {
            try {
                servo = hardwareMap.get(Servo.class, "twist");
                servo.scaleRange(0.5, 0.875);

            } catch (Exception e) {
                Global.exceptions.append("Twist\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

        public double getPosition () {
            return servo.getPosition();
        }

        public void setPosition (double rotation) {
            servo.setPosition(rotation);
        }
    }

    public static class Claw {
        public boolean isValid = true;
//        private CRServo claw;
        private Servo claw;

        private final double openRotation   = 1;
        private final double closedRotation = 0.45;

        Claw (HardwareMap hardwareMap) {
            try {
                claw = hardwareMap.get(Servo.class, "claw");
                claw.scaleRange(0, 1);
            } catch (Exception e) {
                Global.exceptions.append("Claw\n");
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

        public double getPosition () {
            return claw.getPosition();
        }
    }

    public static class Artie {
        public boolean isValid = true;
        private Servo left;
        private Servo right;
//        private ArtiePos artiePos;
//        private ArtiePos lastArtiePos;

        Artie(HardwareMap hardwareMap) {
            try {
                left = hardwareMap.get(Servo.class, "leftArm");
                left.scaleRange(0,1);
            } catch (Exception e) {
                Global.exceptions.append("leftArtie\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            // left motor - right goes back
            // right motor - left goes back ( so 0)

            try {
                right = hardwareMap.get(Servo.class, "rightArm");
                right.setDirection(Servo.Direction.REVERSE);
                right.scaleRange(0,1);
            } catch (Exception e) {
                Global.exceptions.append("rightArtie\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

//            artiePos = ArtiePos.DOWN;
//            lastArtiePos = artiePos;
        }
//        public enum ArtiePos {
//            UP,
//            DOWN,
//            BACK
//        }
//
//        public void setPosition (ArtiePos pos) {
//            artiePos = pos;
//        }

        public void setPosition (double pos) {
            left.setPosition(pos);
            right.setPosition(pos);
        }

        public Positions getPosition(){
            return new Positions(left.getPosition(), right.getPosition());
        }

//        public void updatePosition () {
//            if (artiePos != lastArtiePos) {
//                 todo : update these through testing
//
//                final double back = 1;
//                final double down = 0.5;
//                final double up   = 0.8;
//
//                if (artiePos == ArtiePos.UP)
//                    moveServos(up);
//                else if (artiePos == ArtiePos.DOWN)
//                    moveServos(down);
//                else if (artiePos == ArtiePos.BACK)
//                    moveServos(back);
//            }
//
//            lastArtiePos = artiePos;
//        }
//
//        public ArtiePos getPosition () {
//            return artiePos;
//        }
//
//        public void setPower (double pow) {
//            left.setPower(pow);
//            right.setPower(pow);
//        }
    }
}
