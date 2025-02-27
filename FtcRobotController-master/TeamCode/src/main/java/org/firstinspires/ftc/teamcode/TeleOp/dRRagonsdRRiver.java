package org.firstinspires.ftc.teamcode.TeleOp;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.Autonomous.Tests.dRRagonsAuto;
import java.util.List;
@Config
//@TeleOp(name = "dRRagonsdRRiver", group = "TeleOp")
public class dRRagonsdRRiver extends LinearOpMode {
    double y = -gamepad2.left_stick_y;
    public double amnt;
    double myAngle;
    public enum ARM_POS {
        UP,
        DOWN,
    }
    ARM_POS armPos;
    public class Linearz {
        private DcMotorEx rightLinear, leftLinear;
        int bottomTicks = 0;
        int fewTicks = 533;
        int halfTicks = 1065;
        int mostTicks = 1597;
        int maxTicks = 2125;
        double linearAverage;
        //actual max is 2130, so we don't overextend we subtract five
        public Linearz(HardwareMap hardwareMap){
            leftLinear = hardwareMap.get(DcMotorEx.class, "leftLinear");
            leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLinear.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLinear.setTargetPositionTolerance(5);

            rightLinear = hardwareMap.get(DcMotorEx.class, "rightLinear");
            rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLinear.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLinear.setTargetPositionTolerance(5);

            linearAverage = ((double) (leftLinear.getCurrentPosition() + rightLinear.getCurrentPosition()) / 2);
        }

        public class ElevatorUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (linearAverage < mostTicks) {
                    leftLinear.setPower(0.75);
                    rightLinear.setPower(0.75);
                } else if (linearAverage >= mostTicks && linearAverage < maxTicks) {
                    leftLinear.setPower(0.2);
                    rightLinear.setPower(0.2);
                } else {
                    leftLinear.setPower(0);
                    rightLinear.setPower(0);
                }
                return false;
            }
        }
        public Action GoUp() {
            return new ElevatorUp();
        }
        public class ElevatorDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (linearAverage > fewTicks) {
                    leftLinear.setPower(0.75);
                    rightLinear.setPower(0.75);
                } else if (linearAverage <= fewTicks && linearAverage > bottomTicks) {
                    leftLinear.setPower(0.2);
                    rightLinear.setPower(0.2);
                } else {
                    leftLinear.setPower(0);
                    rightLinear.setPower(0);
                }
                return false;
            }
        }
        public Action GoDown(){
            return new ElevatorDown();
        }
        public class ElevatorHalf implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (linearAverage < fewTicks || linearAverage > mostTicks) {
                    leftLinear.setPower(0.75);
                    rightLinear.setPower(0.75);
                } else if (linearAverage >= fewTicks && linearAverage != halfTicks || linearAverage <= mostTicks && linearAverage != halfTicks) {
                    leftLinear.setPower(0.2);
                    rightLinear.setPower(0.2);
                } else {
                    leftLinear.setPower(0);
                    rightLinear.setPower(0);
                }
                return false;
            }
        }
        public Action GoHalf (){
            return new ElevatorHalf();
        }
        public class LinearzManual implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftLinear.setPower(y);
                rightLinear.setPower(y);
                return false;
            }
        }
        public Action manualLinear (){
            return new LinearzManual();
        }
    } //test?
    public class Armz {
        private DcMotorEx leftArm, rightArm;
        public Armz (HardwareMap hardwareMap){
            leftArm = hardwareMap.get(DcMotorEx.class, "leftArtie");
            leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftArm.setTargetPositionTolerance(5);

            rightArm = hardwareMap.get(DcMotorEx.class, "rightArtie");
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightArm.setTargetPositionTolerance(5);
            armzAvg  = ((double) (leftArm.getCurrentPosition() + rightArm.getCurrentPosition())) / 2;
        }
        double armzAvg;
        int armUp = 0;
        double armMost = -83.5;
        double armLittle = -250.5;
        int armDown = -334;
        public class ArmToBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (armzAvg < armMost) {
                    leftArm.setPower(0.75);
                    rightArm.setPower(0.75);
                } else if (armzAvg >= armMost && armzAvg < armUp) {
                    leftArm.setPower(0.3);
                    rightArm.setPower(0.3);
                } else {
                    leftArm.setPower(0);
                    rightArm.setPower(0);
                }
                armPos = ARM_POS.UP;

                return false;
            }
        }
        public Action ToBasket () {
            return new ArmToBasket();
        }
        public class ArmToGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (armzAvg < armLittle) {
                    leftArm.setPower(0.75);
                    rightArm.setPower(0.75);
                } else if (armzAvg >= armLittle && armzAvg > armDown) {
                    leftArm.setPower(0.3);
                    rightArm.setPower(0.3);
                } else {
                    leftArm.setPower(0);
                    rightArm.setPower(0);
                }
                armPos = ARM_POS.DOWN;
                return false;
            }
        }
        public Action ToGrab (){
            return new ArmToGrab();
        }

    } //test?
    public class Artie {
        private Servo leftArtie, rightArtie;
        public Artie(HardwareMap hardwareMap){
            leftArtie = hardwareMap.get(Servo.class,"leftArm");
            //leftArtie.scaleRange(0,1);
            rightArtie = hardwareMap.get(Servo.class, "rightArm");
            //rightArtie.scaleRange(0,1);
            rightArtie.setDirection(Servo.Direction.REVERSE);
        }
        double artieUP = 1;
        double artieDOWN = 0.5;
        double artieDOWNmore = 0.4;
        double artieBACK;
        public class ArtieUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftArtie.setPosition(artieUP);
                rightArtie.setPosition(artieUP);
                return false;
            }
        }
        public Action artieUp (){
            return new ArtieUp();
        }
        public class ArtieDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftArtie.setPosition(artieDOWN);
                rightArtie.setPosition(artieDOWN);
                return false;
            }
        }
        public Action artieDown (){
            return new ArtieDown();
        }
        public class ArtieBack implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftArtie.setPosition(artieBACK);
                rightArtie.setPosition(artieBACK);
                return false;
            }
        }
        public Action artieBack (){
            return new ArtieBack();
        }
        public class ArtieDownMore implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftArtie.setPosition(artieDOWNmore);
                rightArtie.setPosition(artieDOWNmore);
                return false;
            }
        }
        public Action artieDownMore (){
            return new ArtieDownMore();
        }
        public class ArmToInit implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftArtie.setPosition(0.5);
                rightArtie.setPosition(0.5);
                return false;
            }
        }
        public Action armToInit(){
            return new ArmToInit();
        }

    } //test? 1+ position tho
    public class Clawz {
        private Servo claw;
        public Clawz (HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "claw");
            claw.scaleRange(0.49, 0.9);
        }
        public class CloseClawz implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action CloseClaw(){
            return new CloseClawz();
        }
        public class OpenClawz implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                return false;
            }
        }
        public Action OpenClaw(){
            return new OpenClawz();
        }
    } //test?
    public class Seesaw {
        private Servo tilt;
        public Seesaw (HardwareMap hardwareMap) {
            tilt = hardwareMap.get(Servo.class, "tilt");
            tilt.scaleRange(0,1);
        }
        int tiltUP;
        int tiltDOWN;
        int tiltBACK;
        public class TiltUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                tilt.setPosition(tiltUP);
                return false;
            }
        }
        public Action tiltUp (){
            return new TiltUp();
        }
        public class TiltDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                tilt.setPosition(tiltDOWN);
                return false;
            }
        }
        public Action tiltDown (){
            return new TiltDown();
        }
        public class TiltBack implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                tilt.setPosition(tiltBACK);
                return false;
            }
        }
        public Action tiltBack(){
            return new TiltBack();
        }
        public class TiltToInit implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                tilt.setPosition(0.2);
                return false;
            }
        }
        public Action tiltToInit (){
            return new TiltToInit();
        }
    } //finish once you find out positions
    public class TwistNTurn {
        private Servo twist;
        public TwistNTurn (HardwareMap hardwareMap){
            twist = hardwareMap.get(Servo.class, "twist");
            twist.scaleRange(0.167, 0.833);
        }
        public class GoingHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                twist.setPosition(0.49);
                return false;
            }
        }
        public Action Resetting() {
            return new GoingHome();
        }
        public class ThisWay implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                twist.setPosition(amnt);
                return false;
            }
        }
        public Action TurnClaw () {
            return new ThisWay();
        }
        public class TwistToInit implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                twist.setPosition(1);
                return false;
            }
        }
        public Action twistToInit(){
            return new TwistToInit();
        }
    } //test?
    public class LarryLime {
        public Limelight3A littleLarryLime;
        public LarryLime(HardwareMap hardwareMap) {
            littleLarryLime = hardwareMap.get(Limelight3A.class, "limelight");
            littleLarryLime.start();
        }
        public class LarryLimeBluest implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                littleLarryLime.pipelineSwitch(Global.BLUE);
                return false;
            }
        }
        public Action LarryLimeBlues() {
            return new LarryLimeBluest();
        }
        public class LarryLimeRed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                littleLarryLime.pipelineSwitch(Global.RED);
                return false;
            }
        }
        public Action LarryLimeRedTV() {
            return new LarryLimeRed();
        }
        public class LarryLimeYeller implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                littleLarryLime.pipelineSwitch(Global.YELLOW);
                return false;
            }
        }
        public Action LarryLimeYellow() {
            return new LarryLimeYeller();
        }
        public double rotateClaw (List<List<Double>> cr) {
            List<Double> tl = cr.get(0),
                    tr = cr.get(1);

            double rise  = Math.abs(tl.get(0) - tr.get(0)),
                    run  = Math.abs(tl.get(1) - tr.get(1));

            return Math.toDegrees(Math.tan(Math.toRadians(rise/run))) + 30;
        }
        public double CalcAngle () {
            LLResult larrysJudgement = littleLarryLime.getLatestResult();
            List<LLResultTypes.ColorResult> colorResults = larrysJudgement.getColorResults();
            myAngle = 0;

            if (!colorResults.isEmpty()) {
                telemetry.addLine("true");

                for (LLResultTypes.ColorResult cr : colorResults) {
                    List<List<Double>> la = cr.getTargetCorners(); // should return {{0,0}, {1,0}, {1,1}, {0,1}} or something like that

                    double angle = rotateClaw(la);
                    telemetry.addData("CR target corners", la.get(0).toString());
                    telemetry.addData("CR target corners", la.get(1).toString());
                    telemetry.addData("Target Angle", angle);
                    myAngle = angle;
                }
            }
            return myAngle;
        }
        public class AngleSet implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                amnt = CalcAngle();
                return false;
            }
        }
        public Action SetAngle () {
            return new AngleSet();
        }
    } //test?
    public class ThisLittleLight {
        private RevBlinkinLedDriver lilLight;

        public ThisLittleLight(HardwareMap hardwareMap) {
            lilLight = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        }
        public class Bushels implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                return false;
            }
        }
        public Action Bushel (){
            return new Bushels();
        }
        public class YellowGlow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                return false;
            }
        }
        public Action LetTheYellowShine (){
            return new YellowGlow();
        }
        public class BlueGlow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                return false;
            }
        }
        public Action LetTheBlueShine (){
            return new BlueGlow();
        }
        public class RedGlow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                return false;
            }
        }
        public Action LetTheRedShine (){
            return new RedGlow();
        }
    } //test?
    @Override
    public void runOpMode() throws InterruptedException {
        Linearz linearSlidez = new Linearz(hardwareMap);
        Armz ourArmz = new Armz(hardwareMap);
        Artie littleArtie = new Artie(hardwareMap);
        Clawz clawz = new Clawz(hardwareMap);
        Seesaw seesaw = new Seesaw(hardwareMap);
        TwistNTurn twistyturny = new TwistNTurn(hardwareMap);
       //LarryLime littleLarryLime = new LarryLime(hardwareMap);
        //ThisLittleLight littleLight = new ThisLittleLight(hardwareMap);


        while (opModeInInit()){

            int starty2 = dRRagonsAuto.myStart;

            switch (starty2) {
                case 1:
                case 3:
                    //Actions.runBlocking(littleLarryLime.LarryLimeBlues());
                    telemetry.addLine("Limelight Set To Blue");
                    telemetry.update();
                    break;
                case 2:
                case 4:
                    //Actions.runBlocking(littleLarryLime.LarryLimeRedTV());
                    telemetry.addLine("Limelight Set To Red");
                    telemetry.update();
                    break;
                default:
                    telemetry.addLine("Please select starting position! If not selected, the robot will not run during Auto.");
                    break;
            }
        }




        MecanumDrive drive = new MecanumDrive(hardwareMap, null);
//47 in to top at full ext artie up
//19 in to top at no ext artie up
//41 in to top at full ext artie out/down
//13 in to top at no ext artie out/down
// artie up +6 in, artie out/down -6in
        Action locateSample = new SequentialAction();
        Action toGrabby = new SequentialAction(
                ourArmz.ToGrab(),
                linearSlidez.GoHalf()
        );
        Action grabby = new SequentialAction (
                //littleLarryLime.SetAngle(),
                twistyturny.TurnClaw(),
                littleArtie.artieDown(),
                clawz.CloseClaw(),
                littleArtie.artieUp(),
                twistyturny.Resetting()
        );
        Action depositTop = new SequentialAction(
                linearSlidez.GoUp(),
                littleArtie.artieBack(),
                seesaw.tiltBack(),
                clawz.OpenClaw()
        );
        Action depositBottom = new SequentialAction(
                linearSlidez.GoHalf(),
                littleArtie.artieBack(),
                seesaw.tiltBack(),
                clawz.OpenClaw()
        );
        Action specimenBottom = new SequentialAction(
                linearSlidez.GoDown(),
                seesaw.tiltUp(),
                littleArtie.artieDownMore(),
                clawz.OpenClaw()
        );
        Action specimenTop = new SequentialAction(
                linearSlidez.GoHalf(),
                seesaw.tiltUp(),
                littleArtie.artieDownMore(),
                clawz.OpenClaw()
        );
        waitForStart();


        if (y != 0){
            Actions.runBlocking(linearSlidez.manualLinear());
        }
        if (gamepad2.x) {
            Actions.runBlocking(grabby);
        }
        if (gamepad2.y){
            Actions.runBlocking(specimenTop);
        }
        if (gamepad2.a){
            Actions.runBlocking(specimenBottom);
        }
        if (gamepad2.b){
            Actions.runBlocking(locateSample);
        }
        if (gamepad2.dpad_up){
            Actions.runBlocking(depositTop);
        }
        if (gamepad2.dpad_down){
            Actions.runBlocking(depositBottom);
        }
        if (gamepad2.dpad_right){

        }
    }
}