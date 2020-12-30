package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "outtakeTest", group = "TeleOp")
public class outtakeTest extends OpMode {
    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    DcMotor fL;
    DcMotor fR;
    DcMotor bR;
    DcMotor bL;
    DcMotor shooter;
    DcMotor shooter2;
    CRServo elevator;
    Servo wobbleGrabber;
    DcMotor intake;
    //Servo elevator;
    DcMotor pulley1;
    Servo pusher;
    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        //elevator = hardwareMap.get(Servo.class, "elevator");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pulley1 = hardwareMap.get(DcMotor.class, "pulley");
        pusher = hardwareMap.get(Servo.class, "pusher");
        elevator = hardwareMap.get(CRServo.class, "elevator");
        telemetry.addData("robot", " initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        trigMecanum();
        if (gamepad2.right_trigger > 0.1) {
            shooter.setPower(-gamepad2.right_trigger);
            shooter2.setPower(-gamepad2.right_trigger);
            telemetry.addData("shooter", shooter.getPower());
            telemetry.update();
        } else {
            shooter.setPower(0);
            shooter2.setPower(0);
            telemetry.addData("shooter", shooter.getPower());
            telemetry.update();
        }
        if (gamepad2.dpad_down) {
            elevator.setPower(1);
            telemetry.addData("elevator", elevator.getPower());
            telemetry.update();
        } else if (gamepad2.dpad_up) {
            elevator.setPower(-1);
            telemetry.addData("elevator", elevator.getPower());
            telemetry.update();
        } else {
            elevator.setPower(0);
        }
        if (gamepad2.a) {
            wobbleGrabber.setPosition(1);
            telemetry.addData("Wobble Grabber", wobbleGrabber.getPosition());
            telemetry.update();
        } else if (gamepad2.b) {
            wobbleGrabber.setPosition(0.68);
            telemetry.addData("Wobble Grabber", wobbleGrabber.getPosition());
            telemetry.update();
        } else if (gamepad2.x) {
            wobbleGrabber.setPosition(0.5);
        }

        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(-1);
            telemetry.addData("intake", intake.getPower());
            telemetry.update();
        } else {
            intake.setPower(0);
            telemetry.addData("intake", intake.getPower());
            telemetry.update();
        }

        /*if(gamepad2.x){
            elevator.setPosition(1);
            telemetry.addData("Elevator", elevator.getPosition());
            telemetry.update();
        }
        else if(gamepad2.y){
            elevator.setPosition(0);
            telemetry.addData("Elevator", elevator.getPosition());
            telemetry.update();
        } */
        if (gamepad1.dpad_down) {
            pulley1.setPower(0.1);
            telemetry.addData("ticks", pulley1.getCurrentPosition());
            telemetry.update();
        } else if (gamepad1.dpad_up) {
            pulley1.setPower(-0.3);
            telemetry.addData("ticks", pulley1.getCurrentPosition());
            telemetry.update();
        } else {
            pulley1.setPower(0);
        }
        if (gamepad1.a) {
            pusher.setPosition(0.8);
        }
        if (gamepad1.b) {
            pusher.setPosition(0.6);
        }
    }
    public void trigMecanum(){
        rightstickx = Math.abs(gamepad1.right_stick_x) * -gamepad1.right_stick_x;
        leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

        leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y;
        leftstickyback = Math.abs(gamepad1.left_stick_y) * -gamepad1.left_stick_y;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX;
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

        telemetry.addData("fl", v1);
        telemetry.addData("fR", v2);
        telemetry.addData("bL", v3);
        telemetry.addData("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData("Right X", rightX);
        telemetry.update();

        /*if(Math.abs(gamepad1.right_stick_y) > 0.1){ //strafing method
            fL.setPower(gamepad1.right_stick_y);
            bL.setPower(-gamepad1.right_stick_y);
            fR.setPower(gamepad1.right_stick_y);
            bR.setPower(-gamepad1.right_stick_y);
        }*/
        if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
            fL.setPower(-v1);
            fR.setPower(v2);
            bL.setPower(v3);// * .79);
            bR.setPower(-v4);// * .79);
        } else if (Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2) {
            fL.setPower(v1);
            fR.setPower(-v2);
            bL.setPower(-v3);// * .79);
            bR.setPower(v4);// * .79);
        } else {
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);// * .79);
            bR.setPower(0);// * .79);
        }
    }

    public void stop(){
        telemetry.addData("Stop!!!", "hammer time");
        telemetry.update();
    }
}
