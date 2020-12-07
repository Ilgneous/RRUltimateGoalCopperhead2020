package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "outtakeTest", group = "TeleOp")
public class outtakeTest extends OpMode {
    hardwareMap robot = new hardwareMap();
    DcMotor shooter;
    DcMotor shooter2;
    //CRServo elevator;
    Servo wobbleGrabber;
    DcMotor intake;
    Servo elevator;
    DcMotor pulley1;
    Servo pusher;
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        elevator = hardwareMap.get(Servo.class, "elevator");
        //wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pulley1 = hardwareMap.get(DcMotor.class, "pulley");
        pusher = hardwareMap.get(Servo.class, "pusher");
        //elevator = hardwareMap.get(Servo.class, "elevator");
        telemetry.addData("robot", " initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.1){
            shooter.setPower(-gamepad1.right_trigger);
            shooter2.setPower(-gamepad1.right_trigger);
            telemetry.addData("shooter", shooter.getPower());
            telemetry.update();
        }
        else{
            shooter.setPower(0);
            shooter2.setPower(0);
            telemetry.addData("shooter", shooter.getPower());
            telemetry.update();
        }
       /* if(gamepad1.left_stick_y > 0.1){
            elevator.setPower(gamepad1.left_stick_y);
            telemetry.addData("elevator", elevator.getPower());
            telemetry.update();
        }
        else{
            elevator.setPower(0);
            telemetry.addData("elevator", elevator.getPower());
            telemetry.update();
        } */
        /*if(gamepad1.a){
            wobbleGrabber.setPosition(1);
            telemetry.addData("Wobble Grabber", wobbleGrabber.getPosition());
            telemetry.update();
        }
        else if(gamepad1.b){
            wobbleGrabber.setPosition(0.68);
            telemetry.addData("Wobble Grabber", wobbleGrabber.getPosition());
            telemetry.update();
        }
        else if(gamepad1.x){
            wobbleGrabber.setPosition(0.5);
        }*/

        if(gamepad1.left_trigger > 0.1){
            intake.setPower(-gamepad1.left_trigger);
            telemetry.addData("intake", intake.getPower());
            telemetry.update();
        }
        else{
            intake.setPower(0);
            telemetry.addData("intake", intake.getPower());
            telemetry.update();
        }

        if(gamepad2.x){
            elevator.setPosition(1);
            telemetry.addData("Elevator", elevator.getPosition());
            telemetry.update();
        }
        else if(gamepad2.y){
            elevator.setPosition(0);
            telemetry.addData("Elevator", elevator.getPosition());
            telemetry.update();
        }
        if(gamepad1.dpad_down){
            pulley1.setPower(0.1);
        }
        else if (gamepad1.dpad_up)
        {
            pulley1.setPower(-0.3);
        }
        else
        {
            pulley1.setPower(0);
        }
        if(gamepad2.a){
            pusher.setPosition(1);
        }
        else if(gamepad2.b){
            pusher.setPosition(0.8);
        }



    }
    public void stop(){
        telemetry.addData("Stop!!!", "hammer time");
        telemetry.update();
    }
}
