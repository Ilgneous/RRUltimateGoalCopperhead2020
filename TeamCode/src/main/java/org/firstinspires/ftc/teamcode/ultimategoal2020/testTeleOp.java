package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "testTeleop", group = "TeleOp")
public class testTeleOp extends OpMode {
    //define motors, servos, sensors, variables, etc.
    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // robot.robot_init(hardwareMap, true);
        // robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop(){
        trigMecanum();
    }

    @Override
    public void stop() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    public void trigMecanum() { //turning and moving method
        rightstickx = Math.abs(gamepad1.right_stick_x) * -gamepad1.right_stick_x ;
        leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

        leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y ;
        leftstickyback = Math.abs(gamepad1.left_stick_y) * -gamepad1.left_stick_y ;

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
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData ("Right X", rightX);
        telemetry.update();

        /*if(Math.abs(gamepad1.right_stick_y) > 0.1){ //strafing method
            fL.setPower(gamepad1.right_stick_y);
            bL.setPower(-gamepad1.right_stick_y);
            fR.setPower(gamepad1.right_stick_y);
            bR.setPower(-gamepad1.right_stick_y);
        }*/
        if(Math.abs(gamepad1.left_stick_x) > 0.1|| Math.abs(gamepad1.left_stick_y) > 0.1){
            fL.setPower(-v1);
            fR.setPower(v2);
            bL.setPower(v3);// * .79);
            bR.setPower(-v4);// * .79);
        }
       else if(Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2){
            fL.setPower(v1);
            fR.setPower(-v2);
            bL.setPower(-v3);// * .79);
            bR.setPower(v4);// * .79);
        }
       else{
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);// * .79);
            bR.setPower(0);// * .79);
        }
    }
//Controls: Left stick y - turning, Right stick y - strafing, Right stick x - moving
}
