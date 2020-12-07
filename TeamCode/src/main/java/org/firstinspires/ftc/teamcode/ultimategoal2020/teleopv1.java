
package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "teleOpv1", group = "TeleOp")
public class teleopv1 extends OpMode {
    hardwareMap hw = new hardwareMap();
    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    /*DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor intake;
    DcMotor shooter;
    Servo wobble;
    Servo elevator;*/
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        trigMecanum();
    }

    @Override
    public void stop() {
        //ZA WARUDO! TOKI WO TOMARE!
    }

    public void trigMecanum() { //turning and moving method
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

        telemetry.addData("fL", v1);
        telemetry.addData("fR", v2);
        telemetry.addData("bL", v3);
        telemetry.addData("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData("RightX", rightX);
        telemetry.update();

        if (Math.abs(gamepad1.right_stick_y) > 0.1) { //strafing method
            hw.fL.setPower(gamepad1.right_stick_y);
            hw.bL.setPower(-gamepad1.right_stick_y);
            hw.fR.setPower(gamepad1.right_stick_y);
            hw.bR.setPower(-gamepad1.right_stick_y);
        }

        hw.fL.setPower(-v1);
        hw.fR.setPower(-v2);
        hw.bL.setPower(v3);// * .79);
        hw.bR.setPower(v4);// * .79);
//test servo and motor values
        if (gamepad2.a = true) {
            hw.wobble.setPosition(0.5);
            telemetry.addData("wobble position", hw.wobble.getPosition());
            telemetry.update();
        }
        if (gamepad2.b = true) {
            hw.wobble.setPosition(0);
            telemetry.addData("wobble position", hw.wobble.getPosition());
            telemetry.update();
        }
        if (gamepad2.x = true) {
            hw.elevator.setPosition(0.5);
            telemetry.addData("elevator position", hw.elevator.getPosition());
            telemetry.update();
        }
        if (gamepad2.y = true) {
            hw.elevator.setPosition(0);
            telemetry.addData("elevator position", hw.elevator.getPosition());
            telemetry.update();
        }
        if (gamepad1.right_trigger > 0.1) {
            hw.shooter.setPower(gamepad1.right_trigger);
            telemetry.addData("shooter", hw.shooter.getPower());
            telemetry.update();
        } else {
            hw.shooter.setPower(0);
            telemetry.addData("shooter", hw.shooter.getPower());
            telemetry.update();
        }
        if(gamepad2.left_trigger > 0.1){
            hw.intake.setPower(0);
            telemetry.addData("shooter", hw.shooter.getPower());
            telemetry.update();
        }
    }
}
