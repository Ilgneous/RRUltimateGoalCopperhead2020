package org.firstinspires.ftc.teamcode.oldexcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.oldexcode.Movement.RobotHw;


@TeleOp(name="Skystone TeleOP", group="Pushbot")

// @ AUTHOR ROBERT HAYDEN WARREN

public class SkystoneTeleop extends LinearOpMode{

    RobotHw robot = new RobotHw();

    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    public double chungusStartPos;
    public double speed = 1;
    public double rotationSpeed = 1;

    public int clampcount = 1;
    public int constant = 1;

    public boolean teleop = true;
    public boolean clawdown = false;
    public boolean grabberR = false;
    public boolean grabberB = false;

    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime robert = new ElapsedTime();

    // AUTHOR ROBERT HAYDEN WARREN
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, teleop);
        ElapsedTime runTime = new ElapsedTime();
        //robot.brakeMode();
        runTime.reset();
        chungusStartPos = robot.liftRotate.getCurrentPosition();
        robot.liftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        runtime.reset();
        robert.reset();

        while (opModeIsActive()) {
            // Sin Cos Atan inputs for mecanum
            trigMecanum();
            // telemetry.addData("start", chungusStartPos);
            telemetry.addData("rotation speed", rotationSpeed * .5);
            telemetry.update();

            // Foundation Clamp
            if (gamepad1.y) { //up
                robot.clamp.setPosition(0.2);
            }
            if (gamepad1.a) { //down
                robot.clamp.setPosition(1);
            }

            // Claw
            if (gamepad2.right_bumper && runtime.seconds() > 1) {
                if (!clawdown) {
                    robot.clawDown(); // down
                    clawdown = true;
                }
                else {
                    robot.clawUp(); // up
                    clawdown = false;
                }
                runtime.reset();
            }

            // Red Grabber stone pick up
            if (gamepad1.x && robert.seconds() > 1){
                if (!grabberB) {
                    robot.grabberRDown();
                    robot.gripRUp();
                    grabberB = true;
                }
                else {
                    robot.gripRDown();
                    sleep(500);
                    robot.grabberRUp();

                    grabberR = false;
                }
                robert.reset();
            }

            // Red Grabber pinch and bring up stone
            if(gamepad1.b && robert.seconds() > 1){
                if (!grabberR) {
                    robot.grabberBDown();
                    robot.gripBUp();
                    grabberR = true;
                }
                else {
                    robot.gripBDown();
                    sleep(500);
                    robot.grabberBUp();
                    grabberR = false;

                }
                robert.reset();
            }

            // Reverse Mode
            if (gamepad1.dpad_down){
                constant = -1; // Reverse
            }
            if (gamepad1.dpad_up){
                constant = 1; // Forward
            }

            //intake
            if (gamepad1.right_trigger > .1){
                robot.intakeL.setPower(gamepad1.right_trigger);
                robot.intakeR.setPower(-gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > .1){
                robot.intakeL.setPower(-gamepad1.left_trigger);
                robot.intakeR.setPower(gamepad1.left_trigger);
            }
            else{
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }

            // Lift
            if(gamepad2.a && runtime.seconds() > 1) {
                if (rotationSpeed == 1){
                    rotationSpeed = .5;
                }

                else {
                    rotationSpeed = 1;
                }
                runtime.reset();
            }

            if ((gamepad2.left_stick_y < 0))
            {
                robot.liftRotate.setPower(-0.5 * gamepad2.left_stick_y * rotationSpeed);
            }

            else if (gamepad2.left_stick_y > 0 && (robot.liftRotate.getCurrentPosition() > chungusStartPos))
            {
                robot.liftRotate.setPower(-0.5 * gamepad2.left_stick_y * rotationSpeed);
            }

            else{
                robot.liftRotate.setPower(0);
            }

            robot.liftExtend.setPower(gamepad2.right_stick_y);

            if(gamepad2.x){
                // telemetry.addLine(("first stage"));
                // telemetry.update();
                double aboveOrBelow = robot.liftRotate.getCurrentPosition() - 500; //-200
                double makeItOne = 1 / (Math.abs(aboveOrBelow)); // 1/200
                double power = -0.5;
                // telemetry.addData("1", aboveOrBelow);
                // telemetry.addData("2", makeItOne);
                // telemetry.addData("3", power);
                // telemetry.addLine(("2nd stage"));
                // telemetry.addData("calc", aboveOrBelow * makeItOne * power);
                // telemetry.update();
                robot.liftRotate.setPower(aboveOrBelow * makeItOne * power);
            }

            //Slowmode drivetrain
            if (gamepad1.right_bumper) {
                speed = 1;
            }
            if (gamepad1.left_bumper) {
                speed = .5;
            }

            if(gamepad2.y){
                double aboveOrBelow = robot.liftRotate.getCurrentPosition() - 750; //-200
                double makeItOne = 1 / (Math.abs(aboveOrBelow)); // 1/200
                double power = -0.5;
                // telemetry.addData("1", aboveOrBelow);
                // telemetry.addData("2", makeItOne);
                // telemetry.addData("3", power);
                // telemetry.addLine(("2nd stage"));
                // telemetry.addData("calc", aboveOrBelow * makeItOne * power);
                // telemetry.update();
                robot.liftRotate.setPower(aboveOrBelow * makeItOne * power);
            }

            if(gamepad2.b){

                double aboveOrBelow = robot.liftRotate.getCurrentPosition() - 1000; //-200
                double makeItOne = 1 / (Math.abs(aboveOrBelow)); // 1/200
                double power = -0.5;
                // telemetry.addData("1", aboveOrBelow);
                // telemetry.addData("2", makeItOne);
                // telemetry.addData("3", power);
                // telemetry.addLine(("2nd stage"));
                // telemetry.addData("calc", aboveOrBelow * makeItOne * power);
                // telemetry.update();
                robot.liftRotate.setPower(aboveOrBelow * makeItOne * power);
            }
        }
    }

    public void trigMecanum() {

        rightstickx = Math.abs(gamepad1.right_stick_x) * -gamepad1.right_stick_x ;
        leftstickx = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * constant;

        leftstickyfront = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y * constant;
        leftstickyback = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y * -constant;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX;
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

        /*
        telemetry.addData("fl", v1);
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData ("Right X", rightX);
        telemetry.update();
        */

        robot.fL.setPower(-v1 * speed);
        robot.fR.setPower(-v2 * speed);
        robot.bL.setPower(v3 * speed);// * .79);
        robot.bR.setPower(v4 * speed);// * .79);
    }
}
