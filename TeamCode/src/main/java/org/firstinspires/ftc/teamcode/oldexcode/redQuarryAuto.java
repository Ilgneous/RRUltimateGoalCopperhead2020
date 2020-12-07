package org.firstinspires.ftc.teamcode.oldexcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldexcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.oldexcode.Vision.BitMapVision;

@Autonomous(name="redQuarryAuto", group="12596")
public class redQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;
    String skyStonePos = null;
    RobotHw robot = new RobotHw();
    double correction = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        bm1 = new BitMapVision(this); // Init bitmapvision
        robot.init(this); // init robot

        while (!isStarted())
        {
            skyStonePos = bm1.findRedSkystones();
            telemetry.addData("stone", skyStonePos);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Reset tracking angle
            robot.resetAngle();
            //Move to the 1st stone
            robot.goStraightGyro(18, .7, 3);
            // Rotate to align grabberR with stone
            robot.turnPID(90, .78/90, 0, .2, 10);
            // Align with center stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(2, 0.2, 1);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-5, 0.2, 2);
            } else { // right
                robot.goStraightGyro(14, 0.2, 3);
            }
            //prime grabber and grip for stone
            robot.gripRUp();
            robot.grabberRDown();
            sleep(500);
            // reset encoders for backtracking
            robot.reset();
            // Approach stone
            robot.approachStonesRed(.5);
            //grab stone
            robot.gripRDown();
            sleep(1000);
            //pick up stone
            robot.grabberRUp();
            // Pull Stone out
            robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 4.5, .6);
            //calculate angle needed to correct
            correction = robot.correctAngle(90) ;
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .13/correction, 0, 0, 1);

            // Testing telemetry
            /*
            telemetry.addData("start angle: ", startPos.firstAngle);
            telemetry.addData("angle : ", robot.getAngle());
            telemetry.addData("correct angle : ", correction);
            telemetry.update();
            */

            // Drive to deposit 1st stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(60, 1, 7);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(66, 1, 7);
            } else {
                robot.goStraightGyro(55, 1, 7);
            }
            robot.strafeLeftGyro(4, .5, 5);
            // grabberR lets go of stone
            robot.grabberRDown();
            robot.gripRUp();
            sleep(300);
            //put grabber up to go back to quarry
            robot.gripRDown();
            robot.grabberRUp();
            robot.strafeRightGyro(4, .5);
            //go back to quarry
            if (skyStonePos == "center") {
                robot.goStraightGyro(-89.5, 1, 5);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-94.5, 1, 5);
            } else {
                robot.goStraightGyro(-84.5, 1, 5);
            }
            // Prime grabber to get stone
            robot.gripRUp();
            sleep(100);
            robot.grabberRDown();
            //calculate angle needed to correct
            correction = robot.correctAngle(90);
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .13/correction, 0, 0, 1);
            //approach 2nd stone
            robot.approachStonesRed(.5);
            //grab 2nd stone
            robot.gripRDown();
            sleep(500);
            robot.grabberRUp();
            // Pull Stone out
            robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 4.5, .6);
            // Correct robot angle
            correction = robot.correctAngle(90) ;
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .13/correction, 0, 0, 1); // Straighten out
            // go to deposit 2nd stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(90, 1, 7);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(99, 1, 7);
            } else {
                robot.goStraightGyro(85, 1, 7);
            }
            robot.strafeLeftGyro(4, .5, 5);
            // grabberR lets go of stone
            robot.grabberRDown();
            robot.gripRUp();
            sleep(200);
            // Grabber folds back up
            robot.grabberRUp();
            robot.gripRDown();
            sleep(200);
            robot.strafeRightGyro(4, .5);
            //park
            if (skyStonePos == "center") {
                robot.goStraightGyro(-11, 0.7, 2);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-13, 0.7, 2);
            } else {
                robot.goStraightGyro(-15, 0.7, 2);
            }
            sleep(30000);
        }
        telemetry.addLine("done");
        telemetry.update();
    }
}