package org.firstinspires.ftc.teamcode.oldexcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldexcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.oldexcode.Vision.BitMapVision;

@Autonomous(name="blueQuarryAuto", group="12596")
public class blueQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;
    String skyStonePos = null;
    RobotHw robot = new RobotHw();
    double correction = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        bm1 = new BitMapVision(this);
        robot.init(this);

        while (!isStarted())
        {
            skyStonePos = bm1.findBlueSkystones();
            telemetry.addData("stone", skyStonePos);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Reset tracking angle
            robot.resetAngle();
            robot.grabberBDown();
            robot.gripBUp();
            //Move to the 1st stone
            robot.goStraightGyro(18, .7, 3);
            // Rotate to align grabberR with stone
            robot.turnPID(90, .78/90, 0, .2, 10);
            // Align with center stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(-1.5, 0.2, 1);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-9.5, 0.2, 3);
            } else {
                robot.goStraightGyro(3.5, 0.2, 3);
            }
            //prime grabber and grip for stone

            // reset encoders for backtracking
            robot.reset();
            // Approach stone
            robot.approachStonesBlue(.4);
            //grab stone
            robot.gripBDown();
            sleep(1000);
            //pick up stone
            robot.grabberBUp();
            // Pull Stone out
            robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 6, .6);
            //calculate angle needed to correct
            correction = robot.correctAngle(90) ;
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .14/correction, 0, 0, 1);

            // Testing telemetry
            /*
            telemetry.addData("start angle: ", startPos.firstAngle);
            telemetry.addData("angle : ", robot.getAngle());
            telemetry.addData("correct angle : ", correction);
            telemetry.update();
            */

            // Drive to deposit 1st stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(-60, 1, 7);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-54, 1, 7);
            } else {
                robot.goStraightGyro(-66, 1, 7);
            }
            robot.strafeLeftGyro(4, .5, 5);
            // grabberR lets go of stone
            robot.grabberBDown();
            robot.gripBUp();
            sleep(300);
            //put grabber up to go back to quarry
            robot.gripBDown();
            robot.grabberBUp();
            robot.strafeRightGyro(3, .5);
            //calculate angle needed to correct
            correction = robot.correctAngle(90);
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .14/correction, 0, 0, 1);
            //go back to quarry
            if (skyStonePos == "center") {
                robot.goStraightGyro(91, 1, 5);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(86, 1, 5);
            } else {
                robot.goStraightGyro(100.5, 1, 5);
            }
            // Prime grabber to get stone
            robot.gripBUp();
            sleep(100);
            robot.grabberBDown();
            sleep(200);
            //calculate angle needed to correct
            correction = robot.correctAngle(90);
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .14/correction, 0, 0, 1);
            //approach 2nd stone
            robot.approachStonesBlue(.4);
            //grab 2nd stone
            robot.gripBDown();
            sleep(500);
            robot.grabberBUp();
            // Pull Stone out
            robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 5, .6);
            // Correct robot angle
            correction = robot.correctAngle(90) ;
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .15/correction, 0, 0, 1); // Straighten out
            // go to deposit 2nd stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(-90, 1, 7);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-88, 1, 7);
            } else {
                robot.goStraightGyro(-96, 1, 7);
            }
            robot.strafeLeftGyro(4, .5, 5);
            // grabberR lets go of stone
            robot.grabberBDown();
            robot.gripBUp();
            sleep(200);
            // Grabber folds back up
            robot.grabberBUp();
            robot.gripBDown();
            sleep(200);
            robot.strafeRightGyro(3, .5);
            //calculate angle needed to correct
            correction = robot.correctAngle(90);
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .14/correction, 0, 0, 1);
            //park
            if (skyStonePos == "center") {
                robot.goStraightGyro(13, 0.7, 2);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(21, 1, 4);
            } else {
                robot.goStraightGyro(13, 0.7, 2);
            }
            sleep(30000);




        }
        telemetry.addLine("done");
        telemetry.update();
    }
    }
