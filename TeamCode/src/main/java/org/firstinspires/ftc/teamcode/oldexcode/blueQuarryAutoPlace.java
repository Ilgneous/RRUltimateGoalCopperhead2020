package org.firstinspires.ftc.teamcode.oldexcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldexcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.oldexcode.Vision.BitMapVision;

@Autonomous(name="blueQuarryAutoPlace", group="12596")
public class blueQuarryAutoPlace extends LinearOpMode {

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

            // Middle Pathing
            robot.resetAngle();
            //robot.startPos = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (skyStonePos == "center") {

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(-2, 0.3, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.5); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(600);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(8, .7); // Pull Stone out
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

               if (Math.abs(correction) > 2)
                   robot.turnPID(90, .1/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(-95, 1, 7); // go to deposit 1st stone
                robot.strafeLeftGyro(10, .8, 3);
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.strafeRightGyro(10, .8);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(125.5, 1, 5); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
            //    correction = robot.correctAngle(90) ; //calculate angle needed to correct
           //     if (Math.abs(correction) > 2)
            //        robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.5); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(11, .7);
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .1/correction, 0, 0, 1);
                // Pull Stone out
                // correction = robot.correctAngle(90) ;
                // if (Math.abs(correction) > 2)
                //  robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(-90, 1, 7); // go to deposit 2nd stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone
                sleep(200);
                robot.grabberBUp();
                robot.gripBDown();
                sleep(200);
                robot.goStraightGyro(13, 0.8, 2); //park

            } else if (skyStonePos == "left") {

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone
                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(-11, 0.3, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.5); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(8, .8);
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .1/correction, 0, 0, 1);
                // Pull Stone out
             //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

               // if (Math.abs(correction) > 2)
                 //   robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(-85, 1, 7); // go to deposit 1st stone
                robot.strafeLeftGyro(10, .9, 3);
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.strafeRightGyro(10, .9);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(113.5, 1, 5); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
             //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
             //   if (Math.abs(correction) > 2)
               //     robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.5); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(8, .8); // Pull Stone out

                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .2/correction, 0, 0, 1);
             //   correction = robot.correctAngle(90) ;
             //   if (Math.abs(correction) > 2)
             //       robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(-91, 1, 7); // go to deposit 2nd stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(200);
                robot.grabberBUp();
                robot.gripBDown();
                sleep(200);
                robot.goStraightGyro(13, 0.9, 2); //park
                sleep(30000);
            } else { // right

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(3.5, 0.3, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.6); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(9, .8); // Pull Stone out
              //  correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

              //  if (Math.abs(correction) > 2)
              //      robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .2/correction, 0, 0, 1);

                robot.goStraightGyro(-104, 1, 7); // go to deposit 1st stone
                robot.strafeLeftGyro(10, .9, 3);
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.strafeRightGyro(9, .9);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(131.5, 1, 6); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
                //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
                //   if (Math.abs(correction) > 2)
                //  robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.6); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(9.5, .8); // Pull Stone out
             //   correction = robot.correctAngle(90) ;
             //   if (Math.abs(correction) > 2)
             //       robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out

                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .2/correction, 0, 0, 1);

                robot.goStraightGyro(-95, 1, 7); // go to deposit 2nd stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone
                sleep(200);
                robot.grabberBUp();
                robot.gripBDown();
                sleep(200);
                robot.goStraightGyro(11, 0.8, 2); //park
                sleep(30000);
            }
        }
        telemetry.addLine("done");
        telemetry.update();
    }
    }
