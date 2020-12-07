package org.firstinspires.ftc.teamcode.oldexcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldexcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.oldexcode.Vision.BitMapVision;

@Autonomous(name="redQuarryAutoPlace", group="12596")
public class redQuarryAutoPlace extends LinearOpMode {

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
            skyStonePos = bm1.findRedSkystones();
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
                robot.goStraightGyro(1, 0.2, 1); // Align with center stone
                robot.gripRUp(); //prime grabber and grip for stone
                robot.grabberRDown();
                sleep(500);
                robot.approachStonesRed(.4); // Approach stone
                robot.gripRDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberRUp(); //pick up stone
                robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 3, .6); // Pull Stone out
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(95, 1, 7); // go to deposit 1st stone
                robot.strafeLeftGyro(10, .6, 3);
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone

                sleep(300);
                robot.strafeRightGyro(10, .7);
                robot.gripRDown(); //put grabber up to go back to quarry
                robot.grabberRUp();
                robot.goStraightGyro(-120.5, 1, 5); //go back to quarry
                robot.gripRUp();
                sleep(100);
                robot.grabberRDown();
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesRed(.4); //approach 2nd stone
                robot.gripRDown(); //grab 2nd stone
                sleep(500);
                robot.grabberRUp();
                robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 3, .7); // Pull Stone out
                correction = robot.correctAngle(90) ;
                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(90, 1, 7); // go to deposit 2nd stone
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone
                sleep(200);
                robot.grabberRUp();
                robot.gripRDown();
                sleep(200);
                robot.goStraightGyro(-13, 0.8, 2); //park
                sleep(30000);
                //robot.alignStonesR(.13);

//                //second stone
//                robot.goStraightGyro(91, .4, 5); // Go back to the stones
//                sleep(500);
//                robot.strafeRightGyro(10.5, .25); // go in to get 2nd stone
//                //robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
//                robot.goStraightGyro(-.6, .2, .2);
//                sleep(1000);
//                robot.strafeLeft(10.5, .5); // Pull stone out
//                robot.goStraightGyro(-100, .7, 7); // Cross the bridge
//               // robot.grabberR.setPosition(.5); // Drop the stone
//                sleep(500);
//                robot.strafeRight(3, .3);
//                robot.goStraightGyro(26, .5, 3); // park
//                sleep(30000);


            } else if (skyStonePos == "left") {

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(-5, 0.3, 1); // Align with center stone
                robot.gripRUp(); //prime grabber and grip for stone
                robot.grabberRDown();
                sleep(500);
                robot.approachStonesRed(.4); // Approach stone
                robot.gripRDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberRUp(); //pick up stone
                robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 3, .8); // Pull Stone out
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(105, 1, 7); // go to deposit 1st stone
                robot.strafeLeftGyro(12, .7, 3);
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone

                sleep(500);
                robot.strafeRightGyro(10, .8);
                robot.gripRDown(); //put grabber up to go back to quarry
                robot.grabberRUp();
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .2/correction, 0, 0, 1); // Straighten out

                robot.goStraightGyro(-128.5, 1, 5); //go back to quarry
                robot.gripRUp();
                sleep(100);
                robot.grabberRDown();
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesRed(.4); //approach 2nd stone
                robot.gripRDown(); //grab 2nd stone
                sleep(500);
                robot.grabberRUp();
                robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 3, .7); // Pull Stone out
                correction = robot.correctAngle(90) ;
                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(97, 1, 7); // go to deposit 2nd stone
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone

                sleep(200);
                robot.grabberRUp();
                robot.gripRDown();
                sleep(200);
                robot.goStraightGyro(-10, 0.9, 2); //park
                sleep(30000);
            } else { // right

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(9.5, 0.3, 3); // Align with center stone
                robot.gripRUp(); //prime grabber and grip for stone
                robot.grabberRDown();
                sleep(500);
                robot.approachStonesRed(.4); // Approach stone
                robot.gripRDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberRUp(); //pick up stone
                robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 3, .7); // Pull Stone out
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(90, 1, 7); // go to deposit 1st stone
                robot.strafeLeftGyro(10, .7, 3);
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone

                sleep(300);
                robot.strafeRightGyro(10, .7);
                robot.gripRDown(); //put grabber up to go back to quarry
                robot.grabberRUp();
                robot.goStraightGyro(-115.5, 1, 5); //go back to quarry
                robot.gripRUp();
                sleep(100);
                robot.grabberRDown();
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesRed(.4); //approach 2nd stone
                robot.gripRDown(); //grab 2nd stone
                sleep(500);
                robot.grabberRUp();
                robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 3, .7); // Pull Stone out
                correction = robot.correctAngle(90) ;
                if (Math.abs(correction) > 2)
                    robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(85, 1, 7); // go to deposit 2nd stone
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone
                sleep(200);
                robot.grabberRUp();
                robot.gripRDown();
                sleep(200);
                robot.goStraightGyro(-13, 0.8, 2); //park
                sleep(30000);
            }
        }
        telemetry.addLine("done");
        telemetry.update();
    }


}