package org.firstinspires.ftc.teamcode.oldexcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldexcode.Movement.RobotHw;

@Autonomous(name="redFoundationAuto", group="12596")
public class redFoundationAuto extends LinearOpMode {
    RobotHw robot = new RobotHw();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);


//        waitForStart();
//        robot.strafeRightGyro(48, 1);
//        //Strafe right 4ft
//
//        robot.grabberRDown();
//        robot.grabberBDown();
//        //servos down
//
//        robot.strafeLeftGyro(48, 1, 5);
//        //strafe left 4ft
//
//        robot.grabberRUp();
//        robot.grabberBUp();
//        //servos up
//
//        robot.goStraightGyro(60,1, 5);
//        //move forward 5ft

        waitForStart();
        robot.goStraightGyro(12, .7, 3);
        robot.strafeLeftGyro(30, .5, 8);
        //Strafe right 4ft

        robot.grabberRDown();
        robot.grabberBDown();
        sleep(2000);
        //servos down
        robot.goStraightGyro(4,.7, 3);

        //robot.turnPID(90,.3,0,0,1);
        robot.strafeRightGyro(33, 0.7);

        // servos up
        robot.grabberRUp();
        robot.grabberBUp();
        sleep(2000);

        robot.goStraightGyro(-20, 1, 5);
        robot.strafeLeftGyro(4, 0.5, 8);
        robot.goStraightGyro(-45, 1, 5);




    }

}
