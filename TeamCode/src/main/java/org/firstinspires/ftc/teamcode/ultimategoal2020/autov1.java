package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

@Autonomous(name = "autov1", group = "Autonomous")
public class autov1 extends LinearOpMode {

    hardwareMap robot = new hardwareMap();
    WebCamVision vision = null;
    //ElapsedTime time = new ElapsedTime();
    //WebCamVision vision = new WebCamVision(this);

    // Testing
    @Override
    public void runOpMode() throws InterruptedException {

        //stuff that happens after init is pressed
        vision = new WebCamVision(this);
        robot.init(this);

        waitForStart();

        while (opModeIsActive()) {

            //call functions
            String colorVals = vision.rbgVals();
            telemetry.addLine(colorVals);
            telemetry.update();
            //robot.strafeLeft(20, 0.5);

            sleep(3000000);

            // strafe left
            // turn right
            //dump.setPosition(1);
            // go backwards
            // strafe left


        }

        telemetry.addLine("done");
        telemetry.update();
    }
}

    //create functions
/*
    public void go_straight (double distance, double pwr) {
        double startVal = fL.getCurrentPosition();

        while ((Math.abs(fL.getCurrentPosition() - startVal) < distance) && opModeIsActive()) {
            fL.setPower(pwr);
            fR.setPower(pwr);
            bL.setPower(pwr);
            bR.setPower(pwr);
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void go_straight_gyro (double distance, double pwr) {
        gyro.resetZAxisIntegrator();
        double startVal = fL.getCurrentPosition();

        while ((Math.abs(fL.getCurrentPosition() - startVal) < distance) && opModeIsActive()) {
            if (gyro.getIntegratedZValue() > 1) {
                fL.setPower(pwr * 0.9);
                fR.setPower(pwr);
                bL.setPower(pwr * 0.9);
                bR.setPower(pwr);
            }

            else if (gyro.getIntegratedZValue() < 1) {
                fL.setPower(pwr);
                fR.setPower(pwr * 0.9);
                bL.setPower(pwr);
                bR.setPower(pwr * 0.9);
            }

            else {
                fL.setPower(pwr);
                fR.setPower(pwr);
                bL.setPower(pwr);
                bR.setPower(pwr);
            }
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void strafe_right (double distance, double pwr) {
        double startVal2 = fL.getCurrentPosition();

        while((Math.abs(fL.getCurrentPosition() - startVal2) < distance) && opModeIsActive()) {
            fL.setPower(pwr);
            fR.setPower(-pwr);
            bL.setPower(-pwr);
            bR.setPower(pwr);

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void strafe_right_gyro (double distance, double pwr) {
        gyro.resetZAxisIntegrator();
        double startVal2 = fL.getCurrentPosition();
    
        while((Math.abs(fL.getCurrentPosition() - startVal2) < distance) && opModeIsActive()) {
            if (gyro.getIntegratedZValue() > 1) {
                fL.setPower(pwr * 0.9);
                fR.setPower(-pwr);
                bL.setPower(-pwr * 0.9);
                bR.setPower(pwr);
            }

            else if (gyro.getIntegratedZValue() < 1) {
                fL.setPower(pwr);
                fR.setPower(-pwr * 0.9);
                bL.setPower(-pwr);
                bR.setPower(pwr * 0.9);
            }

            else {
                fL.setPower(pwr);
                fR.setPower(-pwr);
                bL.setPower(-pwr);
                bR.setPower(pwr);
            }

        }
    
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    
    public void turnBasic_right(double turn, double pwr) {
        gyro.resetZAxisIntegrator();

        while ((gyro.getIntegratedZValue() < turn) && opModeIsActive()) {
                fL.setPower(pwr);
                fR.setPower(-pwr);
                bL.setPower(pwr);
                bR.setPower(-pwr);

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void turnBasic_left(double turn, double pwr) {
        gyro.resetZAxisIntegrator();

        while ((gyro.getIntegratedZValue() > turn) && opModeIsActive()) {
            fL.setPower(-pwr);
            fR.setPower(pwr);
            bL.setPower(-pwr);
            bR.setPower(pwr);

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    
    public void turnPID(double angle, double pwr, double d) {
        gyro.resetZAxisIntegrator();
        time.reset();

        double deltaAngle = Math.abs(angle - gyro.getHeading());
        double pastdeltaAngle = deltaAngle;
        double currentTime;
        double kP = pwr / angle;
        double kI = 0.01;
        double kD = d / angle;
        double prevTime = 0;
        double apply = 0;
        double deltaTime;

        while (Math.abs(deltaAngle) > 1){
            deltaAngle = Math.abs(angle - gyro.getHeading());
            kP = deltaAngle * kP;
            currentTime = time.milliseconds();
            deltaTime =  currentTime - prevTime;
            kI = deltaAngle * deltaTime * kI;
            kD = (deltaAngle - pastdeltaAngle) / deltaTime * kD;
            apply = kP + kI + kD;

            fL.setPower(-apply);
            fR.setPower(apply);
            bL.setPower(-apply);
            bR.setPower(apply);

            prevTime = currentTime;
            pastdeltaAngle = deltaAngle;
        }
    }

    public void straightPID(double dist, double pwr, double d) {
        gyro.resetZAxisIntegrator();
        //reset();
        time.reset();


        double avgEncoder = (fL.getCurrentPosition() + fR.getCurrentPosition()) / 2;
        double gyroFix = 0;
        double deltaDist = Math.abs(dist - avgEncoder);
        double pastdeltaDist = deltaDist;
        double currentTime;
        double kP = pwr / dist;
        double kI = 0.01;
        double kD = d / dist;
        double prevTime = 0;
        double apply = 0;
        double deltaTime;

        while (Math.abs(deltaDist) > 1){
            avgEncoder = (fL.getCurrentPosition() + fR.getCurrentPosition()) / 2;
            deltaDist = Math.abs(dist - avgEncoder);
            kP = deltaDist * kP;
            currentTime = time.milliseconds();
            deltaTime =  currentTime - prevTime;
            kI = deltaDist * deltaTime * kI;
            kD = (deltaDist - pastdeltaDist) / deltaTime * kD;
            apply = kP + kI + kD;

            if (gyro.getHeading() < 1) {
                gyroFix = 0;
                fL.setPower(apply);
                fR.setPower(apply);
                bL.setPower(apply);
                bR.setPower(apply);
            }

            else if (gyro.getHeading() > 1 && gyro.getHeading() < 60) {
                gyroFix += .001;
                fL.setPower(apply);
                fR.setPower(apply + gyroFix);
                bL.setPower(apply);
                bR.setPower(apply + gyroFix);
            }

            else if (gyro.getHeading() < 359 && gyro.getHeading() > 300) {
                gyroFix += .001;
                fL.setPower(apply + gyroFix);
                fR.setPower(apply);
                bL.setPower(apply + gyroFix);
                bR.setPower(apply);
            }

            prevTime = currentTime;
            pastdeltaDist = deltaDist;
        }


    }
}*/


