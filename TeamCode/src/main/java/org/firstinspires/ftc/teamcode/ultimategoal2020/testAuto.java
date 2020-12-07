package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "testAuto", group = "Autonomous")
public class testAuto extends LinearOpMode {
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    public BNO055IMU imu;
    public double globalAngle;
    //GyroSensor gyro = null;
    ElapsedTime time = new ElapsedTime();
    public Orientation lastAngles = new Orientation();
    public void runOpMode(){
        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        //gyro = hardwareMap.gyroSensor.get("gyro");
        waitForStart();
      //  moveStraight(1000, 0.5);
        while(opModeIsActive()){
            //strafe(1000, 0.5);
            go_straight_gyro(500, 0.5);
        }
        telemetry.addLine("go team");
        telemetry.addLine("hi");
        telemetry.addData("fL position:", fL.getCurrentPosition());
        telemetry.update();

    }
    public void moveStraight(double distance, double power) {
        double startingValue = fL.getCurrentPosition();
        while ((Math.abs(fL.getCurrentPosition() - startingValue) < distance) && opModeIsActive()) {
            fL.setPower(-power*0.8);
            fR.setPower(power*0.9);
            bL.setPower(-power*0.9);
            bR.setPower(power*0.8);
        }
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return -globalAngle;
    }

    public void go_straight_gyro (double distance, double pwr) {
        resetAngle();
        double startVal = fL.getCurrentPosition();

        while ((Math.abs(fL.getCurrentPosition() - startVal) < distance) && opModeIsActive()) {
            if (getAngle() > 1) {
                fL.setPower(pwr * 0.9);
                fR.setPower(pwr);
                bL.setPower(pwr * 0.9);
                bR.setPower(pwr);
            }

            else if (getAngle() < 1) {
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
    public void turn(double turn, double power){

    }
    public void strafe(double distance, double power){
       double startingValue = fL.getCurrentPosition();
       while((Math.abs(fL.getCurrentPosition() - startingValue) < distance) && opModeIsActive()){
           fL.setPower(-power*0.9);
           fR.setPower(-power*0.9);
           bL.setPower(power*0.93);
           bR.setPower(power);
       }
    }
}
