package org.firstinspires.ftc.teamcode.oldexcode.Movement;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



public class RobotHw {

    // Motors
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;
    public DcMotor intakeR = null;
    public DcMotor intakeL = null;
    public DcMotor liftExtend = null;
    public DcMotor liftRotate = null;
    // Servos
    //public Servo clip = null;
    public Servo claw = null;
    public Servo rotate = null;
    public Servo clamp = null;
    public Servo grabberR = null;
    public Servo grabberB = null;
    public Servo gripB = null;
    public Servo gripR = null;

    // HardwareMap
    HardwareMap hwMap;

    // Linear Opmode
    LinearOpMode opmode;

    // Time
    public ElapsedTime runtime = new ElapsedTime();

    // Tick Conversion
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public Orientation startPos = null;
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;

    public double leftCorrect;
    public double rightCorrect;

    public ColorSensor sensorColorBMid;
    public ColorSensor sensorColorBEdge;
    //public ColorSensor sensorColorBotFront;
    public ColorSensor sensorColorRMid;
    public ColorSensor sensorColorREdge;
    public ColorSensor sensorColorBotBack;

    public DistanceSensor sensorDistanceBMid;
    public DistanceSensor sensorDistanceBEdge;
    public DistanceSensor sensorDistanceRMid;
    public DistanceSensor sensorDistanceREdge;

    public double degreesToTicks;


    // Initialize Components
    public void init(LinearOpMode lOpmode) {
        opmode = lOpmode;
        // Hardware map
        hwMap = opmode.hardwareMap;

        degreesToTicks = 0; //add in actual conversion

        // Define and Initialize Motors
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");

/*        intakeR = hwMap.get(DcMotor.class, "intakeR");
        intakeL = hwMap.get(DcMotor.class, "intakeL");

        liftExtend = hwMap.get(DcMotor.class, "liftExtend");
        liftRotate = hwMap.get(DcMotor.class, "liftRotate");

        //Define and initialize servos
        //clip = hwMap.get(Servo.class, "clip");
        claw = hwMap.get(Servo.class, "claw");
        rotate = hwMap.get(Servo.class, "rotate");
        clamp = hwMap.get(Servo.class, "clamp");
        grabberR = hwMap.get(Servo.class, "grabberR");
        grabberB = hwMap.get(Servo.class, "grabberB");
        gripB = hwMap.get(Servo.class, "gripB");
        gripR = hwMap.get(Servo.class, "gripR");
*/
        degreesToTicks = 560.0 / 360.0;

        //set direction of motors
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);

        initColor();
        //intakeL.setDirection(DcMotor.Direction.rotate);
        //intakeR.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders, and run without encoders
        reset();

        // Set motor powers to zero
        stopMotors();


        // Set all motors to zero power
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        //driveTrain.srvMarker.setPosition(1);


        opmode.telemetry.addData("Mode", "calibrating...");
        opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
            opmode.sleep(50);
            opmode.idle();
        }

        opmode.telemetry.addData("Mode", "waiting for start");
        opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opmode.telemetry.update();

        opmode.telemetry.addData("fl", fL.getCurrentPosition());
        opmode.telemetry.addData("fr", fR.getCurrentPosition());
        opmode.telemetry.addData("bl", bL.getCurrentPosition());
        opmode.telemetry.addData("br", bR.getCurrentPosition());
        opmode.telemetry.update();

    }
    public void init(LinearOpMode lOpmode, Boolean teleop) {
        opmode = lOpmode;
        // Hardware map
        hwMap = opmode.hardwareMap;

        // Define and Initialize Motors
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");

        intakeR = hwMap.get(DcMotor.class, "intakeR");
        intakeL = hwMap.get(DcMotor.class, "intakeL");

        liftExtend = hwMap.get(DcMotor.class, "liftExtend");
        liftRotate = hwMap.get(DcMotor.class, "liftRotate");

        //Define and initialize servos
        //clip = hwMap.get(Servo.class, "clip");
        claw = hwMap.get(Servo.class, "claw");

        rotate = hwMap.get(Servo.class, "rotate");

        clamp = hwMap.get(Servo.class, "clamp");

        grabberR = hwMap.get(Servo.class, "grabberR");
        grabberB = hwMap.get(Servo.class, "grabberB");

        gripB = hwMap.get(Servo.class, "gripB");
        gripR = hwMap.get(Servo.class, "gripR");

        //set direction of motors
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders, and run without encoders
        reset();

        // Set motor powers to zero
        stopMotors();

        // Set all motors to zero power
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opmode.telemetry.addData("fl", fL.getCurrentPosition());
        opmode.telemetry.addData("fr", fR.getCurrentPosition());
        opmode.telemetry.addData("bl", bL.getCurrentPosition());
        opmode.telemetry.addData("br", bR.getCurrentPosition());
        opmode.telemetry.update();

    }

    public double atTarget(double distance) {
        return Math.abs(distance * COUNTS_PER_INCH);
    }

    public void moveStraight(double distance, double power) {
        dtEncoderModeOn();
        int startVal = fL.getCurrentPosition();
        if (fL.getCurrentPosition() != 0) {
            while (Math.abs(fL.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        } else if (fR.getCurrentPosition() != 0) {
            while (Math.abs(fR.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        } else if (bL.getCurrentPosition() != 0) {
            while (Math.abs(bL.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        } else if (bR.getCurrentPosition() != 0) {
            while (Math.abs(bR.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        }
    }

    // Set motors to zero power
    public void stopMotors() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void grabberBDown() {
        grabberR.setPosition(0);
    }

    public void grabberBUp() {
        grabberR.setPosition(1);
    }

    public void grabberRDown() {
        grabberB.setPosition(.72);
    }

    public void grabberRUp() {
        grabberB.setPosition(.27);
    }

    public void gripBDown(){
        gripB.setPosition(0);
    }

    public void gripBUp(){
        gripB.setPosition(.46);
    }

    public void gripRUp(){
        gripR.setPosition(0.02);
    }

    public void clawDown(){
        claw.setPosition(0);
    }

    public void clawUp(){
        claw.setPosition(.43);
    }

    public void gripRDown(){
        gripR.setPosition(.75);
    }


    // Strafe right
    public void strafeRight(double distance, double power) {
        // Reset encoders
        reset();
        // Set desired target position
        double target = Math.abs(distance * (537.6 / 11));
        // Set motors to brake
        brakeMode();
        // While robot is moving to desired position
        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
            /* debugging code, uncomment if needed
            opmode.telemetry.addData("avg", encoderAvg());
            opmode.telemetry.addData("fl", fL.getCurrentPosition());
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
            */
        }
        // Set motors to zero power
        stopMotors();
        // Set motors to float
        floatMode();
    }

    // Strafe left
    public void strafeLeft(double distance, double power) {
        // Reset encoders
        reset();
        // Set desired target position
        double target = Math.abs(distance * (537.6 / 11));
        // Set motors to brake
        brakeMode();
        // While robot is moving to desired position
        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);
            /* debugging code, uncomment if needed
            opmode.telemetry.addData("avg", encoderAvg());
            opmode.telemetry.addData("fl", fL.getCurrentPosition());
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
            */
        }
        // Set motors power to 0
        stopMotors();
        // Set motors to float
        floatMode();
    }

    // Reset Encoders, and set mode to run without encoders
    public void reset() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        liftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        liftExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        liftRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();

    }

    // Return avg of all 4 motor encoder values
    public double encoderAvg() {




        double avg = 0;

        // FR motor
        avg += Math.abs(fR.getCurrentPosition());
        // FL motor
        avg += Math.abs(fL.getCurrentPosition());
        //BL motor
        avg += Math.abs(bL.getCurrentPosition());
        //BR motor
        avg += Math.abs(bR.getCurrentPosition());
        return avg / 4;
    }

    // Turn on encoders
    public void dtEncoderModeOn() {
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set motors to freely rotate at zero power
    public void floatMode() {
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Set motors to hold position at zero power
    public void brakeMode() {
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goStraightGyro(double distance, double power, double timeout) {

        double speed = 0.0;

        //reset();
        resetAngle();
        opmode.sleep(100);
        double rightPower;
        // rotate
        if (distance > 0) {
            power = power - .15;

        } // reverse
        else {
            power = -power + .15;
        }

        double target = Math.abs(distance * (537.6/15.5));

        brakeMode();
        reset();
        dtEncoderModeOn();
        runtime.reset();
        if (distance > 0) {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < timeout) {
                speed = (.15 + (power * remainingDistance(distance)));
                //speed = 1;
                if (getAngle() > 1) {
                    fL.setPower(.9 * speed);
                    fR.setPower(1.0 * speed * .9793);
                    bL.setPower(.9 * speed);
                    bR.setPower(1.0 * speed);
                }
                else if (getAngle() < -1) {
                    fL.setPower(1.0 * speed);
                    fR.setPower(.9 * speed * .9793);
                    bL.setPower(1.0 * speed);
                    bR.setPower(.9 * speed);
                }
                else {
                    fL.setPower(speed);
                    fR.setPower(speed * .9793);
                    bL.setPower(speed);
                    bR.setPower(speed);// * .9793);
                }
                //opmode.telemetry.addData("avg : ", encoderAvg());
                //opmode.telemetry.addData("fl ticks : ", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr ticks : ", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl ticks : ", bL.getCurrentPosition());
                //opmode.telemetry.addData("br ticks : ", bR.getCurrentPosition());

                opmode.telemetry.addData("angle : ", getAngle());
                opmode.telemetry.addData("fl : ", fL.getPower());
                opmode.telemetry.addData("fr : ", fR.getPower());
                opmode.telemetry.addData("bl : ", bL.getPower());
                opmode.telemetry.addData("br : ", bR.getPower());
                opmode.telemetry.update();


            }
        }
        else {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < timeout) {
                speed = (-.15 + (power * remainingDistance(distance)));
                if (getAngle() > 1) {
                    fL.setPower(1.0 * speed);
                    fR.setPower(.9 * speed * .9793);
                    bL.setPower(1.0 * speed);
                    bR.setPower(.9 * speed);
                }
                else if (getAngle() < -1) {
                    fL.setPower(.9 * speed);
                    fR.setPower(1.0 * speed * .9793);
                    bL.setPower(.9 * speed);
                    bR.setPower(1.0 * speed);
                }
                else {
                    fL.setPower(speed);
                    fR.setPower(speed * .9793);
                    bL.setPower(speed);
                    bR.setPower(speed);// * .9793);
                }

                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                opmode.telemetry.addData("angle", getAngle());
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        }

        stopMotors();
        floatMode();
        resetAngle();
    }

    public void lineRed(double distance, double leftPower, double timeout) {


        //reset();
        resetAngle();
        opmode.sleep(100);
        double rightPower;
        // rotate
        if (distance > 0) {
            rightPower = leftPower ;
            rightPower = rightPower - .15;
            leftPower = leftPower -.15;

        } // reverse
        else {
            rightPower = leftPower ;
            rightPower = -rightPower + .15;
            leftPower = -leftPower + .15;
        }

        double target = Math.abs(distance * (537.6/15.5));

        brakeMode();
        reset();
        dtEncoderModeOn();
        runtime.reset();
        if (distance > 0) {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < timeout) {
                if (getAngle() > 1) {
                    fL.setPower(.8 * (.15 + (rightPower * remainingDistance(distance))));
                    fR.setPower(1.2 * (.15 + (leftPower * remainingDistance(distance))));
                    bL.setPower(.8 * (.15 + (leftPower * remainingDistance(distance))));
                    bR.setPower(1.2 * (.15 + (rightPower * remainingDistance(distance))));
                }
                else if (getAngle() < -1) {
                    fL.setPower(1.2 * (.15 + (rightPower * remainingDistance(distance))));
                    fR.setPower(.8 * (.15 + (leftPower * remainingDistance(distance))));
                    bL.setPower(1.2 * (.15 + (leftPower * remainingDistance(distance))));
                    bR.setPower(.8 * (.15 + (rightPower * remainingDistance(distance))));
                }
                else {
                    fL.setPower((.15 + (rightPower * remainingDistance(distance))));
                    fR.setPower((.15 + (leftPower * remainingDistance(distance))));
                    bL.setPower((.15 + (leftPower * remainingDistance(distance))));
                    bR.setPower((.15 + (rightPower * remainingDistance(distance))));
                }
                opmode.telemetry.addData("avg : ", encoderAvg());
                opmode.telemetry.addData("fl ticks : ", fL.getCurrentPosition());
                opmode.telemetry.addData("fr ticks : ", fR.getCurrentPosition());
                opmode.telemetry.addData("bl ticks : ", bL.getCurrentPosition());
                opmode.telemetry.addData("br ticks : ", bR.getCurrentPosition());

                opmode.telemetry.addData("angle : ", getAngle());
                opmode.telemetry.addData("fl : ", fL.getPower());
                opmode.telemetry.addData("fr : ", fR.getPower());
                opmode.telemetry.addData("bl : ", bL.getPower());
                opmode.telemetry.addData("br : ", bR.getPower());
                opmode.telemetry.update();


            }
        }
        else {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < timeout) {
                if (getAngle() > 1) {
                    fL.setPower(1.2 * (.15 + (-rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(1.2 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(.8 * (-.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else if (getAngle() < -1) {
                    fL.setPower(.8 * (.15 + (-rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(1.2 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(1.2 * (-.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else {
                    fL.setPower(.15 + (-rightPower - .15) * ((target - encoderAvg()) / target));
                    fR.setPower(-.15 + (leftPower * ((target - encoderAvg()) / target)));
                    bL.setPower(-.15 + (leftPower * ((target - encoderAvg()) / target)));
                    bR.setPower(-.15 + (rightPower * ((target - encoderAvg()) / target)));
                }

                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                opmode.telemetry.addData("angle", getAngle());
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        }

        stopMotors();

        floatMode();
        resetAngle();
    }


    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double remainingDistance(double distance) {
        double target = Math.abs(distance * (537.6/15.5));
        return (target - encoderAvg()) / target;
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

    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.

        double correction, angle, gain = .0;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }



    public void rotate(double degrees, double power) {


        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftPower, rightPower;


        // restart imu movement tracking.
        resetAngle();
        opmode.telemetry.addLine().addData("Robot Angle", getAngle());
        opmode.sleep(500);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > 0) {   // turn right.
            leftPower = power - .17;
            rightPower = -power + .17;
        } else if (degrees < 0) {   // turn left.
            leftPower = -power + .17;
            rightPower = power - .17;
        } else return;


        // set power to rotate.
        fL.setPower(leftPower);
        bL.setPower(leftPower);
        fR.setPower(rightPower);
        bR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On left turn we have to get off zero first.

            while (opmode.opModeIsActive() && getAngle() > degrees) {
                fL.setPower(-.17 + (leftPower * ((degrees - getAngle()) / degrees)));
                bL.setPower(-.17 + (leftPower * ((degrees - getAngle()) / degrees)));
                fR.setPower(.17 + (rightPower * ((degrees - getAngle()) / degrees)));
                bR.setPower(.17 + (rightPower * ((degrees - getAngle()) / degrees)));

                opmode.telemetry.addData("degrees", getAngle());
                opmode.telemetry.addData("lastAngle", lastAngles);
                opmode.telemetry.addData("globalangle", globalAngle);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        } else    // right turn.
            while (opmode.opModeIsActive() && getAngle() < degrees) {
                fL.setPower(.17 + (leftPower * ((degrees - getAngle()) / degrees)));
                bL.setPower(.17 + (leftPower * ((degrees - getAngle()) / degrees)));
                fR.setPower(-.17 + (rightPower * ((degrees - getAngle()) / degrees)));
                bR.setPower(-.17 + (rightPower * ((degrees - getAngle()) / degrees)));

                opmode.telemetry.addData("degrees", getAngle());
                //telemetry.addData("lastangle", lastAngles);
                //telemetry.addData("globalangle", globalAngle);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());

                opmode.telemetry.update();


            }

            //turn the motors off.
        stopMotors();
        lastDegrees = degrees;

        // wait for rotation to stop.
        opmode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void turnToHeading(double heading, double power) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftPower, rightPower;


        // restart imu movement tracking.
        resetAngle();
        opmode.telemetry.addLine().addData("Robot Angle", getAngle());
        opmode.sleep(500);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (heading > angles.firstAngle) {   // turn right.
            leftPower = power - .17;
            rightPower = -power + .17;
        } else if (heading < angles.firstAngle) {   // turn left.
            leftPower = -power + .17;
            rightPower = power - .17;
        } else return;

        double degrees = Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading));

        // set power to rotate.
        fL.setPower(leftPower);
        bL.setPower(leftPower);
        fR.setPower(rightPower);
        bR.setPower(rightPower);

        // rotate until turn is completed.
        if (heading < angles.firstAngle) {
            // On left turn we have to get off zero first.

            while (opmode.opModeIsActive() && angles.firstAngle > heading) {
                fL.setPower(-.17 + (leftPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));
                bL.setPower(-.17 + (leftPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));
                fR.setPower(.17 + (rightPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));
                bR.setPower(.17 + (rightPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));

                opmode.telemetry.addData("degrees", getAngle());
                opmode.telemetry.addData("lastAngle", lastAngles);
                opmode.telemetry.addData("globalangle", globalAngle);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        } else    // right turn.
            while (opmode.opModeIsActive() && angles.firstAngle < heading) {
                fL.setPower(.17 + (leftPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));
                bL.setPower(.17 + (leftPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));
                fR.setPower(-.17 + (rightPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));
                bR.setPower(-.17 + (rightPower * ((degrees - Math.abs(Math.abs(angles.firstAngle) - Math.abs(heading))) / degrees)));

                opmode.telemetry.addData("degrees", getAngle());
                //telemetry.addData("lastangle", lastAngles);
                //telemetry.addData("globalangle", globalAngle);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());

                opmode.telemetry.update();


            }

        //turn the motors off.
        stopMotors();
        //lastDegrees = degrees;

        // wait for rotation to stop.
        opmode.telemetry.addLine("done");
        opmode.telemetry.update();
        opmode.sleep(500);


        // reset angle tracking on new heading.
        //resetAngle();
    }

    public void turnPID(double angle, double p, double i, double d, double timeout){
        runtime.reset();
        resetAngle();
        double kP = Math.abs(p);
        double kD = d;
        double kI = i;
        double integral = 0;
        double currentTime = runtime.milliseconds();
        double pastTime = 0;
        startPos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //opmode.telemetry.addData("start offset: ", startPos.firstAngle);
        //opmode.telemetry.update();
        //opmode.sleep(2000);
        double target = (angle + startPos.firstAngle);
        double prevError = target - getAngle();
        double error = prevError;
        double power = 0;
        while ((Math.abs(error) > .5 && runtime.seconds() < timeout && opmode.opModeIsActive())) {
            pastTime = currentTime;
            currentTime = runtime.milliseconds();
            double dT = currentTime - pastTime;
            prevError = error;
            error = target - getAngle();
            integral += dT * (prevError);

            if (Math.abs(integral * kI) > .15 && Math.abs(error * kP) < .1) {
                kI = Math.abs(.15/integral);
            }
            else {
                kI = 0;
            }

            power = (error * kP) + integral * kI + ((error - prevError) / dT * kD);
            if (power < 0) {
                fL.setPower(power - .1);
                bL.setPower(power - .1);
                fR.setPower(-1 * (power) + .1);
                bR.setPower(-1 * (power) + .1);

            } else {
                fL.setPower(power + .1);
                bL.setPower(power + .1);
                fR.setPower(-1 * (power) - .1);
                bR.setPower(-1 * (power) - .1);
            }


            opmode.telemetry.addData("angle: ", getAngle());
            opmode.telemetry.addData("P", (error * kP));
            opmode.telemetry.addData("I", (integral * kI));
            opmode.telemetry.addData("integral", integral);
            opmode.telemetry.addData("D", ((Math.abs(error) - Math.abs(prevError)) / dT * kD));
            opmode.telemetry.update();
            prevError = error;
        }
        stopMotors();
    }

    public void gyroCorrect() {
        if (getAngle() > referenceAngle + 1) {
            rightCorrect = .8;
        }
        else if (getAngle() < referenceAngle - 1) {
            leftCorrect = .8;
        }
        else {
            leftCorrect = 1;
            rightCorrect = 1;
        }
    }

    public void setReferenceAngle() {
        resetAngle();
        referenceAngle = getAngle();
    }

    public double correctAngle(double angle) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //opmode.telemetry.addData("angle", angles.firstAngle);
        //opmode.telemetry.update();
        //opmode.sleep(5000);

        double deltaAngle = angle + angles.firstAngle ;

        return deltaAngle;


    }

    public void strafeRightGyro (double distance, double power) {
        //bR.setDirection(DcMotorSimple.Direction.REVERSE);
        //fR.setDirection(DcMotorSimple.Direction.REVERSE);

        reset();
        resetAngle();
        double target = Math.abs(distance * (537.6/11));

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {

            if (getAngle() > 1) {
                fL.setPower(power * .9);
                fR.setPower(-power * .9);
                bL.setPower(-power * 1.1 * 0.91);
                bR.setPower(power * 1.1 * 0.91);
            }
            else if (getAngle() < -1) {
                fL.setPower(power * 1.1);
                fR.setPower(-power * 1.1);
                bL.setPower(-power * .9 * 0.91);
                bR.setPower(power * .9 * 0.91);
            }
            else {
                fL.setPower(power);
                fR.setPower(-power);
                bL.setPower(-power* 0.91);
                bR.setPower(power* 0.91);
            }
            //opmode.telemetry.addData("avg", encoderAvg());
            //opmode.telemetry.addData("fl", fL.getCurrentPosition());
            //opmode.telemetry.addData("fr", fR.getCurrentPosition());
            //opmode.telemetry.addData("bl", bL.getCurrentPosition());
            //opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
        }
        stopMotors();

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void strafeLeftGyro (double distance, double power, int time) {
        runtime.reset();
        reset();
        resetAngle();
        double target = Math.abs(distance * (537.6/11));

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < time) {

            if (getAngle() > 1) {
                fL.setPower(-power * 1.1);
                fR.setPower(power * 1.1);
                bL.setPower(power * .9 * 0.91);
                bR.setPower(-power * .9 * 0.91);
            }
            else if (getAngle() < -1) {
                fL.setPower(-power * .9);
                fR.setPower(power * .9 );
                bL.setPower(power * 1.1 * 0.91);
                bR.setPower(-power * 1.1 * 0.91);
            }
            else {
                fL.setPower(-power);
                fR.setPower(power);
                bL.setPower(power * 0.91);
                bR.setPower(-power * 0.91);
            }
            //opmode.telemetry.addData("avg", encoderAvg());
            //opmode.telemetry.addData("fl", fL.getCurrentPosition());
            //opmode.telemetry.addData("fr", fR.getCurrentPosition());
            //opmode.telemetry.addData("bl", bL.getCurrentPosition());
            //opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
        }
        stopMotors();

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initColor() {

        sensorColorBMid = hwMap.get(ColorSensor.class, "sensorColorBMid");
        sensorColorBEdge = hwMap.get(ColorSensor.class, "sensorColorBEdge");
        sensorColorRMid = hwMap.get(ColorSensor.class, "sensorColorRMid");
        sensorColorREdge = hwMap.get(ColorSensor.class, "sensorColorREdge");
        sensorColorBotBack = hwMap.get(ColorSensor.class, "sensorColorBotBack");
       // sensorColorRight = hwMap.get(ColorSensor.class, "sensorColorRight");

        // get a reference to the distance sensor that shares the same name.
        sensorDistanceBEdge = hwMap.get(DistanceSensor.class, "sensorColorBEdge");
        //sensorDistanceRight = hwMap.get(DistanceSensor.class, "sensorColorRight");
        sensorDistanceBMid = hwMap.get(DistanceSensor.class, "sensorColorBMid");
        sensorDistanceREdge = hwMap.get(DistanceSensor.class, "sensorColorREdge");
        sensorDistanceRMid = hwMap.get(DistanceSensor.class, "sensorColorRMid");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        sensorColorBEdge.enableLed(false);
       // sensorColorRight.enableLed(false);
        sensorColorBMid.enableLed(false);

        sensorColorREdge.enableLed(false);
        // sensorColorRight.enableLed(false);
        sensorColorRMid.enableLed(false);

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
        /*
        while (opmode.opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColorBEdge.red() * SCALE_FACTOR),
                    (int) (sensorColorBEdge.green() * SCALE_FACTOR),
                    (int) (sensorColorBEdge.blue() * SCALE_FACTOR),
                    hsvValues);
            Color.RGBToHSV((int) (sensorColorBMid.red() * SCALE_FACTOR),
                    (int) (sensorColorBMid.green() * SCALE_FACTOR),
                    (int) (sensorColorBMid.blue() * SCALE_FACTOR),
                    hsvValues);

            Color.RGBToHSV((int) (sensorColorREdge.red() * SCALE_FACTOR),
                    (int) (sensorColorREdge.green() * SCALE_FACTOR),
                    (int) (sensorColorREdge.blue() * SCALE_FACTOR),
                    hsvValues);
            Color.RGBToHSV((int) (sensorColorRMid.red() * SCALE_FACTOR),
                    (int) (sensorColorRMid.green() * SCALE_FACTOR),
                    (int) (sensorColorRMid.blue() * SCALE_FACTOR),
                    hsvValues);
        }
        */

    }

    public void moveLeft(double power) {
        if (getAngle() > 1) {
            fL.setPower(-power * 1.2);
            fR.setPower(power * 1.2);
            bL.setPower(-power * .8);
            bR.setPower(-power * .8);
        }
        else if (getAngle() < -1) {
            fL.setPower(-power * .8);
            fR.setPower(power * .8);
            bL.setPower(-power * 1.2);
            bR.setPower(-power * 1.2);
        }
        else {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(-power);
            bR.setPower(-power);
        }
    }

    public void moveRight(double power) {
        if (getAngle() > 1) {
            fL.setPower(power * .8);
            fR.setPower(-power * .8);
            bL.setPower(power * 1.2);
            bR.setPower(power * 1.2);
        }
        else if (getAngle() < -1) {
            fL.setPower(power * 1.2);
            fR.setPower(-power * 1.2);
            bL.setPower(power * .8);
            bR.setPower(power * .8);
        }
        else {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(power);
        }
    }



    public void approachStonesRed (double power) {
        resetAngle();
        brakeMode();
        double speed = 1;

        boolean cont = true;
        //gripRUp();
        //grabberRDown();

        while (cont == true && opmode.opModeIsActive()){
            //moveLeft(power * speed);
            if (getAngle() > 1) {
                fL.setPower((-power * 1.1) * speed);
                fR.setPower((power * 1.1) * speed);
                bL.setPower((power * .9) * speed * .91);
                bR.setPower((-power * .9) * speed* .91);
            }
            else if (getAngle() < -1) {
                fL.setPower((-power * .9) * speed);
                fR.setPower((power * .9) * speed );
                bL.setPower((power * 1.1) * speed* .91);
                bR.setPower((-power * 1.1) * speed* .91);
            }
            else {
                fL.setPower(-power * speed);
                fR.setPower(power * speed );
                bL.setPower(power * speed* .91);
                bR.setPower(-power * speed* .91);
            }
            if (!Double.isNaN(sensorDistanceREdge.getDistance(DistanceUnit.CM))) {
                reset();
                speed = .5;
            }
            if (sensorDistanceREdge.getDistance(DistanceUnit.CM) < 9.5 || sensorDistanceRMid.getDistance(DistanceUnit.CM) < 9.5) {
                cont = false;
            }

            opmode.telemetry.addData("front", sensorDistanceREdge.getDistance(DistanceUnit.CM));
            opmode.telemetry.addData("back", sensorDistanceRMid.getDistance(DistanceUnit.CM));
            opmode.telemetry.update();
        }

        //strafeLeftGyro(2, 0.2, 1);
        stopMotors();
        floatMode();
        //gripRDown();
    }
    public void approachStonesBlue (double power) {
        resetAngle();
        brakeMode();
        double speed = 1;

        boolean cont = true;
        //gripRUp();
        //grabberBDown();

        while (cont == true && opmode.opModeIsActive()){
            //moveLeft(power * speed);
            if (getAngle() > 1) {
                fL.setPower((-power * 1.1) * speed);
                fR.setPower((power * 1.1) * speed);
                bL.setPower((power * .9) * speed * .91);
                bR.setPower((-power * .9) * speed* .91);
            }
            else if (getAngle() < -1) {
                fL.setPower((-power * .9) * speed);
                fR.setPower((power * .9) * speed );
                bL.setPower((power * 1.1) * speed* .91);
                bR.setPower((-power * 1.1) * speed* .91);
            }
            else {
                fL.setPower(-power * speed);
                fR.setPower(power * speed );
                bL.setPower(power * speed* .91);
                bR.setPower(-power * speed* .91);
            }
            if (sensorDistanceBMid.getDistance(DistanceUnit.CM) < 11) {
                reset();
                speed = .5;
            }
            if (sensorDistanceBEdge.getDistance(DistanceUnit.CM) < 5.5 || sensorDistanceBMid.getDistance(DistanceUnit.CM) < 8) {
                cont = false;
            }

            opmode.telemetry.addData("front", sensorDistanceBEdge.getDistance(DistanceUnit.CM));
            opmode.telemetry.addData("back", sensorDistanceBMid.getDistance(DistanceUnit.CM));
            opmode.telemetry.update();
        }

        stopMotors();
        floatMode();
    }



    public void alignStonesB (double power) {

        double conditionBEdge = (sensorColorBEdge.red() * sensorColorBEdge.green()) / (sensorColorBEdge.blue() * sensorColorBEdge.blue());
        double conditionBMid = (sensorColorBMid.red() * sensorColorBMid.green()) / (sensorColorBMid.blue() * sensorColorBMid.blue());
        opmode.telemetry.addData("edge",conditionBEdge);
        opmode.telemetry.addData("mid",conditionBMid);
        opmode.telemetry.update();
        while ((conditionBMid > 2 || conditionBEdge > 2) && opmode.opModeIsActive()) {
            conditionBMid = (sensorColorBMid.red() * sensorColorBMid.green()) / (sensorColorBMid.blue() * sensorColorBMid.blue());
            conditionBEdge = (sensorColorBEdge.red() * sensorColorBEdge.green()) / (sensorColorBEdge.blue() * sensorColorBEdge.blue());
            opmode.telemetry.addData("edge",conditionBEdge);
            opmode.telemetry.addData("mid",conditionBMid);
            opmode.telemetry.update();
            if (conditionBEdge > 2 && conditionBMid < 2) {
                moveBackward(power);
                if (conditionBEdge <= 2) {
                    grabberB.setPosition(0.15);
                }
            }

            else if (conditionBEdge < 2 && conditionBMid > 2) {
                moveForward(power);
            }

            else {
                moveBackward(power);
            }

            opmode.telemetry.addData("edge", conditionBEdge);
            opmode.telemetry.addData("mid", conditionBMid);
            opmode.telemetry.update();
        }

        stopMotors();
    }

    public void diagonalFR(double distance, double power)
    {
        reset();
        double target = Math.abs(distance * (537.6/11));

        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()){
            fR.setPower(power);
            bL.setPower(power);
        }
        stopMotors();
    }

    public void alignStonesR (double power) {

        double conditionREdge = (sensorColorREdge.red() * sensorColorREdge.green()) / (sensorColorREdge.blue() * sensorColorREdge.blue());
        double conditionRMid = (sensorColorRMid.red() * sensorColorRMid.green()) / (sensorColorRMid.blue() * sensorColorRMid.blue());
        runtime.reset();
        brakeMode();

        while ((conditionRMid > 3 || conditionREdge > 3) && opmode.opModeIsActive()) {
            conditionRMid = (sensorColorRMid.red() * sensorColorRMid.green()) / (sensorColorRMid.blue() * sensorColorRMid.blue());
            conditionREdge = (sensorColorREdge.red() * sensorColorREdge.green()) / (sensorColorREdge.blue() * sensorColorREdge.blue());
            if (conditionREdge > 3 && conditionRMid < 3) {
                moveForward(power);
                if (conditionREdge <= 3) {
                    grabberR.setPosition(0.65);
                }


            }

            else if (conditionREdge < 3 && conditionRMid > 3) {
                moveBackward(power);
            }

            else {
                moveForward(power);

            }

            opmode.telemetry.addData("edge", conditionREdge);
            opmode.telemetry.addData("mid", conditionRMid);
            opmode.telemetry.update();
        }


        stopMotors();
    }

    public void crossLineRed () {

       while (sensorColorBMid.red() < 200) {
           moveForward(0.3);
       }

       stopMotors();
    }

  /*  public void crossLineBlue () {

        while (sensorColorBotFront.red() < 200) {
            moveBackward(0.3);
        }

        stopMotors();
    } */

    public void moveBackward (double power) {

        //reset();
        resetAngle();
        opmode.sleep(100);

        //brakeMode();
        runtime.reset();

        if (getAngle() > 1) {
            fL.setPower(1.2 * -power);
            fR.setPower(0.8 * power);
            bL.setPower(1.2 * -power);
            bR.setPower(0.8 * power);
        }
        else if (getAngle() < -1) {
            fL.setPower(.8 * -power);
            fR.setPower(1.2 * power);
            bL.setPower(.8 * -power);
            bR.setPower(1.2 * power);
        }
        else {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(-power);
            bR.setPower(power);
        }

        //stopMotors();

        //floatMode();
        //resetAngle();

    }

    public void moveForward(double power) {

        //reset();
        resetAngle();
        opmode.sleep(100);



        //brakeMode();
        //runtime.reset();

        if (getAngle() > 1) {
            fL.setPower(.8 * power);
            fR.setPower(1.2 * -power);
            bL.setPower(.8 * power);
            bR.setPower(1.2 * -power);
        } else if (getAngle() < -1) {
            fL.setPower(1.2 * power);
            fR.setPower(.8 * -power);
            bL.setPower(1.2 * power);
            bR.setPower(.8 * -power);
        } else {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(-power);
        }

        //stopMotors();

       // floatMode();
        //resetAngle();
    }

    public void rotateTo(double angle){
        liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(liftRotate.getCurrentPosition() < (angle * degreesToTicks)){
            liftRotate.setPower(.3);
        }

        liftRotate.setPower(0);
    }

    public void extendTo(double length){
        liftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(liftExtend.getCurrentPosition() < length){
            liftExtend.setPower(.3);
        }

        liftExtend.setPower(0);
    }

}
