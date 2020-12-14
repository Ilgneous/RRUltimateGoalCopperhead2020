package org.firstinspires.ftc.teamcode.ultimategoal2020;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


public class hardwareMap {

    // Motors
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;

    // Servos
    //public Servo clip = null;
    public Servo wobble = null;
    public Servo elevator = null;

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
        intake = opmode.hardwareMap.get(DcMotor.class, "intake");
        shooter = opmode.hardwareMap.get(DcMotor.class, "shooter");
        wobble = opmode.hardwareMap.get(Servo.class, "wobble");
        elevator = opmode.hardwareMap.get(Servo.class, "elevator");

        degreesToTicks = 560.0 / 360.0;

        //set direction of motors
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);

        //intake.setDirection(DcMotor.Direction.FORWARD);
        //shooter.setDirection(DcMotor.Direction.FORWARD);

        //initColor();
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

        //Define and initialize servos
        //clip = hwMap.get(Servo.class, "clip");
        wobble = opmode.hardwareMap.get(Servo.class, "wobble");
        elevator = opmode.hardwareMap.get(Servo.class, "elevator");

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


        opmode.telemetry.addData("fl", fL.getCurrentPosition());
        opmode.telemetry.addData("fr", fR.getCurrentPosition());
        opmode.telemetry.addData("bl", bL.getCurrentPosition());
        opmode.telemetry.addData("br", bR.getCurrentPosition());
        opmode.telemetry.update();

    }

    public double atTarget(double distance) {
        return Math.abs(distance * COUNTS_PER_INCH);
    }

    public void goStraight (double distance, double power) {
        dtEncoderModeOn();
        int startVal = fL.getCurrentPosition();
        while (Math.abs(fL.getCurrentPosition() - startVal) < atTarget(distance) && opmode.opModeIsActive()) {
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }

        stopMotors();
        floatMode();
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
            fL.setPower(-power * 0.9);
            fR.setPower(power);
            bL.setPower(power * 0.9);
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
    public void turnPID(double angle, double pwr, double i, double d) {
        resetAngle();

        double deltaAngle = Math.abs(angle - getAngle());
        double pastdeltaAngle = deltaAngle;
        double currentTime = runtime.milliseconds();
        double kP = pwr / angle;
        double kI = i;
        double kD = d / angle;
        double prevTime = 0;
        double apply = 0;
        double deltaTime;

        while (Math.abs(deltaAngle) > 1){
            deltaAngle = Math.abs(angle - getAngle());
            kP = deltaAngle * kP;
            prevTime = currentTime;
            pastdeltaAngle = deltaAngle;
            currentTime = runtime.milliseconds();
            deltaTime =  currentTime - prevTime;
            kI = deltaAngle * deltaTime * kI;
            kD = (deltaAngle - pastdeltaAngle) / deltaTime * kD;
            apply = kP + kI + kD;

            fL.setPower(-apply);
            fR.setPower(apply);
            bL.setPower(-apply);
            bR.setPower(apply);


        }
    }

    public void straightPID(double dist, double pwr, double d) {
        resetAngle();
        reset();

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
            currentTime = runtime.milliseconds();
            deltaTime =  currentTime - prevTime;
            kI = deltaDist * deltaTime * kI;
            kD = (deltaDist - pastdeltaDist) / deltaTime * kD;
            apply = kP + kI + kD;

            if (getAngle() < 1) {
                gyroFix = 0;
                fL.setPower(apply);
                fR.setPower(apply);
                bL.setPower(apply);
                bR.setPower(apply);
            }

            else if (getAngle() > 1 && getAngle() < 60) {
                gyroFix += .001;
                fL.setPower(apply);
                fR.setPower(apply + gyroFix);
                bL.setPower(apply);
                bR.setPower(apply + gyroFix);
            }

            else if (getAngle() < 359 && getAngle() > 300) {
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



}
