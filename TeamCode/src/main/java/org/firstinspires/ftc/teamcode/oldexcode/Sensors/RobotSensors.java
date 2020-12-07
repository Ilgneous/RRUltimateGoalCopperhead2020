package org.firstinspires.ftc.teamcode.oldexcode.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotSensors {

    public BNO055IMU imu;
    LinearOpMode opMode;
    Orientation angles;
    BNO055IMU.Parameters parameters;

    public RobotSensors(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");;
        imu.initialize(parameters);
    }

    public void updateOrientation() {
        angles = imu.getAngularOrientation();
    }

}
