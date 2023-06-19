package org.firstinspires.ftc.teamcode.ObjectClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.LinkedList;

public class Gyro {

   /* local OpMode members. */

    HardwareMap hwMap = null;
    // BNO055IMU imu;
    IMU imu;
    private ElapsedTime turnPeriod = new ElapsedTime();

    //gyro members
    private final RevHubOrientationOnRobot hubOrientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, // direction of control hub logo on robot
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT); // direction of USB ports on robot
 //   public Orientation lastAngles = new Orientation();
 //   public double currAngle = 0.0;

    private int deltaLength = 5; // Number of readings used for calculating velocity and acceleration
    public LinkedList<Orientation> angles = new LinkedList<>();
    public LinkedList<Double> turnAngle = new LinkedList<>();
    public LinkedList<Double> tiltAngle = new LinkedList<>();
    public LinkedList<Double> tiltVelocity = new LinkedList<>();
    public LinkedList<Double> tiltAccel = new LinkedList<>();
    public LinkedList<ElapsedTime> readTime = new LinkedList<>();

//    public Orientation originalOrientation;
    LinearOpMode activeOpMode;

    /* Constructor */
    public Gyro(LinearOpMode mode) {
        activeOpMode = mode;
    }

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*
        // code for old control hub IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
         */

        // code for new control hub imu
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)

    public void UpdateGyro(ElapsedTime runtime) {
        int calcLocation = Math.min(deltaLength, angles.size());

        readTime.add(0,runtime);
        angles.add(0, imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));  //getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES));
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        AngularVelocity veloc = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        turnAngle.add(0, (double) angles.get(0).firstAngle);
        tiltAngle.add(0, (double) angles.get(0).thirdAngle);  // This angle will vary based on the orientation of the control hub.

        if(readTime.get(0) == readTime.get(calcLocation)) {
            tiltVelocity.add(0, (tiltAngle.get(0) - tiltAngle.get(calcLocation)) / (readTime.get(0).seconds() - readTime.get(calcLocation).seconds()));
            tiltAccel.add(0, (tiltVelocity.get(0) - tiltAngle.get(calcLocation)) / (readTime.get(0).seconds() - readTime.get(calcLocation).seconds()));
        }

        else {
            tiltVelocity.add(0, 0.0);
            tiltAccel.add(0, 0.0);
        }
    }

    /*
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }
        currAngle += deltaAngle;
        lastAngles = orientation;
        return currAngle;
    }

    public double getAbsoluteAngle() {
        originalOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double absoluteAngle = originalOrientation.firstAngle;
        return absoluteAngle;
    }
     */

}
