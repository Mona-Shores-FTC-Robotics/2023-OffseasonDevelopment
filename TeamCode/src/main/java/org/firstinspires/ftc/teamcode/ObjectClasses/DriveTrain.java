package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

public class DriveTrain {
    // DriveTrain tuning constants
    final double P = 1;
    final double D = 0;
    final double I = 0;
    final double F = 0;
    private final double STRAFE_FACTOR = 1.2;
    private final double STARTING_RAMP_VALUE = 1;
    private final double RAMP_INCREMENT = 0;
    private final double MAX_TIP = 10;
    private final double STICK_DEAD_ZONE = .1;

    // DriveTrain physical constants
    private final double TICKS_PER_REV = 537.7;
    private final double DRIVE_GEAR_REDUCTION = 1;
    private final double WHEEL_DIAMETER_INCHES = 3.93701;
    private final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // DriveTrain tunabl



    /* Public OpMode objects and variables. */
    public DcMotorEx driveMotor[] = new DcMotorEx[4]; // {leftfront, rightfront, leftback, rightback}
    public final String driveMotorNames [] = {"LFDrive", "RFDrive", "LBDrive", "RBDrive"};
    public double driveMotorPower[] = {0,0,0,0};
    public double driveMotorTargetSpeed[] = {0,0,0,0};
    public int driveMotorTargetPosition[] = {0,0,0,0};

    public double drive = 0;
    public double strafe = 0;
    public double turn = 0;

    /* Private OpMode objects and variables */
    private double ramp = STARTING_RAMP_VALUE;
    private String runningToPosition = "Not Running";
    private boolean tipRecovery = false;

    public final ElapsedTime runToPositionPeriod = new ElapsedTime();
    private LinearOpMode activeOpMode;
    private HardwareMap hwMap = null;

    /* Constructor */
    public DriveTrain(LinearOpMode opMode) {
        activeOpMode = opMode;
    }

    /* METHOD: Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        activeOpMode = opMode;

        // Define and Initialize Motors

        for (int i = 0; i < 4; i++ ){
            driveMotor[i] = ahwMap.get(DcMotorEx.class, driveMotorNames[i]);
            driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if ((double) (i) % 2 == 1 ) driveMotor[i].setDirection(DcMotorSimple.Direction.FORWARD);
            else if ((double) (i) % 2 == 0) driveMotor[i].setDirection(DcMotorSimple.Direction.REVERSE);
            // add if need to set PID: driveMotor[i].setVelocityPIDFCoefficients(P,I,D,F);
        }

    }

    /** Code to determine drive method*/
    public void DriveModeSelection(Gamepad driverGamepad, LinkedList<Double> angle, LinkedList<Double> tipAngle,
                                   LinkedList<Double> tipSpeed, LinkedList<Double> tipAccel){
        double driveInput = -driverGamepad.left_stick_y;
        double strafeInput = driverGamepad.left_stick_x;
        double turnInput = driverGamepad.right_stick_x;
        boolean turnToStraight = driverGamepad.y;
        boolean turnToRight = driverGamepad.b;
        boolean turnToLeft = driverGamepad.x;
        boolean turntoBack = driverGamepad.a;

        if (tipAngle.get(0) > MAX_TIP || tipRecovery){
            tipRecovery(tipAngle, tipSpeed, tipAccel);
        }
        else if (Math.abs(driveInput) > STICK_DEAD_ZONE || Math.abs(strafeInput) > STICK_DEAD_ZONE
                || Math.abs(turnInput) > STICK_DEAD_ZONE){
            fieldOrientedControl(driveInput,strafeInput,turnInput,angle.get(0));
            MecanumDriveSpeedControl();
        }
        else if (turntoBack || turnToLeft || turnToRight || turnToStraight){
            // uncomment when method exists:  TurnToAngle(turn to angle inputs)
        }
        /* else if (some other criteria){
           Some other method
           }
         */

    }

    /** fieldOrientedControl takes inputs from the gamepad and the angle of the robot.
     * These inputs are used to calculate the drive, strafe and turn inputs needed for the MecanumDrive method.
     */
    public void fieldOrientedControl (double forward, double sideways, double turning, double robotAngle){
        // Consider moving these constants to top of class
        double DRIVE_SPEED_FACTOR = .8;
        double STRAFE_SPEED_FACTOR = 1.0;
        double TURN_SPEED_FACTOR = .8;

        drive = DRIVE_SPEED_FACTOR * (forward * Math.cos(robotAngle) + sideways * Math.sin(robotAngle));
        strafe = STRAFE_SPEED_FACTOR * (forward * Math.sin(robotAngle) + sideways * Math.cos(robotAngle));
        turn = TURN_SPEED_FACTOR * turning;

    }

    public void MecanumDriveSpeedControl() {

        // Put Mecanum Drive math and motor commands here.
        double dPercent = Math.abs(drive) / (Math.abs(drive) + Math.abs(strafe) + Math.abs(turn));
        double sPercent = Math.abs(strafe) / (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe));
        double tPercent = Math.abs(turn) / (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe));

        driveMotorTargetSpeed[0] = TICKS_PER_REV * ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[1] = TICKS_PER_REV * ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[2] = TICKS_PER_REV * ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));
        driveMotorTargetSpeed[3] = TICKS_PER_REV * ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));

        for (int i = 0; i < 4; i++ ){
            driveMotor[i].setVelocity(driveMotorTargetSpeed[i]);
            // this may not be needed: driveMotor[i].setPower(driveMotorPower[i]);
        }
        // The following code was used last year, but I don't know when Power would ever not be a number, so I am removing.
        //But, I am keeping the code just in case.
        /*  Begin commented code A
        if (!Double.isNaN(leftFrontPower) && !Double.isNaN(rightFrontPower) && !Double.isNaN(leftBackPower) && !Double.isNaN(rightBackPower)) {
            LFDrive.setPower(leftFrontPower);
            RFDrive.setPower(rightFrontPower);
            LBDrive.setPower(leftBackPower);
            RBDrive.setPower(rightBackPower);
        } else {
            LFDrive.setPower(0);
            RFDrive.setPower(0);
            LBDrive.setPower(0);
            RBDrive.setPower(0);
        }
        End commented code A
         */
    }
    /**
     Case opmode is active && not already driving
         Set all motors to stop and reset encoders
         Set target positon value for each motor
         Set each motor to its target position
            
     */
    public void MecanumDrivePositionControl(double targetPower, String driveMode, double driveDistance) {
        if (activeOpMode.opModeIsActive() && runningToPosition != driveMode) {

            // Calculate Motor Target Position based on drive mode.
            if (driveMode == "drive"){
                for (int i =0; i < 4; i++){
                    driveMotorTargetPosition[0] = (int) (driveDistance * COUNTS_PER_INCH);
                }
            }
            else if (driveMode == "strafe") {
                driveMotorTargetPosition[0] = (int) -(driveDistance * COUNTS_PER_INCH * STRAFE_FACTOR);
                driveMotorTargetPosition[1] = (int) (driveDistance * COUNTS_PER_INCH * STRAFE_FACTOR);
                driveMotorTargetPosition[2] = (int) (driveDistance * COUNTS_PER_INCH * STRAFE_FACTOR);
                driveMotorTargetPosition[3] = (int) -(driveDistance * COUNTS_PER_INCH * STRAFE_FACTOR);
            }

            runToPositionPeriod.reset();

            //reset starting ramp value
            ramp = STARTING_RAMP_VALUE * targetPower;

            for (int i = 0; i<4; i++) {
                driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                driveMotor[i].setTargetPosition(driveMotorTargetPosition[i]);
                driveMotor[i].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                driveMotor[i].setPower(abs(ramp));
            }
            //we are now driving in the defined driveMode
            runningToPosition = driveMode;
        }
        else if (driveMotor[0].isBusy() || driveMotor[1].isBusy() || driveMotor[2].isBusy() || driveMotor[3].isBusy()) {
            ramp = Math.min(ramp + RAMP_INCREMENT, targetPower);
            for (int i = 0; i < 4; i++) driveMotor[i].setPower(abs(ramp));
        }
        else {
                runningToPosition = "NotRunning";
                for (int i = 0; i < 4; i++) driveMotor[i].setPower(0);
        }

    }

    public boolean tipRecovery(LinkedList<Double> tipAngle, LinkedList<Double> tipVelocity, LinkedList<Double> tipAcceleration){
        return false;
    }

}
