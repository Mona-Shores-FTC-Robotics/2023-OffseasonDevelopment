package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

public class DriveTrain {
    // DriveTrain tuning constants
    final double P = 1; // default = 10
    final double D = 0; // default = 0
    final double I = 0; // default = 3
    final double F = 0; // default = 0
    private final double STRAFE_FACTOR = 1.2;
    private final double STARTING_RAMP_VALUE = 1.0;
    private final double RAMP_INCREMENT = 0.0;
    private final double MAX_TIP = 10.0;
    private final double STICK_DEAD_ZONE = .1;
    private final double AUTO_TURN_MAX_SPEED = 1.0;
    private final double ALLOWED_TURN_ERROR = .5;
    private final double TURN_TO_ANGLE_FACTOR = 1.0 / 180.0;

    // DriveTrain physical constants
    private final double MAX_MOTOR_SPEED_RPS = 312.0 / 60.0;
    private final double TICKS_PER_REV = 537.7;
    private final double DRIVE_GEAR_REDUCTION = 1.0;
    private final double WHEEL_DIAMETER_INCHES = 3.93701;
    private final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

    /* Public OpMode objects and variables. */
    public DcMotorEx driveMotor[] = new DcMotorEx[4]; // {leftfront, rightfront, leftback, rightback}
    public final String driveMotorNames [] = {"LFDrive", "RFDrive", "LBDrive", "RBDrive"};
    public double driveMotorPower[] = {0.0,0.0,0.0,0.0};
    public double driveMotorTargetSpeed[] = {0.0,0.0,0.0,0.0};
    public int driveMotorTargetPosition[] = {0,0,0,0};

    public double drive = 0.0;
    public double strafe = 0.0;
    public double turn = 0.0;

    /* Private OpMode objects and variables */
    private double ramp = STARTING_RAMP_VALUE;
    private String runningToPosition = "Not Running";
    private boolean tipRecoveryFlag = false;
    private boolean turning = false;
    private double targetAngle = 0.0;
    private double angleDelta = 0.0;

    private final ElapsedTime runToPositionPeriod = new ElapsedTime();
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
        DcMotorSimple.Direction fwd = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rvrs = DcMotorSimple.Direction.REVERSE;
        DcMotorSimple.Direction driveMotorDirections[] = {rvrs, fwd, rvrs, fwd};

        for (int i = 0; i < 4; i++ ){
            driveMotor[i] = ahwMap.get(DcMotorEx.class, driveMotorNames[i]);
            driveMotor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setDirection(driveMotorDirections[i]);
         //   if ((double) (i) % 2 == 1 ) driveMotor[i].setDirection(DcMotorSimple.Direction.FORWARD);
           // else if ((double) (i) % 2 == 0) driveMotor[i].setDirection(DcMotorSimple.Direction.REVERSE);
            // add if need to set PID: driveMotor[i].setVelocityPIDFCoefficients(P,I,D,F);
        }

    }

    /** Code to determine drive method*/
    public void driveModeSelection(Gamepad driverGamepad, LinkedList<Double> angle, LinkedList<Double> tiltAngle,
                                   LinkedList<Double> tiltSpeed, LinkedList<Double> tiltAccel){
        double driveInput = -driverGamepad.left_stick_y;
        double strafeInput = driverGamepad.left_stick_x;
        double turnInput = driverGamepad.right_stick_x;
        boolean turnToStraight = driverGamepad.y;
        boolean turnToRight = driverGamepad.b;
        boolean turnToLeft = driverGamepad.x;
        boolean turntoBack = driverGamepad.a;
        String turnAngles = new String();

        activeOpMode.telemetry.addData("tip angle", Math.round(tiltAngle.get(0)));
        activeOpMode.telemetry.addData("tip recovery", tipRecoveryFlag);

        //turning = turnToAngleCalc(turnToRight,turnToStraight,turnToLeft,turntoBack,turnInput,angle);
        // fieldOrientedControl(driveInput, strafeInput,auto drive inputs, auto strafe inputs, angle.get(0));
        if (Math.abs(tiltAngle.get(0)) > MAX_TIP || tipRecoveryFlag){
            tipRecoveryFlag = tipRecovery(tiltAngle.get(0), tiltSpeed.get(0), tiltAccel.get(0));
        }
        else if (Math.abs(driveInput) > STICK_DEAD_ZONE || Math.abs(strafeInput) > STICK_DEAD_ZONE // delete condition, always calculate driver inputs
                || Math.abs(turnInput) > STICK_DEAD_ZONE){
            if (driverGamepad.right_bumper) {
                fieldOrientedControl(driveInput, strafeInput, turnInput, angle.get(0));
            }
            else {
                // future improvement: change code to allow driving while performing an automatic turn
                drive = -driverGamepad.left_stick_y;
                strafe = driverGamepad.left_stick_x;
                turn = driverGamepad.right_stick_x;
            }
            mecanumDriveSpeedControl();
        }
        else if (turntoBack || turnToLeft || turnToRight || turnToStraight || turning){ // delete condition, always calculate
            turning = turnToAngleCalc(turnToRight,turnToStraight,turnToLeft,turntoBack,angle);
            if (turning) {mecanumDriveSpeedControl();}
        }
        // else if (some auto drive command && not left/right stick command) {auto drive method}
        else {
            drive = 0; // delete line
            strafe = 0; // delete line
            turn = 0; // delete line
            mecanumDriveSpeedControl();
        }

        if (angle.size() > 4) {
            for (int i = 0; i < 5; i++) {
                turnAngles = turnAngles + " " + Math.round(angle.get(i));
            }
        }
        activeOpMode.telemetry.update();
    }

    /** fieldOrientedControl takes inputs from the gamepad and the angle of the robot.
     * These inputs are used to calculate the drive, strafe and turn inputs needed for the MecanumDrive method.
     */
    public void fieldOrientedControl (double forward, double sideways, double turning, double robotAngle){  // remove turning input
        // Consider moving these constants to top of class
        robotAngle = robotAngle * Math.PI / 180;
        double DRIVE_SPEED_FACTOR = 1.0;
        double STRAFE_SPEED_FACTOR = 1.0;
        double TURN_SPEED_FACTOR = 1.0; // delete line

        // if (Math.abs(forward) < STICK_DEAD_ZONE) {forward = 0;}
        // if (Math.abs(sideways) < STICK_DEAD_ZONE) {sideways = 0;}

        double magnitude = Math.sqrt(Math.pow(forward, 2) + Math.pow(sideways, 2));
        double driveAngle = Math.copySign(Math.acos(forward/magnitude), Math.asin(-sideways));
        double deltaAngle = robotAngle-driveAngle;


        drive = DRIVE_SPEED_FACTOR * magnitude * Math.cos(deltaAngle);
        strafe = STRAFE_SPEED_FACTOR * magnitude * Math.sin(deltaAngle);
        turn = TURN_SPEED_FACTOR * turning; // Delete line

        /*
        driveSquared = Math.pow(forward * Math.cos(robotAngle),2) + Math.pow(sideways * Math.sin(robotAngle), 2);
        strafeSquared = Math.pow(forward * Math.sin(robotAngle),2) - Math.pow(sideways * Math.cos(robotAngle), 2);
        int sign = Long.signum(strafeSquared);

        drive = DRIVE_SPEED_FACTOR * Math.copySign(Math.pow(Math.abs(driveSquared), 0.5), driveSquared);
        strafe = STRAFE_SPEED_FACTOR * Math.copySign(Math.pow(Math.abs(strafeSquared), 0.5), strafeSquared);
        turn = TURN_SPEED_FACTOR * turning;

         */

    }

    public void mecanumDriveSpeedControl() {
        String caption = new String();
        String angleValues = new String();

        // Put Mecanum Drive math and motor commands here.
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        driveMotorTargetSpeed[0] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[1] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        driveMotorTargetSpeed[2] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        driveMotorTargetSpeed[3] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        activeOpMode.telemetry.addData("drive input", drive);
        activeOpMode.telemetry.addData("strafe input", strafe);
        activeOpMode.telemetry.addData("turn input", turn);

        for (int i = 0; i < 4; i++ ){
            driveMotor[i].setVelocity(driveMotorTargetSpeed[i]);
            // this may not be needed: driveMotor[i].setPower(driveMotorPower[i]);
            caption = "Motor " + i + " Target Speed";
            activeOpMode.telemetry.addData(caption, Math.round(100 * driveMotorTargetSpeed[i] / TICKS_PER_REV));
            activeOpMode.telemetry.addData("Actual Motor Speed", Math.round(100 * driveMotor[i].getVelocity() / TICKS_PER_REV));
        }
    }

    public void mecanumDrivePowerControl (){
        String caption = new String();
        String angleValues = new String();

        // Put Mecanum Drive math and motor commands here.
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));
        driveMotorPower[0] = ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        driveMotorPower[1] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        driveMotorPower[2] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        driveMotorPower[3] = MAX_SPEED_TICK_PER_SEC * ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        activeOpMode.telemetry.addData("drive input", drive);
        activeOpMode.telemetry.addData("strafe input", strafe);
        activeOpMode.telemetry.addData("turn input", turn);

        for (int i = 0; i < 4; i++ ) {
            driveMotor[i].setPower(driveMotorPower[i]);
            caption = "Motor " + i + " Target Speed";
            activeOpMode.telemetry.addData(caption, Math.round(100 * driveMotorTargetSpeed[i] / TICKS_PER_REV));
            activeOpMode.telemetry.addData("Actual Motor Speed", Math.round(100 * driveMotor[i].getVelocity() / TICKS_PER_REV));
        }
    }

    public void mecanumDrivePositionControl(double targetPower, String driveMode, double driveDistance) {
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

    // turns to a specified angle based on driver inputs.
    public  boolean turnToAngleCalc(boolean right, boolean fwd, boolean left, boolean back, LinkedList<Double> robotAngle){
        if (fwd) {targetAngle = 0;}
        else if (left) {targetAngle = 90;}
        else if (back) {targetAngle = 180;}
        else if (right) {targetAngle = -90;}

        angleDelta = targetAngle - robotAngle.get(0);
        if (angleDelta > 180) {angleDelta -= 360;}
        else if (angleDelta < -180) {angleDelta += 360;}

        /*
        if (Math.abs(turnInput) > STICK_DEAD_ZONE) {
            targetAngle = robotAngle.get(0);
            turn = turnInput;

            return false;
        }

         */
        if (Math.abs(angleDelta) > ALLOWED_TURN_ERROR){ // convert to else if
            turn = - angleDelta * TURN_TO_ANGLE_FACTOR * AUTO_TURN_MAX_SPEED;
            drive = 0;
            strafe = 0;

            return true;
        }
        else {return false;}
    }

    /**
     * Tip Recovery Code
     * Case: Angle > Max Tip
     * Power Front Wheels forwards, power is function of angular velocity and/or acceleration
     * Set Driver Control to rumble
     * Set Tip Recovery to Recovering // This will take control from the driver
     * Case: Angle < Min Recovery && Angle > - Min Recovery && Tip Recovery == Recovering
     * Set Tip Recovery to Normal // This will return control to the driver
     * Stop driver control rumble
     * Case: Angle < - Max Tip
     * Power Rear Wheels backwards, power is function of angular velocity and/or acceleration
     * Set Driver Control to rumble
     * Set Tip Recovery to Recovering // This will take control from the driver
     * Case: Angle < Max Tip && Angle > Min Tip && Tip Recovery == Recovery
     * Run some reduced recovery function.
     * Case: Angle > - Max Tip && Angle < - Min Tip && Tip Recovery == Recovery
     * Run some reduced recovery function.
     * End Case
     */
    public boolean tipRecovery(double tipAngle, double tipVelocity, double tipAcceleration) {

        //Tip Recovery Variables
        // tip angle where tip recovery code takes over
        double maxTip = 10;
        // multiplier for acceleration or velocity to get recovery power
        double recoveryPower = 50;
        // tip angle where tip recovery code is stopped and driver control resumes
        double tipRecovered = 5;
        // multiplier for angle to get final recovery power
        double finalRecoveryPower = 25;
        int sign[] = {1, 1, 1, 1};

        if (Math.abs(tipAngle) > maxTip && tipAngle * tipVelocity > 0){

            for (int i = 0; i<4; i++) {
                driveMotor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                driveMotor[i].setPower(recoveryPower * -tipAngle/10.0 * sign[i]);
            }
            activeOpMode.telemetry.addData("front motor power", driveMotor[0].getPower());
            activeOpMode.telemetry.addData("back motor power", driveMotor[2].getPower());

            //write code to make driver control rumble, this could be in a different section.
            return true;
        }
        else if (Math.abs(tipAngle) < tipRecovered && tipRecoveryFlag){

            // Sets the power to 0 in case the driver is not touching any controls.
            for (int i = 0; i<4; i++) {
                driveMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                driveMotor[i].setPower(0);
            }

            //write code to make driver control rumble stop, this could be in a different section.
            return false;
        }
        else if (Math.abs(tipAngle) > tipRecovered && tipRecoveryFlag && tipVelocity < 0){
            for (int i = 0; i<4; i++) {
                driveMotor[i].setPower(finalRecoveryPower * -tipAngle/10);
            }
            return true;
        }
        return true;
    }

}
