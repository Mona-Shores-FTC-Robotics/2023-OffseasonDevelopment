package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;

import java.util.ArrayList;

@TeleOp(name = "Test Chassis", group =  "Chassis Bot")

public class TestChassis extends LinearOpMode {
    DriveTrain MecDrive = new DriveTrain(this);
    org.firstinspires.ftc.teamcode.ObjectClasses.Gyro Gyro = new Gyro(this);
    //    GamepadHandling GamePads = new GamepadHandling(this);
    private final ElapsedTime runtime = new ElapsedTime();


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        boolean driveMethodSpeedControl = false; // false = power control, true = speed control
        boolean manualControl = false;
        boolean autoControl = false;
        double testRunStart = -2;
        double maximumSpeed[] = {0, 0, 0, 0};
        double timeTo200RPM[] = {0, 0, 0, 0};
        double autoDriveInput = 1;
        double P = 10;
        double I = 3;
        double D = 0;
        double F = 12;
        double F_INCREMENT = 1;

        MecDrive.init(hardwareMap, this);
        Gyro.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
        }

        runtime.reset();
        while (opModeIsActive()) {
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                driveMethodSpeedControl = !driveMethodSpeedControl;
            }


            MecDrive.drive = - currentGamepad1.left_stick_y;
            MecDrive.strafe = currentGamepad1.left_stick_x;
            MecDrive.turn = currentGamepad1.right_stick_x;

            if (Math.abs(currentGamepad1.left_stick_x) + Math.abs(currentGamepad1.left_stick_y)+Math.abs(currentGamepad1.right_stick_x) > .1){
                manualControl = true;
                MecDrive.drive = - currentGamepad1.left_stick_y;
                MecDrive.strafe = currentGamepad1.left_stick_x;
                MecDrive.turn = currentGamepad1.right_stick_x;
            }
            else {
                manualControl = false;
            }

            if(driveMethodSpeedControl && manualControl){
                telemetry.addData("Drive Method", "Speed Control");
                MecDrive.mecanumDriveSpeedControl();
            }
            else if(manualControl) {
                MecDrive.mecanumDrivePowerControl();
            }
            else if (autoControl && runtime.seconds() - testRunStart < 1){

                for (int i = 0; i < 4; i++){
                    maximumSpeed[i] = Math.max(MecDrive.driveMotor[0].getVelocity() * MecDrive.MAX_SPEED_TICK_PER_SEC, maximumSpeed[i]);

                    if (maximumSpeed[i] < 200) {
                        timeTo200RPM[i] = runtime.seconds();
                    }
                }
            }
            else if (currentGamepad1.y){  // 1 second of speed control driving
                autoControl = true;
                testRunStart = runtime.seconds();
                MecDrive.drive = autoDriveInput;
                MecDrive.strafe = MecDrive.turn = 0;
                MecDrive.mecanumDriveSpeedControl();
                for (int i = 0; i < 4; i++){
                    maximumSpeed[i] = 0;
                    timeTo200RPM[i] = 0;
                }
            }
            else if (currentGamepad1.x){ // 1 second of speed control strafing
                autoControl = true;
                testRunStart = runtime.seconds();
                MecDrive.strafe = autoDriveInput;
                MecDrive.drive = MecDrive.turn = 0;
                MecDrive.mecanumDriveSpeedControl();
                for (int i = 0; i < 4; i++){
                    maximumSpeed[i] = 0;
                    timeTo200RPM[i] = 0;
                }
            }
            else if (currentGamepad1.a){  // 1 second of power control driving
                autoControl = true;
                testRunStart = runtime.seconds();
                MecDrive.drive = autoDriveInput;
                MecDrive.strafe = MecDrive.turn = 0;
                MecDrive.mecanumDrivePowerControl();
                for (int i = 0; i < 4; i++){
                    maximumSpeed[i] = 0;
                    timeTo200RPM[i] = 0;
                }
            }
            else if (currentGamepad1.b){ // 1 second of power control strafing
                autoControl = true;
                testRunStart = runtime.seconds();
                MecDrive.strafe = autoDriveInput;
                MecDrive.drive = MecDrive.turn = 0;
                MecDrive.mecanumDrivePowerControl();
                for (int i = 0; i < 4; i++){
                    maximumSpeed[i] = 0;
                    timeTo200RPM[i] = 0;
                }
            }
            else if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                autoDriveInput = Math.max(1, autoDriveInput + .1);
            }
            else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                autoDriveInput = Math.min(.1, autoDriveInput - .1);
            }
            else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                F = F + F_INCREMENT;
                for (int i = 0; i < 4; i++){
                    MecDrive.driveMotor[i].setVelocityPIDFCoefficients(P, I, D, F);
                }
            }
            else if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                F = Math.min(0, F - F_INCREMENT);
            }

            telemetry.addData("PIDF Coefficients", MecDrive.driveMotor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("input power", autoDriveInput);
            telemetry.addData("maximum speed",maximumSpeed);
            telemetry.addData("time to reach 200 rpm", timeTo200RPM);
            telemetry.update();
        }
    }
}
