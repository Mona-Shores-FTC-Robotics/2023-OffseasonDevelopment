package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Basic AutoOpMode")
public class BasicAutoOpMode extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain(this);
    org.firstinspires.ftc.teamcode.ObjectClasses.Gyro Gyro = new Gyro(this);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private TrajectorySequence sampleTrajectory;
    private Pose2d currentPose;
    private Pose2d startPose;

    @Override
    public void runOpMode() {
        SampleMecanumDrive MecDrive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted()) {
            //Store the previous loop's gamepad values.
            previousGamepad1 = GamepadHandling.copy(currentGamepad1);
            previousGamepad2 = GamepadHandling.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = GamepadHandling.copy(gamepad1);
            currentGamepad2 = GamepadHandling.copy(gamepad2);

            telemetry.update();
            sleep(20);
        }

        startPose = new Pose2d(0, 0, 0);

        sampleTrajectory = MecDrive.trajectorySequenceBuilder(startPose)
                .forward(1.0)
                .waitSeconds(0.2)
                .back(1.0)
                .build();

        MecDrive.setPoseEstimate(startPose);
        MecDrive.followTrajectorySequence(sampleTrajectory);
    }
}


