package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {
    LinearOpMode activeOpMode = null;
    DriveTrain mec_drive = new DriveTrain(activeOpMode);

    /* Constructor */
    public Robot(LinearOpMode opMode) {
        activeOpMode = opMode;
    }


}
