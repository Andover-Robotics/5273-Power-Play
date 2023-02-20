package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.AutoPaths;

@Autonomous(name = "ParkAutonomous", group = "Competition")
public class ParkAutonomous extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        AutoPaths autoPaths=new AutoPaths(this.hardwareMap);

        waitForStart();


        autoPaths.drive.followTrajectory(autoPaths.stackLeft);

        if (isStopRequested()) {
            return;
        }

        while (!isStopRequested() && opModeIsActive()) {
            continue;
        }
    }

}
