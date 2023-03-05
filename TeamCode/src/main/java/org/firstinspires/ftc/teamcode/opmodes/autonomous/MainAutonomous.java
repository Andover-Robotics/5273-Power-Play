package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.apriltag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Parking Autonomous", group = "Competition")

public class MainAutonomous extends LinearOpMode {
    double fx = 578.272;
    double fy = 578.272;
    double cx = 640;
    double cy = 360;

    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot=Bot.getInstance(this);

        AutoPaths autoPaths = new AutoPaths(hardwareMap, telemetry);



        bot.manipulator.verticalLinearSlides.resetSlideEncoders();
        // init (hopefully)
        bot.manipulator.horizontalLinearSlides.retractSlides();
        bot.manipulator.horizontalArm.setTransfer();
        bot.manipulator.verticalArm.setInit();

        waitForStart();

        autoPaths.drive.followTrajectorySequence(autoPaths.stack);
        telemetry.update();

        if (isStopRequested()) {
            return;
        }


        while (!isStopRequested() && opModeIsActive()) { }
    }

}

