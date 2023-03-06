package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        bot=Bot.getInstance(this);

        AutoPaths autoPaths = new AutoPaths(hardwareMap, telemetry);
        colorSensor=hardwareMap.get(ColorSensor.class, "colorSensor");



        bot.manipulator.verticalLinearSlides.resetSlideEncoders();
        // init (hopefully)
        bot.manipulator.horizontalArm.closeClaw();
        bot.manipulator.horizontalArm.setIdle();
        bot.manipulator.verticalLinearSlides.hover();
        bot.manipulator.verticalArm.setTransfer();
        bot.manipulator.verticalArm.closeClaw();

        waitForStart();

        autoPaths.drive.followTrajectorySequence(autoPaths.detect);

        autoPaths.parkingPose=colorDetected();


        autoPaths.drive.followTrajectorySequence(autoPaths.park);



        telemetry.update();

        if (isStopRequested()) {
            return;
        }


        while (!isStopRequested() && opModeIsActive()) { }
    }
    private int colorDetected(){
        int red=colorSensor.red();
        int green=colorSensor.green();
        int blue = colorSensor.blue();
        telemetry.addData("color", colorSensor.red()+" "+colorSensor.green()+" "+colorSensor.blue());
        if(red > blue && red > green){
            return 1;
        }
        else if(green > blue && green > red){

            return 2;

        }
        else if(blue > red && blue > green){
            return 3;
        }
        return 2;

    }


}

