package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    private final Bot bot = Bot.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        AutoPaths autoPaths = new AutoPaths(this.hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.032, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        telemetry.addData("Init done","Done!");
        telemetry.update();

        int PARK_NUMBER = 2;

        while (!isStarted()&&!isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            for (AprilTagDetection tag: currentDetections) {
                telemetry.addData("tag", tag.id);
                telemetry.update();

                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    PARK_NUMBER = tag.id;
                }
            }

            sleep(20);
        }

        waitForStart();

        switch (PARK_NUMBER){
            case 1:
                pipelineResult = PipelineResult.ONE;
                break;
            case 2:
                pipelineResult = PipelineResult.TWO;
                break;
            case 3:
                pipelineResult = PipelineResult.THREE;
        }

        Thread runSlides = new Thread(() -> {
            while (opModeIsActive()) {
                bot.manipulator.verticalLinearSlides.loop();
                bot.manipulator.horizontalLinearSlides.loop();
            }
        });

        runSlides.run();

        if (autonomousType == AutonomousType.STACK) { autoPaths.drive.followTrajectory(autoPaths.stack);}
        autoPaths.drive.followTrajectory(autoPaths.park);


        telemetry.addData("Park Number", PARK_NUMBER);
        telemetry.update();

        if (isStopRequested()) {
            return;
        }

        while (!isStopRequested() && opModeIsActive()) { }
    }

}

