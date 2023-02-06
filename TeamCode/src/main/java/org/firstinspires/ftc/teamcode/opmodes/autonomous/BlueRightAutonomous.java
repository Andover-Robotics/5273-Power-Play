package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.apriltag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "BlueRightAutonomous", group = "Competition")
public class BlueRightAutonomous extends LinearOpMode {

    double fx = 578.272;
    double fy = 578.272;
    double cx = 640;
    double cy = 360;

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


        telemetry.addData("init done","(real)");
        telemetry.update();
        int PARK_NUMBER=2;
        while(!isStarted()&&!isStopRequested()){
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            for(AprilTagDetection tag: currentDetections){
                telemetry.addData("tag", tag.id);
                telemetry.update();
                if(tag.id==1 || tag.id==2 || tag.id==3){
                    PARK_NUMBER=tag.id;
                }
            }

            sleep(20);
        }

        waitForStart();
        autoPaths.outtake.claw.closeClaw();
        autoPaths.outtake.linearSlides.setLevel(LinearSlides.Level.HOVER);


        if(PARK_NUMBER==1){
            autoPaths.drive.followTrajectory(autoPaths.parkingOne);
        }
        else if(PARK_NUMBER==2){
            autoPaths.drive.followTrajectory(autoPaths.parkingTwo);
        }
        else{
            autoPaths.drive.followTrajectory(autoPaths.parkingThree);
        }


        telemetry.addData("Park Number", PARK_NUMBER);
        telemetry.update();











        if (isStopRequested()) {
            return;
        }


        while (!isStopRequested() && opModeIsActive()) {



        }
    }

}
