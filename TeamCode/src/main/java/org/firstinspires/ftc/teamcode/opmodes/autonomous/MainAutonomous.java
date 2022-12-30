package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.GlobalConfig.PipelineResult.ONE;
import static org.firstinspires.ftc.teamcode.GlobalConfig.PipelineResult.THREE;
import static org.firstinspires.ftc.teamcode.GlobalConfig.PipelineResult.TWO;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.apriltag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Main Autonomous", group = "Competition")
public class MainAutonomous extends LinearOpMode {//TODO: add reversing for competition

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    boolean performActions = true;
    GamepadEx gamepad;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    //720p
    double fx = 578.272;
    double fy = 578.272;
    double cx = 640;
    double cy = 360;

    // UNITS ARE METERS
    double tagsize = 0.075; //ONLY FOR TESTING

    // Tag ID 1,2,3 from the 36h11 family
    int ID_LEFT = 1;
    int ID_MIDDLE = 2;
    int ID_RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    private Bot bot;

    @Override
    public void runOpMode() {

        bot.instance = null;

        bot = Bot.getInstance(this);

        AutoPaths paths = new AutoPaths(this);

        gamepad = new GamepadEx(gamepad1);

        telemetry.addData("Alliance", GlobalConfig.alliance);
        telemetry.update();

        //This is for cam config, if we are going to use the phones cam then we will most likely need to edit this
        //However, if we use an external webcam, then we won't have to edit anything here.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Note: When config a cam, the name will already be "Webcam 1".
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == ID_LEFT || tag.id == ID_MIDDLE || tag.id == ID_RIGHT )
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (!tagFound) {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
                telemetry.update();
                continue;
            }

            telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
            tagToTelemetry(tagOfInterest);

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        List<AutoPaths.AutoPathElement> trajectories;

        double trackWidth = 17.0;

        Vector2d startingVector = new Vector2d(-36.0, -72.0 + trackWidth / 2);
        Pose2d startingPose = new Pose2d(startingVector.getX(), (GlobalConfig.alliance == GlobalConfig.Alliance.RED) ? startingVector.getY() : - startingVector.getY(), (GlobalConfig.alliance == GlobalConfig.Alliance.RED) ? 0.0 : - 0.0);

        bot.roadRunner.setPoseEstimate(startingPose);

        /* Actually do something useful */

        if(tagOfInterest == null || tagOfInterest.id == ID_LEFT) {
            trajectories = paths.createTrajectory(ONE);
        }
        
        else if(tagOfInterest.id == ID_MIDDLE) {
            trajectories = paths.createTrajectory(TWO);
        }

        else {
            trajectories = paths.createTrajectory(THREE);
        }

        /* the actual movement */
        for (AutoPaths.AutoPathElement item : trajectories) {

            telemetry.addData("executing path element", item.getName());
            telemetry.update();

            if (item instanceof AutoPaths.AutoPathElement.Path) {
                bot.roadRunner.followTrajectory(((AutoPaths.AutoPathElement.Path) item).getTrajectory());
            } else if (item instanceof AutoPaths.AutoPathElement.Action && performActions) {
                ((AutoPaths.AutoPathElement.Action) item).getRunner().invoke();
            }

            if (isStopRequested())
                return;
        }



        // poseEstimate = bot.roadRunner.getPoseEstimate();



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
