package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;

public class AutoPaths {

    public final SampleMecanumDrive drive;

    public final TrajectorySequence stack;

    private static final double EPSILON = 2.0;
    private static final double TRACK_WIDTH = 15.0;
    private static double TRACK_LENGTH;
    private final ColorSensor colorSensor;

    //TODO: Tune positions(add turning) - Use RRPathVisualizer

    // private Pose2d intakePose = p2d( - 54.0 + TRACK_WIDTH / 2, - 12.0, PI / 2);
    

    private Pose2d startPose = new Pose2d(0, 0, 0);

    private int parkingPose=1;

    private Map<Integer, Integer> parkingPoses = new HashMap<>();





    public AutoPaths(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new SampleMecanumDrive(hardwareMap);
        colorSensor=hardwareMap.get(ColorSensor.class, "colorSensor");
        

        parkingPoses.put(1, -24);
        parkingPoses.put(2, 1);
        parkingPoses.put(3, 24);

        

        stack = drive.trajectorySequenceBuilder(startPose)
                .forward(18)
                .waitSeconds(1)
                .addTemporalMarker(()->{colorDetected(telemetry);})
                .forward(30)
                .waitSeconds(5)
                .strafeRight(parkingPoses.get(parkingPose))
                .build();

        telemetry.addData("parkingPose", parkingPose);
    }

    private int colorDetected(Telemetry telemetry){
        int red=colorSensor.red();
        int green=colorSensor.green();
        int blue = colorSensor.blue();
        telemetry.addData("color", colorSensor.red()+" "+colorSensor.green()+" "+colorSensor.blue());
            if(red > blue && red > green){
            this.parkingPose=1;
            return 1;
        }
            else if(green > blue && green > red){

            this.parkingPose=2;
            return 2;
        }
            else if(blue > red && blue > green){
            this.parkingPose= 3;
            return 3;

        }
    }



}
