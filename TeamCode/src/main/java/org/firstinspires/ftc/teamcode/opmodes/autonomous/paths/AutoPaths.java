package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Bot;

import java.util.HashMap;
import java.util.Map;

public class AutoPaths {

    public final RRMecanumDrive drive;

    public final Trajectory park;

    private static final double EPSILON = 2.0;
    private static final double TRACK_WIDTH = 15.0;
    private static double TRACK_LENGTH;
    private final ColorSensor colorSensor;

    //TODO: Tune positions(add turning) - Use RRPathVisualizer

    // private Pose2d intakePose = p2d( - 54.0 + TRACK_WIDTH / 2, - 12.0, PI / 2);
    

    private Pose2d startPose = new Pose2d(0, 0, 0);

    private Map<GlobalConfig.PipelineResult, Pose2d> parkingPose = new HashMap<>();
    


    private final Bot bot = Bot.getInstance();

    public AutoPaths(HardwareMap hardwareMap) {
        drive = new RRMecanumDrive(hardwareMap);
        colorSensor=hardwareMap.get(ColorSensor.class, "colorSensor");
        

        parkingPose.put(PipelineResult.ONE, new Pose2d(-24, 48, 0));
        parkingPose.put(PipelineResult.TWO, new Pose2d(0, 48, 0));
        parkingPose.put(PipelineResult.THREE, new Pose2d(24, 48, 0));

        

        park = drive.trajectoryBuilder(startPose)
                .forward(48)
                .addTemporalMarker(1,  ()->{colorDetected();})
                .lineToSplineHeading(parkingPose.get(pipelineResult))
                .build();
    }

    private void colorDetected(){
        if(colorSensor.red()>colorSensor.green() &&colorSensor.red()>colorSensor.blue()){
            pipelineResult=PipelineResult.ONE;
        }
        else if(colorSensor.green()>colorSensor.red() &&colorSensor.green()>colorSensor.blue()){
            pipelineResult=PipelineResult.TWO;
        }
        else if(colorSensor.blue()>colorSensor.green() &&colorSensor.blue()>colorSensor.red()){
            pipelineResult= PipelineResult.THREE;
        }
    }



}
