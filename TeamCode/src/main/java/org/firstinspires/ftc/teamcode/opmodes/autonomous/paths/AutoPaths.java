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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.HashMap;
import java.util.Map;

public class AutoPaths {

    public final SampleMecanumDrive drive;


    private static final double EPSILON = 2.0;
    private static final double TRACK_WIDTH = 15.0;
    private static double TRACK_LENGTH;

    public final TrajectorySequence detect;
    public final TrajectorySequence park;

    //TODO: Tune positions(add turning) - Use RRPathVisualizer

    // private Pose2d intakePose = p2d( - 54.0 + TRACK_WIDTH / 2, - 12.0, PI / 2);


    private Pose2d startPose = new Pose2d(0, 0, 0);

    public int parkingPose=1;

    private Map<Integer, Integer> parkingPoses = new HashMap<>();





    public AutoPaths(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new SampleMecanumDrive(hardwareMap);

<<<<<<< Updated upstream

        parkingPoses.put(1, -26);
        parkingPoses.put(2, 1);
        parkingPoses.put(3, 26);

        detect=drive.trajectorySequenceBuilder(startPose)
                .forward(18)
                .waitSeconds(5)
=======
        park = drive.trajectoryBuilder(startPose)
                .strafeRight(parkingPose.get(pipelineResult));
>>>>>>> Stashed changes
                .build();

        park = drive.trajectorySequenceBuilder(detect.end())
                .forward(30)
                .waitSeconds(1)
                .strafeRight(parkingPoses.get(parkingPose))
                .build();

        telemetry.addData("parkingPose", parkingPose);
        telemetry.update();
    }




}
