package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;
import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Bot;

import java.util.HashMap;
import java.util.Map;

public class AutoPaths {

    public final RRMecanumDrive drive;

    public final Trajectory park;

    private static double EPSILON = 2.0;
    private static double TRACK_WITDH = 15.0;

    //TODO: Tune positions(add turning) - Use RRPathVisualizer

    private Pose2d intakePose = p2d( - 54.0 + TRACK_WITDH / 2, - 12.0, PI / 2);

    private Pose2d deliveryPose = p2d(- 24.0 - TRACK_WITDH * sqrt(2.0) / 4 - EPSILON,
                    - EPSILON * sqrt(2.0) / 4 - EPSILON,
                    7 * PI / 4);

    private Pose2d startPose = p2d(-36.0, - 72.0 + TRACK_WITDH / 2, 0.0);

    private Map<GlobalConfig.PipelineResult, Pose2d> parkingPose = new HashMap<>();

    public final Trajectory stack;

    private final Bot bot = Bot.getInstance();

    public AutoPaths(HardwareMap hardwareMap) {
        drive = new RRMecanumDrive(hardwareMap);

        parkingPose.put(PipelineResult.ONE, p2d(- 12.0, - 36.0, PI));
        parkingPose.put(PipelineResult.TWO, p2d(- 36.0, -36.0, PI));
        parkingPose.put(PipelineResult.THREE, p2d(- 60.0, - 36.0, PI));

        park = drive.trajectoryBuilder((autonomousType == AutonomousType.STACK) ? deliveryPose : startPose)
                .splineTo(p2v(parkingPose.get(GlobalConfig.pipelineResult)), PI / 2)
                .build();

        stack = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(deliveryPose)
                .lineToLinearHeading(intakePose)
                .lineToLinearHeading(deliveryPose)
                .addTemporalMarker(0.01, () -> bot.outtake.outtake())
                .addTemporalMarker(5.01, () -> bot.outtake.reset())
                .addTemporalMarker(10.01, () -> bot.outtake.intake())
                .addTemporalMarker(15.01, () -> bot.outtake.outtake())
                .addTemporalMarker(20.01, () -> bot.outtake.reset())
                .build();
    }

    private Pose2d p2d(double x, double y, double h) {
        return new Pose2d((side == Side.AUDIENCE) ? x : - x,
                (alliance == Alliance.RED) ? y : - y,
                (side == Side.AUDIENCE) ? ((alliance == Alliance.RED) ? h : (PI - h)):
                        2 * Math.PI - ((alliance == Alliance.RED) ? h : (PI - h)));
    }

    private Vector2d p2v(Pose2d pose) { return new Vector2d(pose.getX(), pose.getY()); }

}
