package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;
import static java.lang.Math.PI;

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

    private static final double EPSILON = 2.0;
    private static final double TRACK_WIDTH = 15.0;
    private static double TRACK_LENGTH;

    //TODO: Tune positions(add turning) - Use RRPathVisualizer

    // private Pose2d intakePose = p2d( - 54.0 + TRACK_WIDTH / 2, - 12.0, PI / 2);

    private Pose2d deliveryPose = p2d(- 34.0, - 7.0, PI);

    private Pose2d startPose = p2d(- 72.0 + TRACK_WIDTH / 2, - 36.0, PI);

    private Map<GlobalConfig.PipelineResult, Pose2d> parkingPose = new HashMap<>();

    public final Trajectory stack;

    private final Bot bot = Bot.getInstance();

    public AutoPaths(HardwareMap hardwareMap) {
        drive = new RRMecanumDrive(hardwareMap);

        Pose2d parkingTrajectoryStartPose = (autonomousType == AutonomousType.STACK) ? deliveryPose : startPose;

        parkingPose.put(PipelineResult.ONE, p2d(- 12.0, parkingTrajectoryStartPose.getY(), PI));
        parkingPose.put(PipelineResult.TWO, p2d(- 36.0, parkingTrajectoryStartPose.getY(), PI));
        parkingPose.put(PipelineResult.THREE, p2d(- 60.0, parkingTrajectoryStartPose.getY(), PI));

        park = drive.trajectoryBuilder(parkingTrajectoryStartPose, parkingTrajectoryStartPose.getHeading())
                .lineTo(p2v(parkingPose.get(GlobalConfig.pipelineResult)))
                .build();

        //Cycling in place

        stack = drive.trajectoryBuilder(startPose)
                .lineTo(p2v(deliveryPose))
                .addTemporalMarker(4.01, () -> bot.manipulator.prepareToOuttake())
                .addTemporalMarker(4.01, () -> bot.manipulator.verticalLinearSlides.extend())
                .addTemporalMarker(5.01, () -> bot.manipulator.extendVerticalArm())
                .addTemporalMarker(7.01, () -> bot.manipulator.openVerticalClaw())
                .addTemporalMarker(8.01, () -> bot.manipulator.resetVertical())
                .addTemporalMarker(8.01, () -> bot.manipulator.horizontalLinearSlides.extend())
                .addTemporalMarker(8.01, () -> bot.manipulator.intake())
                .addTemporalMarker(10.01, () -> bot.manipulator.prepareToOuttake())
                .addTemporalMarker(10.01, () -> bot.manipulator.verticalLinearSlides.extend())
                .addTemporalMarker(11.01, () -> bot.manipulator.extendVerticalArm())
                .addTemporalMarker(12.01, () -> bot.manipulator.openVerticalClaw())
                .addTemporalMarker(12.01, () -> bot.manipulator.resetVertical())
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
