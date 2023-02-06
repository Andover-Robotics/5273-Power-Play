package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Manipulator;

public class AutoPaths {

    public final RRMecanumDrive drive;

    public final Trajectory parkingOne;
    public final Trajectory parkingTwo;
    public final Trajectory parkingThree;
    public final Manipulator outtake;

    public final Trajectory stackRight;

    public final Trajectory stackLeft;

    public final int CONE_ONE_HEIGHT;

    public AutoPaths(HardwareMap hardwareMap){
        drive = new RRMecanumDrive(hardwareMap);

        parkingOne=drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0,48), Math.toRadians(90))
                .splineTo(new Vector2d(24,0), Math.toRadians(0))
                .build();
        parkingTwo = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(48,0), Math.toRadians(0))
                .build();
        parkingThree = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0,48), Math.toRadians(-90))
                .splineTo(new Vector2d(24,0), Math.toRadians(0))
                .build();

        outtake =new Manipulator(hardwareMap);

        stackRight=drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0,48), Math.toRadians(-90))
                .splineTo(new Vector2d(24,0), Math.toRadians(0))
                .build();

        stackLeft=drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0,48), Math.toRadians(90))
                .splineTo(new Vector2d(24,0), Math.toRadians(0))
                .build();

        CONE_ONE_HEIGHT=550;


    }

}
