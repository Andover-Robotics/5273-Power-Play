package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Subsystems;

public class AutoPaths {

    public final RRMecanumDrive drive;

//    public final Trajectory parkingOne;
//    public final Trajectory parkingTwo;
//    public final Trajectory parkingThree;

    public final Subsystems subsystems;

    public final Trajectory stackRight;

    public final Trajectory stackLeft;

    public final int CONE_ONE_HEIGHT;

    public AutoPaths(HardwareMap hardwareMap){
        drive = new RRMecanumDrive(hardwareMap);

        subsystems =new Subsystems(hardwareMap);

        stackRight=drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(1, 57, Math.toRadians(14.03)))
                .build();

        stackLeft=drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-1, 57, Math.toRadians(14.03)))
                .build();

        CONE_ONE_HEIGHT=550;


    }

}
