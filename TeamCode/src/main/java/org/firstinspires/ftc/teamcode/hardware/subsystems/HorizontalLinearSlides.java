package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HorizontalLinearSlides extends SubsystemBase {



    public enum SlidePos {
        INIT_POS,
        TRANSFER_POS,
        EXTENDED
    }


    //TODO: Tune position controller

    private final double kP = 0.05;
    private final double kV = 0.01;

    //TODO: Find values for levels of extension(ticks)

    private static final int RETRACTED = -20;
    private static final int MIDWAY = 56;
    private static final int EXTENDED =  80;

    private static final double POWER_CONSTANT = 0.01;

    private final int TOLERANCE = 3;

    public static SlidePos currentSlidePos = SlidePos.INIT_POS;
    private static int targetPos;

    public final MotorEx slideMotor;

    private final DistanceSensor distanceSensor;



    public HorizontalLinearSlides(HardwareMap hardwareMap) {
        slideMotor = new MotorEx(hardwareMap, "horizontalSlideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setPositionTolerance(TOLERANCE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }



    public int getCurrentPos() { return slideMotor.getCurrentPosition(); }

    public int getTargetPos(){
        return targetPos;
    }

    public boolean atTargetHeight() { return slideMotor.atTargetPosition(); }


    public void runUsingDistanceSensor() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        int ticks = distanceToTicks(distance);
    }

    private int distanceToTicks(double distance) {
        double angle = Math.acos((Math.pow(12.0, 2) + Math.pow(distance, 2) - Math.pow(14.0, 2)) / (2 * 12.0 * distance));

        if (!Double.isNaN(angle)) { // 1 full revolution

            return (int) ((Math.PI / 2 - angle) / (2 * Math.PI) * Motor.GoBILDA.RPM_312.getCPR());
        }

        return (int) ((Math.PI / 2) / (2 * Math.PI) * Motor.GoBILDA.RPM_312.getCPR());
    }


    @Override
    public void periodic() {
        if(currentSlidePos == SlidePos.INIT_POS) {
            slideMotor.set(-1.0);
        }
        else if(currentSlidePos == SlidePos.TRANSFER_POS) {

        }
        else if(currentSlidePos == SlidePos.EXTENDED) {
            // logic here
        }
    }
}