package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HorizontalLinearSlides extends SubsystemBase {



    //TODO: Tune position controller

    private final double kP = 0.05;

    //TODO: Find values for levels of extension(ticks)


    private static final double POWER_CONSTANT = 0.02;

    private boolean RETRACTED = true;
    private final int TOLERANCE = 4;
    private static int targetPos = 0;

    public final MotorEx slideMotor;

    private final DistanceSensor distanceSensor;



    public HorizontalLinearSlides(HardwareMap hardwareMap) {
        slideMotor = new MotorEx(hardwareMap, "horizontalSlideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setPositionTolerance(TOLERANCE);
        slideMotor.setPositionCoefficient(kP);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }



    public int curPos() { return slideMotor.getCurrentPosition(); }

    public int targetPos(){
        return targetPos;
    }

    public double getDist() { return distanceSensor.getDistance(DistanceUnit.INCH); }

    public boolean atTargetHeight() { return slideMotor.atTargetPosition(); }

    public void retractSlides() {
        slideMotor.setRunMode(Motor.RunMode.RawPower);
        slideMotor.set(1.0);
        RETRACTED = true;
        targetPos = 0;
    }

    public void extendSlides() {
        RETRACTED = false;
        slideMotor.stopMotor();
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        targetPos = -distanceToTicks(distanceSensor.getDistance(DistanceUnit.INCH));
        slideMotor.setTargetPosition(targetPos);

    }

    public void shiftManual(int shift) {
        targetPos -= shift;
        slideMotor.setTargetPosition(targetPos);
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
        if(!RETRACTED) {
            slideMotor.set(1);
        }

        if(curPos() > 1) { slideMotor.resetEncoder(); slideMotor.stopMotor(); }
    }
}