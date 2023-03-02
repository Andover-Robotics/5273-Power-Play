package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GlobalConfig;

public class HorizontalLinearSlides {


    public enum Level {
        RETRACTED,
        MIDWAY,
        EXTENDED
    }

    public enum RunState {
        MANUAL,
        DISTANCE_SENSOR,
        PRESET
    }

    //TODO: Tune position controller

    private final double kP = 0.05;
    private final double kV = 0.01;

    //TODO: Find values for levels of extension(ticks)

    private static final int RETRACTED = -20;
    private static final int MIDWAY = 56;
    private static final int EXTENDED =  80;

    private static final double POWER_CONSTANT = 0.01;

    private final int TOLERANCE = 5;

    public static Level currentLevel = Level.RETRACTED;
    private static int targetHeight;

    public final MotorEx slideMotor;

    private final DistanceSensor distanceSensor;

    private RunState runState = RunState.PRESET;


    public HorizontalLinearSlides(HardwareMap hardwareMap) {
        slideMotor = new MotorEx(hardwareMap, "horizontalSlideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(slideMotor);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void initializeSlideMotor(MotorEx motor) {
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(RETRACTED);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
    }

    public int getCurrentHeight() { return slideMotor.getCurrentPosition(); }

    public int getTargetHeight(){
        return targetHeight;
    }

    public boolean atTargetHeight() { return slideMotor.atTargetPosition(); }

    private void setTargetHeight() {
        switch(currentLevel) {
            case RETRACTED:
                targetHeight = RETRACTED;
                break;
            case MIDWAY:
                targetHeight = MIDWAY;
                break;
            case EXTENDED:
                targetHeight = EXTENDED;
                break;
        }
    }

    public void setTargetHeight(int height) {
        slideMotor.setPositionCoefficient(kP);
        targetHeight = height;
    }

    public void runSlides(){
        setTargetHeight();
        slideMotor.setTargetPosition(targetHeight);
    }

    public void runSlides(int ticks){
        if (ticks < 0) { return;}
        setTargetHeight(ticks);
        slideMotor.setTargetPosition(targetHeight);
    }

    public void setLevel(Level level) {
        switch(level) {
            case RETRACTED:
                currentLevel = Level.RETRACTED;
                break;
            case MIDWAY:
                currentLevel = Level.MIDWAY;
                break;
            case EXTENDED:
                currentLevel = Level.EXTENDED;
                break;
        }

        runSlides();
    }
    public void incrementLevel() {
        switch(currentLevel) {
            case RETRACTED:
                currentLevel = Level.MIDWAY;
                break;
            case MIDWAY:
            case EXTENDED:
                currentLevel = Level.EXTENDED;
                break;
        }

        runSlides();
    }

    public void decrementLevel() {
        switch(currentLevel) {
            case RETRACTED:
            case MIDWAY:
                currentLevel = Level.RETRACTED;
                break;
            case EXTENDED:
                currentLevel = Level.MIDWAY;
                break;
        }

        runSlides();
    }

    public void extend() {
        currentLevel = Level.EXTENDED;
        runSlides();
    }

    public void retract() {
        currentLevel = Level.RETRACTED;
        runSlides();
    }

    public void setManual() { runState = RunState.MANUAL; }

    public void setRunUsingDistanceSensor() { runState = RunState.DISTANCE_SENSOR; }

    public void setPreset() { runState = RunState.PRESET; }

    public void runManual(double input) {
        slideMotor.set(input * POWER_CONSTANT);
    }

    public void runUsingDistanceSensor() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        int ticks = distanceToTicks(distance);
        runSlides(ticks);
    }

    private int distanceToTicks(double distance) {
        double angle = Math.acos((Math.pow(12.0, 2) + Math.pow(distance, 2) - Math.pow(14.0, 2)) / (2 * 12.0 * distance));

        if (!Double.isNaN(angle)) { // 1 full revolution

            return (int) ((Math.PI / 2 - angle) / (2 * Math.PI) * Motor.GoBILDA.RPM_312.getCPR());
        }

        return (int) ((Math.PI / 2) / (2 * Math.PI) * Motor.GoBILDA.RPM_312.getCPR());
    }


    public void loop(){
        switch(runState) {
            case DISTANCE_SENSOR:
                runUsingDistanceSensor();
                break;
            case PRESET:
                if (!slideMotor.atTargetPosition()) { slideMotor.set(kV); }
                break;
            case MANUAL:
        }
    }
}