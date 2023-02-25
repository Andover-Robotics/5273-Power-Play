package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GlobalConfig;

public class HorizontalLinearSlides {

    private class Color {

        private final int COLOR_TOLERANCE = 20;
        public int r, g, b;

        public Color(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public boolean matchColor(Color c){
            return (Math.abs(this.r - c.r) < COLOR_TOLERANCE && Math.abs(this.g - c.g) < COLOR_TOLERANCE && Math.abs(this.b - c.b) < COLOR_TOLERANCE);
        }
    }

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

    private static final int RETRACTED = 0;
    private static final int MIDWAY = 1700;
    private static final int EXTENDED =  4000;

    private static final double POWER_CONSTANT = 0.05;

    //TODO: Tune TICKS_TO_INCHES

    private static final double TICKS_TO_INCHES = 56.8;

    private final int TOLERANCE = 50;

    public static Level currentLevel = Level.RETRACTED;
    private static int targetHeight;

    private final MotorEx rightSlideMotor;
    private final MotorEx leftSlideMotor;

    private final DistanceSensor distanceSensor;
    private final ColorSensor colorSensor;

    private RunState runState = RunState.PRESET;

    private Color BLUE_CONE = new Color(0, 0, 255);
    private Color RED_CONE = new Color(255, 0, 0);

    public HorizontalLinearSlides(HardwareMap hardwareMap) {
        rightSlideMotor = new MotorEx(hardwareMap, "horizontalRightSlideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(rightSlideMotor);

        leftSlideMotor = new MotorEx(hardwareMap, "horizontalLeftSlideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(leftSlideMotor);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void initializeSlideMotor(MotorEx motor) {
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(RETRACTED);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
    }

    public int getCurrentHeight() { return (rightSlideMotor.getCurrentPosition() + leftSlideMotor.getCurrentPosition()) / 2; }

    public int getTargetHeight(){
        return targetHeight;
    }

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
        rightSlideMotor.setPositionCoefficient(kP);
        targetHeight = height;
    }

    public void runSlides(){
        setTargetHeight();
        rightSlideMotor.setTargetPosition(targetHeight);
    }

    public void runSlides(int ticks){
        if(ticks < 0){ return;}
        setTargetHeight(ticks);
        rightSlideMotor.setTargetPosition(targetHeight);
        leftSlideMotor.setTargetPosition(targetHeight);
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

    public boolean coneDetected() {
        Color colorDetected = new Color(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        return colorDetected.matchColor((GlobalConfig.alliance == GlobalConfig.Alliance.BLUE) ? BLUE_CONE : RED_CONE);
    }

    public void runManual(double input) {
        rightSlideMotor.set(input * POWER_CONSTANT);
        leftSlideMotor.set(input * POWER_CONSTANT);
    }

    public void runUsingDistanceSensor() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        runSlides((int) (distance * TICKS_TO_INCHES));
    }


    public void loop(){
        switch(runState) {
            case DISTANCE_SENSOR:
                runUsingDistanceSensor();
                break;
            case PRESET:
                if (!rightSlideMotor.atTargetPosition()) { rightSlideMotor.set(kV); }
                break;
            case MANUAL:
        }
    }
}