package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalLinearSlides {

    public enum Level {
        RETRACTED,
        MIDWAY,
        EXTENDED
    }

    //TODO: Tune position controller

    private final double kPUpward = 0.25;
    private final double kPDownward = 0.06;

    private final double kS = 0.004;
    private final double kV = 0.01;

    //TODO: Find values for levels of extension(ticks)

    private static final int RETRACTED = 0;
    private static final int MIDWAY = 1700;
    private static final int EXTENDED =  4000;

    public static Level currentLevel = Level.RETRACTED;
    private static int targetHeight;

    private final int TOLERANCE = 50;

    public final MotorEx slideMotor;


    public HorizontalLinearSlides(HardwareMap hardwareMap) {
        slideMotor = new MotorEx(hardwareMap, "horizontalSlideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(slideMotor);
    }

    public void initializeSlideMotor(MotorEx motor) {
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(RETRACTED);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
    }

    public int getCurrentHeight() {
        return slideMotor.getCurrentPosition();
    }

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
        if (height > targetHeight) { slideMotor.setPositionCoefficient(kPUpward); }

        else { slideMotor.setPositionCoefficient(kPDownward); }

        targetHeight = height;
    }

    public void runSlides(){
        setTargetHeight();
        slideMotor.setTargetPosition(targetHeight);
    }

    public void runSlides(int ticks){
        if(ticks < 0){ return;}
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

    public void loop(){
        if (slideMotor.atTargetPosition()) { slideMotor.set(kS); }
        else { slideMotor.set(kV); }
    }
}