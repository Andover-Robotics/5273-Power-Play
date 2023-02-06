package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.GregorianCalendar;

public class LinearSlides {

    public enum Level {
        HOVER,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }


    //TODO: find values for junction heights(ticks)




    private final double kPUpward=0.25;
    private final double kPDownward=0.06;
    private final double kS=0.004;
    private final double kV=0.01;


    private static final int HOVER_HEIGHT = 200;
    private static final int GROUND_HEIGHT = 0;
    private static final int LOW_HEIGHT = 1700;
    private static final int MEDIUM_HEIGHT = 2800;
    private static final int HIGH_HEIGHT =  4000;
    public static Level currentLevel = Level.GROUND;
    private static int targetHeight;

    private final int TOLERANCE=50;

    public final MotorEx slideMotor;


    public LinearSlides(HardwareMap hardwareMap) {

        slideMotor = new MotorEx(hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(slideMotor);
    }

    public void initializeSlideMotor(MotorEx motor) {
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(GROUND_HEIGHT);
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
            case HOVER:
                targetHeight = HOVER_HEIGHT;
                break;
            case GROUND:
                targetHeight = GROUND_HEIGHT;
                break;
            case LOW:
                targetHeight = LOW_HEIGHT;
                break;
            case MEDIUM:
                targetHeight = MEDIUM_HEIGHT;
                break;
            case HIGH:
                targetHeight = HIGH_HEIGHT;
                break;
        }
    }
    public void setTargetHeight(int height) {
        if(height>targetHeight){
            slideMotor.setPositionCoefficient(kPUpward);
        }
        else{
            slideMotor.setPositionCoefficient(kPDownward);
        }
        targetHeight=height;
    }

    public void extend(){
        setTargetHeight();
        slideMotor.setTargetPosition(targetHeight);
    }

    public void extend(int ticks){
        if(ticks<0){
            return;
        }
        setTargetHeight(ticks);
        slideMotor.setTargetPosition(targetHeight);
    }

    public void setLevel(Level level) {
        switch(level) {
            case HOVER:
                currentLevel = Level.HOVER;
                break;
            case GROUND:
                currentLevel = Level.GROUND;
                break;
            case LOW:
                currentLevel = Level.LOW;
                break;
            case MEDIUM:
                currentLevel = Level.MEDIUM;
                break;
            case HIGH:
                currentLevel = Level.HIGH;
                break;
        }
        extend();
    }
    public void incrementLevel() {
        switch(currentLevel) {
            case HOVER:
            case GROUND:
                currentLevel = Level.LOW;
                break;
            case LOW:
                currentLevel = Level.MEDIUM;
                break;
            case MEDIUM:
            case HIGH:
                currentLevel = Level.HIGH;
                break;
        }
        extend();
    }

    public void decrementLevel() {

        switch(currentLevel) {
            case HOVER:
            case GROUND:
            case LOW:
                currentLevel = Level.GROUND;
                break;
            case MEDIUM:
                currentLevel = Level.LOW;
                break;
            case HIGH:
                currentLevel = Level.MEDIUM;
                break;
        }
        extend();

    }
    public void hover() {
        currentLevel = Level.HOVER;
        extend();
    }

    public void retract() {
        currentLevel = Level.GROUND;
        extend();
    }
    public void loop(){
        if(slideMotor.atTargetPosition()){
            slideMotor.set(kS);
        }
        else{
            slideMotor.set(kV);
        }


    }
}