package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.GregorianCalendar;

public class LinearSlides {

    private enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }


    //TODO: find values for junction heights(ticks)

    private  static final double POWER = 1;

    private static final double kP= 5; //TODO: Tune
    private static final double kI= 5; //TODO: Tune
    private static final double kD= 5; //TODO: Tune

    private static final int GROUND_HEIGHT = 0;
    private static final int LOW_HEIGHT = 1700;
    private static final int MEDIUM_HEIGHT = 2800;
    private static final int HIGH_HEIGHT =  4000;
    public static Level currentLevel = Level.GROUND;
    private static int targetHeight;

    public final DcMotorEx slideMotor;


    public LinearSlides(HardwareMap hardwareMap) {

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        initializeSlideMotor(slideMotor);
    }

    public void initializeSlideMotor(DcMotorEx motor) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(GROUND_HEIGHT);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPositionPIDFCoefficients(kP);
    }

    public int getCurrentHeight() {
        return slideMotor.getCurrentPosition();
    }
    public int getTargetHeight(){
        return targetHeight;
    }
    private void setTargetHeight() {

        switch(currentLevel) {
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
    private void setTargetHeight(int height) {
        targetHeight=height;
    }

    public void extend(){
        setTargetHeight();
        slideMotor.setTargetPosition(targetHeight);
        slideMotor.setPower(POWER);
    }

    public void extend(int ticks){
        targetHeight = ticks;
        slideMotor.setTargetPosition(targetHeight);
        slideMotor.setPower(POWER);
    }

    public void incrementLevel() {
        switch(currentLevel) {
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

    public void retract() {
        currentLevel = Level.GROUND;
        extend();
    }
}