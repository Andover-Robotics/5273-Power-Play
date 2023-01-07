package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlides {

    private enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }


    //TODO: find values for junction heights(ticks)

    private static final int GROUND_HEIGHT = 50;
    private static final int LOW_HEIGHT = 500;
    private static final int MEDIUM_HEIGHT = 1000;
    private static final int HIGH_HEIGHT =  1400;
    public static Level currentLevel = Level.GROUND;
    private static int targetHeight;



    public final DcMotor leftSlideMotor;
    public final DcMotor rightSlideMotor;

    public LinearSlides(HardwareMap hardwareMap) {

        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        initializeSlideMotor(leftSlideMotor);
        initializeSlideMotor(rightSlideMotor);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);






    }

    private void initializeSlideMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void extend(){
        setTargetHeight();
        leftSlideMotor.setTargetPosition(targetHeight);
        rightSlideMotor.setTargetPosition(targetHeight);
        leftSlideMotor.setPower(0.5);
        rightSlideMotor.setPower(0.5);
    }







}