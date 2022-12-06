package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlides {

    private enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }



    private static final int EXTENDED_POSITION = 640;
    private static final int RETRACTED_POSITION = 34;
    private static final int MAXIMUM_POSITION = 680;
    //junction heights TODO: find values for junction heights
    private static final int GROUND_HEIGHT = 50;
    private static final int LOW_HEIGHT = 200;
    private static final int MEDIUM_HEIGHT = 400;
    private static final int HIGH_HEIGHT = 600;
    private static Level currentLevel = Level.GROUND;
    private static int targetHeight = 34;



    private static final double kP = 0.05;
    private static final double TOLERANCE = 31;

    private static final double SLIDE_SPEED = 0.3;
    private static final double HOLDING_SPEED = 0.1;
    private static final double STOPPED_SPEED = 0.03;
    private static final double RETRACT_SPEED = 0.3;


    private static final double RETRACT_POSITION_COEFF=0.05;


    private final MotorEx leftSlideMotor;
    private final MotorEx rightSlideMotor;

    public LinearSlides(HardwareMap hardwareMap) {


        leftSlideMotor = new MotorEx(hardwareMap, "leftSlideMotor", Motor.GoBILDA.RPM_312);
        rightSlideMotor = new MotorEx(hardwareMap, "rightSlideMotor", Motor.GoBILDA.RPM_312);



        initializeSlideMotor(leftSlideMotor);
        initializeSlideMotor(rightSlideMotor);
    }


    private void initializeSlideMotor(MotorEx motor) {
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
        motor.setPositionCoefficient(kP);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
    public double getStopSpeed(){
        switch(currentLevel){
            case GROUND:
                return 0.0; //
            case LOW:
                return 0.15; //
            case MEDIUM:
                return 0.15; //
            case HIGH:
                return 0.15; //
        }
        return 0.0;

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

    public void extend() {
        setTargetHeight();
        leftSlideMotor.setTargetPosition(targetHeight);
        rightSlideMotor.setTargetPosition(targetHeight);
    }

    public void retract() {
        leftSlideMotor.setTargetPosition(RETRACTED_POSITION);
        rightSlideMotor.setTargetPosition(RETRACTED_POSITION);
    }

    public void extendRightMotor(){
        long startTime=System.nanoTime();
        while(System.nanoTime()<startTime+1e8){
            rightSlideMotor.set(SLIDE_SPEED);
        }

    }
    public void extendLeftMotor(){
        long startTime=System.nanoTime();
        while(System.nanoTime()<startTime+1e8){
            leftSlideMotor.set(SLIDE_SPEED);
        }

    }

    public void periodic() {
        if (!(leftSlideMotor.atTargetPosition() && rightSlideMotor.atTargetPosition())) {
            if (Math.abs(leftSlideMotor.getCurrentPosition()) < Math.abs(leftSlideMotor.getCurrentPosition()) || Math.abs(rightSlideMotor.getCurrentPosition()) < Math.abs(rightSlideMotor.getCurrentPosition())) {
                leftSlideMotor.setPositionCoefficient(RETRACT_POSITION_COEFF);
                rightSlideMotor.setPositionCoefficient(RETRACT_POSITION_COEFF);
                leftSlideMotor.set(RETRACT_SPEED);
                rightSlideMotor.set(RETRACT_SPEED);
            } else {
                leftSlideMotor.set(SLIDE_SPEED);
                rightSlideMotor.set(SLIDE_SPEED);
                switch (currentLevel) {
                    case HIGH:
                        leftSlideMotor.setPositionCoefficient(0.30);
                        rightSlideMotor.setPositionCoefficient(0.015);
                        break;
                    case MEDIUM:
                        leftSlideMotor.setPositionCoefficient(0.034);
                        rightSlideMotor.setPositionCoefficient(0.017);
                        break;
                    case LOW:
                        leftSlideMotor.setPositionCoefficient(0.30);
                        rightSlideMotor.setPositionCoefficient(0.15);
                        break;
                    case GROUND:
                        leftSlideMotor.setPositionCoefficient(0.70);
                        rightSlideMotor.setPositionCoefficient(0.35);
                }
            }
        }
        else{
            leftSlideMotor.set(getStopSpeed());
            rightSlideMotor.set(getStopSpeed());
        }
    }
}