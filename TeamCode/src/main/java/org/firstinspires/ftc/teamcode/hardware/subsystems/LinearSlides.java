package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    private static final double kI = 0.01;
    private static final double kD = 0.005;
    private static final double kF = 0.002;

    private static final double TOLERANCE = 31;



    private final MotorEx leftSlideMotor;
    private final MotorEx rightSlideMotor;

    private final PIDFController linearSlidesPIDFController;

    public LinearSlides(HardwareMap hardwareMap) {


        leftSlideMotor = new MotorEx(hardwareMap, "leftSlideMotor", Motor.GoBILDA.RPM_312);
        rightSlideMotor = new MotorEx(hardwareMap, "rightSlideMotor", Motor.GoBILDA.RPM_312);
        linearSlidesPIDFController= new PIDFController(kP, kI, kD, kF);
        linearSlidesPIDFController.setTolerance(TOLERANCE);


        initializeSlideMotor(leftSlideMotor);
        initializeSlideMotor(rightSlideMotor);
    }


    private void initializeSlideMotor(MotorEx motor) {
        motor.setRunMode(Motor.RunMode.PositionControl);
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

    public Level getCurrentLevel() {
        return currentLevel;
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
        linearSlidesPIDFController.setSetPoint(targetHeight);
    }

    public void retract() {
        linearSlidesPIDFController.setSetPoint(RETRACTED_POSITION);
    }


    public void periodic() {
        while(!linearSlidesPIDFController.atSetPoint()){
            double output=linearSlidesPIDFController.calculate((leftSlideMotor.getCurrentPosition()+rightSlideMotor.getCurrentPosition())/2.0);
            leftSlideMotor.set(output);
            rightSlideMotor.set(output);
        }
    }
}