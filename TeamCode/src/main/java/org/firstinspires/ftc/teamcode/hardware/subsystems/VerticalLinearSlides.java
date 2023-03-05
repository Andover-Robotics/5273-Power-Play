package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// IMPORTANT: IF BATTERY VOLTAGE IS LESS THAN 13 THEN THE SLIDES DON'T WORK
public class VerticalLinearSlides extends SubsystemBase {

    public enum Level {
        HOVER,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    //TODO: find values for junction heights(ticks)

    private final double kPUpward = 0.1;
    private final double kPDownward = 0.01;

    private final double kS = 0.04; // 0.05
    private final double kV = 0.13; // 0.2


    private static final int HOVER_HEIGHT = -500;
    private static final int GROUND_HEIGHT = 0;
    private static final int LOW_HEIGHT = -300;
    private static final int MEDIUM_HEIGHT = -1120;
    private static final int HIGH_HEIGHT =  -1920;

    public static Level currentLevel = Level.GROUND;
    private static int targetHeight;

    private final int TOLERANCE = 50;

    private final MotorEx leftSlideMotor;
    private final MotorEx rightSlideMotor;


    public VerticalLinearSlides(HardwareMap hardwareMap) {
        rightSlideMotor = new MotorEx(hardwareMap, "rightVerticalSlideMotor", Motor.GoBILDA.RPM_435);
        initializeSlideMotor(rightSlideMotor);

        leftSlideMotor = new MotorEx(hardwareMap, "leftVerticalSlideMotor", Motor.GoBILDA.RPM_435);
        initializeSlideMotor(leftSlideMotor);
        targetHeight=-300;
    }

    public void resetSlideEncoders() {
        leftSlideMotor.resetEncoder();
        rightSlideMotor.resetEncoder();
    }
    public void initializeSlideMotor(MotorEx motor) {
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);

    }

    private final int SCUFFED_SLIDE_OFFSET = 0;
    public int getCurrentHeight() { return (rightSlideMotor.getCurrentPosition() + leftSlideMotor.getCurrentPosition()) / 2 + SCUFFED_SLIDE_OFFSET; }
    public int getCurrentLeftHeight() { return leftSlideMotor.getCurrentPosition();}
    public int getCurrentRightHeight() { return rightSlideMotor.getCurrentPosition();}

    public int getTargetHeight(){
        return targetHeight;
    }

    public boolean atTargetHeight() { return Math.abs(getCurrentHeight() - targetHeight) <= TOLERANCE; }

    /***
     * @Range: 0 to 1920 going up, -1920 to 0 going down
     */
    public int getHeightDifference() { return getCurrentHeight()-targetHeight; }

    /**
     * @return both parameters are negative
     */
    public boolean isAboveTargetHeight() {
        return getCurrentHeight() < targetHeight;
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
        if (height > targetHeight){
            rightSlideMotor.setPositionCoefficient(kPUpward);
            leftSlideMotor.setPositionCoefficient(kPUpward);
        }

        else {
            rightSlideMotor.setPositionCoefficient(kPDownward);
            leftSlideMotor.setPositionCoefficient(kPDownward);
        }

        targetHeight = height;
    }

    public void extend(int ticks){
        if (ticks > 0) { return;}
        targetHeight=ticks;
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
                break;
            case MEDIUM:
                currentLevel = Level.MEDIUM;
                break;
            case HIGH:
                currentLevel = Level.HIGH;
                break;
        }

        setTargetHeight();
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

setTargetHeight();
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

        setTargetHeight();
    }

    public void hover() {
        currentLevel = Level.HOVER;
        setTargetHeight();
    }

    public void retract() {
        currentLevel = Level.GROUND;
        setTargetHeight();
    }

    @Override
    public void periodic(){
        if (atTargetHeight()) {
            rightSlideMotor.set(kS);
            leftSlideMotor.set(kS);
        }

        else {
            if(isAboveTargetHeight()){
                rightSlideMotor.set(kV*getHeightDifference()*kPDownward);
                leftSlideMotor.set(kV*getHeightDifference()*kPDownward);
            }
            else{
                rightSlideMotor.set(kV*getHeightDifference()*kPUpward);
                leftSlideMotor.set(kV*getHeightDifference()*kPUpward);
            }
        }
//            rightSlideMotor.set(-Math.pow(200, (double) getCurrentHeight() / 3000 + 0.03));
//            leftSlideMotor.set(-Math.pow(200, (double) getCurrentHeight() / 3000 + 0.03));
//        }
//        else {
//            rightSlideMotor.set(-0.1 * Math.pow(200, (double) getCurrentHeight() / 3000 + 0.03));
//            leftSlideMotor.set(-0.1 * Math.pow(200, (double) getCurrentHeight() / 3000 + 0.03));
//        }
    }
}