package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private final double TOLERANCE = 50;

    private final double OPEN_CLAW_POSITION=0.3; //tune
    private final double CLOSE_CLAW_POSITION=0.0; //tune

    private final double IN_ARM_POSITION=0.0; //tune
    private final double OUT_ARM_POSITION=0.8; //tune

//    private Servo outtakeArm;
//    private Servo outtakeClaw;

    private MotorEx rightVerticalSlide;
    private MotorEx leftVerticalSlide;


    public enum Level {
        GROUND,
        HOVER,
        LOW,
        MEDIUM,
        HIGH
    }


    private final int GROUND_HEIGHT = 0;  //tune
    private final int HOVER_HEIGHT = 200;

    private final int LOW_HEIGHT = 1700;
    private final int MEDIUM_HEIGHT = 2800;
    private final int HIGH_HEIGHT =  4000;

    private int targetHeight;
    private Level currentLevel;




    private final double kPUpward=0.25;
    private final double kPDownward=0.06;
    private final double kS=0.004;
    private final double kV=0.01;

    public Outtake(HardwareMap hardwareMap){
        rightVerticalSlide=hardwareMap.get(MotorEx.class, "rightVerticalSlide");
        leftVerticalSlide=hardwareMap.get(MotorEx.class, "leftVerticalSlide");
        initializeMotor(rightVerticalSlide);
        initializeMotor(leftVerticalSlide);
    }

    private void initializeMotor(MotorEx motor){
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(GROUND_HEIGHT);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
    }

    public int[] getCurrentHeight() {

        return new int[]{rightVerticalSlide.getCurrentPosition(), leftVerticalSlide.getCurrentPosition()};
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
            leftVerticalSlide.setPositionCoefficient(kPUpward);
            rightVerticalSlide.setPositionCoefficient(kPUpward);
        }
        else{
            rightVerticalSlide.setPositionCoefficient(kPDownward);
            leftVerticalSlide.setPositionCoefficient(kPDownward);

        }
        targetHeight=height;
    }
    public void extend(){
        setTargetHeight();
        rightVerticalSlide.setTargetPosition(targetHeight);
        leftVerticalSlide.setTargetPosition(targetHeight);
    }
    public void extend(int ticks){
        if(ticks<0){
            return;
        }
        setTargetHeight(ticks);
        rightVerticalSlide.setTargetPosition(targetHeight);
        leftVerticalSlide.setTargetPosition(targetHeight);
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
        if(rightVerticalSlide.atTargetPosition()&&leftVerticalSlide.atTargetPosition()){
            rightVerticalSlide.set(kS);
            leftVerticalSlide.set(kS);
        }
        else{
            rightVerticalSlide.set(kV);
            leftVerticalSlide.set(kV);
        }
    }

//    public void outtake(){
//        openOuttakeClaw();
//        inOuttakeArm();
//        closeOuttakeClaw();
//        outOuttakeArm();
//
//    }
//    public void outtake(Level level){
//        currentLevel=level;
//        openOuttakeClaw();
//        inOuttakeArm();
//        extend();
//        closeOuttakeClaw();
//        outOuttakeArm();
//
//
//
//    }


//    public void inOuttakeArm(){
//        outtakeArm.setPosition(IN_ARM_POSITION);
//    }
//    public void outOuttakeArm(){
//        outtakeArm.setPosition(OUT_ARM_POSITION);
//    }
//    public void openOuttakeClaw(){
//        outtakeClaw.setPosition(OPEN_CLAW_POSITION);
//    }
//    public void closeOuttakeClaw(){
//        outtakeClaw.setPosition(CLOSE_CLAW_POSITION);
//    }
//

}
