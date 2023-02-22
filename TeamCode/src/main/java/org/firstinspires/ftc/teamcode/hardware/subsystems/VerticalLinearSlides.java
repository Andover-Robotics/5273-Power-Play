package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java
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

========

public class VerticalLinearSlides {
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java

    public enum Level {
        GROUND,
        HOVER,
        LOW,
        MEDIUM,
        HIGH
    }

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java

    private final int GROUND_HEIGHT = 0;  //tune
    private final int HOVER_HEIGHT = 200;

    private final int LOW_HEIGHT = 1700;
    private final int MEDIUM_HEIGHT = 2800;
    private final int HIGH_HEIGHT =  4000;

    private int targetHeight;
    private Level currentLevel;
========
    //TODO: find values for junction heights(ticks)
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java

    private final double kPUpward = 0.25;
    private final double kPDownward = 0.06;

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java


    private final double kPUpward=0.25;
    private final double kPDownward=0.06;
    private final double kS=0.004;
    private final double kV=0.01;

    public Outtake(HardwareMap hardwareMap){
        rightVerticalSlide=hardwareMap.get(MotorEx.class, "rightVerticalSlide");
        leftVerticalSlide=hardwareMap.get(MotorEx.class, "leftVerticalSlide");
        initializeMotor(rightVerticalSlide);
        initializeMotor(leftVerticalSlide);
========
    private final double kS = 0.004;
    private final double kV = 0.01;

    private static final int HOVER_HEIGHT = 200;
    private static final int GROUND_HEIGHT = 0;
    private static final int LOW_HEIGHT = 1700;
    private static final int MEDIUM_HEIGHT = 2800;
    private static final int HIGH_HEIGHT =  4000;

    public static Level currentLevel = Level.GROUND;
    private static int targetHeight;

    private final int TOLERANCE=50;

    public final MotorEx leftSlideMotor;
    public final MotorEx rightSlideMotor;


    public VerticalLinearSlides(HardwareMap hardwareMap) {
        rightSlideMotor = new MotorEx(hardwareMap, "rightVerticalSlideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(rightSlideMotor);

        leftSlideMotor = new MotorEx(hardwareMap, "leftVerticalSlideMotor", Motor.GoBILDA.RPM_312);
        initializeSlideMotor(leftSlideMotor);
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java
    }

    private void initializeMotor(MotorEx motor){
        motor.resetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(GROUND_HEIGHT);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
    }

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java
    public int[] getCurrentHeight() {

        return new int[]{rightVerticalSlide.getCurrentPosition(), leftVerticalSlide.getCurrentPosition()};
    }
========
    public int getCurrentHeight() { return (rightSlideMotor.getCurrentPosition() + leftSlideMotor.getCurrentPosition()) / 2; }
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java

    public int getTargetHeight(){

        return targetHeight;
    }
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java

    private void setTargetHeight() {
========
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java

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
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java
        if(height>targetHeight){
            leftVerticalSlide.setPositionCoefficient(kPUpward);
            rightVerticalSlide.setPositionCoefficient(kPUpward);
        }
        else{
            rightVerticalSlide.setPositionCoefficient(kPDownward);
            leftVerticalSlide.setPositionCoefficient(kPDownward);

========
        if (height > targetHeight){
            rightSlideMotor.setPositionCoefficient(kPUpward);
            leftSlideMotor.setPositionCoefficient(kPUpward);
        }

        else {
            rightSlideMotor.setPositionCoefficient(kPDownward);
            leftSlideMotor.setPositionCoefficient(kPDownward);
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java
        }

        targetHeight = height;
    }
    public void extend(){
        setTargetHeight();
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java
        rightVerticalSlide.setTargetPosition(targetHeight);
        leftVerticalSlide.setTargetPosition(targetHeight);
========
        rightSlideMotor.setTargetPosition(targetHeight);
        leftSlideMotor.setTargetPosition(targetHeight);
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java
    }
    public void extend(int ticks){
        if (ticks < 0) { return;}
        setTargetHeight(ticks);
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java
        rightVerticalSlide.setTargetPosition(targetHeight);
        leftVerticalSlide.setTargetPosition(targetHeight);
========
        rightSlideMotor.setTargetPosition(targetHeight);
        leftSlideMotor.setTargetPosition(targetHeight);
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java
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
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/Outtake.java
        if(rightVerticalSlide.atTargetPosition()&&leftVerticalSlide.atTargetPosition()){
            rightVerticalSlide.set(kS);
            leftVerticalSlide.set(kS);
        }
        else{
            rightVerticalSlide.set(kV);
            leftVerticalSlide.set(kV);
        }
========
        if (Math.abs(getCurrentHeight() - targetHeight) <= TOLERANCE) {
            rightSlideMotor.set(kS);
            leftSlideMotor.set(kS);
        }

        else {
            rightSlideMotor.set(kV);
            leftSlideMotor.set(kV);
        }
>>>>>>>> 5633bc9 (Streamlined Autonomous, Tweaked Paths(1+1 atm, goal is 1+3); Added New Subsystem Code. TODO: New Control Scheme, Tune PID, Finalize Subsystem Code, and Tune Autonomous):TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/VerticalLinearSlides.java
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
