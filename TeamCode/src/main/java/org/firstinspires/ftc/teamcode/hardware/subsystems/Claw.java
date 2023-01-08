package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;


public class Claw {

    //TODO: Tune claw values

    private final double OPEN_GRAB_CLAW_POSITION = 0.70;
    private final double CLOSE_GRAB_CLAW_POSITION =0.45;

    private final double RAISED_ROTATE_CLAW_POSITION = 0.85;

    private final double HOVER_ROTATE_CLAW_POSITION = 0.80;
    private final double LOWERED_ROTATE_CLAW_POSITION =0.40; //0.30

    private final Servo rotateServo;
    private final Servo grabServo;

    public Claw(HardwareMap hardwareMap){


        grabServo = hardwareMap.get(Servo.class, "grabServo");

        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

    }

    public double getRotatePosition() {
        return rotateServo.getPosition();
    }

    public void openGrabClaw() {

        grabServo.setPosition(OPEN_GRAB_CLAW_POSITION);

    }

    public void closeGrabClaw() {

        grabServo.setPosition(CLOSE_GRAB_CLAW_POSITION);

    }

    // TODO: implement function to move the claw backwards a bit while linear slide is raising so that the cone doesn't get caught
    public void raiseRotateClaw() {

        rotateServo.setPosition(RAISED_ROTATE_CLAW_POSITION);

    }

    public void hoverRotateClaw(){
        rotateServo.setPosition(HOVER_ROTATE_CLAW_POSITION);
    }

    public void lowerRotateClaw() {

        rotateServo.setPosition(LOWERED_ROTATE_CLAW_POSITION);

    }

    public void intake() {
        openGrabClaw();
        lowerRotateClaw();
        closeGrabClaw();
        hoverRotateClaw();
    }
    public void outtake(){
        raiseRotateClaw();
        openGrabClaw();
        hoverRotateClaw();

    }

    public void scrollArm(double power){
        if (rotateServo.getPosition() <= RAISED_ROTATE_CLAW_POSITION && power > 0
            || rotateServo.getPosition() >= LOWERED_ROTATE_CLAW_POSITION && power < 0)
            rotateServo.setPosition(rotateServo.getPosition() + power*0.03);
    }


}
