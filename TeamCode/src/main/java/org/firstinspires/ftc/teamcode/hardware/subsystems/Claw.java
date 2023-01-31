package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;


public class Claw {

    //TODO: Tune claw values

    private final double OPEN_CLAW_POSITION = 0.20;
    private final double CLOSE_CLAW_POSITION =0.0;
    private final Servo claw;

    public Claw(HardwareMap hardwareMap){

        claw = hardwareMap.get(Servo.class, "claw");

    }

    public double getRotatePosition() {
        return claw.getPosition();
    }

    public void openClaw() {

        claw.setPosition(OPEN_CLAW_POSITION);

    }

    public void closeClaw() {

        claw.setPosition(CLOSE_CLAW_POSITION);

    }

    public void intake() {

    }
    public void outtake(){

    }


}
