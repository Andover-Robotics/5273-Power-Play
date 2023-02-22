package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;


public class HorizontalArm {

    //TODO: Tune claw values

    private final double OPEN_CLAW_POSITION = 0.20;
    private final double CLOSE_CLAW_POSITION = 0.0;

    private final double ROTATE_CLAW_IN_POSITION = 0.50;
    private final double ROTATE_CLAW_OUT_POSITION = 0.0;

    private final double ARM_OUT_POSITION = 0.50;
    private final double ARM_IN_POSITION = 0.0;

    private final double CONE_INTERVAL = 0.04;

    private final Servo claw;
    private final Servo rotateServo;
    private final Servo arm;

    public HorizontalArm (HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "horizontalClaw");
        arm = hardwareMap.get(Servo.class, "horizontalArm");
        rotateServo = hardwareMap.get(Servo.class, "horizontalRotateServo");
    }

    public double getCloseClawPosition() { return CLOSE_CLAW_POSITION; }

    public double getOpenClawPosition() { return OPEN_CLAW_POSITION; }

    public double getClawPosition() { return claw.getPosition(); }

    public double getRotateServoPosition() { return rotateServo.getPosition(); }

    public double getArmPosition() { return arm.getPosition(); }

    public void openClaw() { claw.setPosition(OPEN_CLAW_POSITION);}

    public void closeClaw() { claw.setPosition(CLOSE_CLAW_POSITION); }

    public void extendArm() { arm.setPosition(ARM_OUT_POSITION); }

    public void retractArm() { arm.setPosition(ARM_IN_POSITION); }

    public void uprightClaw() { rotateServo.setPosition(ROTATE_CLAW_OUT_POSITION);}

    public void flipClaw() { rotateServo.setPosition(ROTATE_CLAW_IN_POSITION); }

    public void rotateArmToConeStack(int cones) { arm.setPosition(ARM_OUT_POSITION + CONE_INTERVAL * cones); }

}
