package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Claw {

    //TODO: Tune claw values

    private final double OPEN_GRAB_CLAW_ANGLE = 0;
    private final double CLOSE_GRAB_CLAW_ANGLE = 60;

    private final double RAISED_ROTATE_CLAW_ANGLE = 135;
    private final double HOVER_ROTATE_CLAW_ANGLE = 30;
    private final double LOWERED_ROTATE_CLAW_ANGLE = 0;

    private final ServoEx rotateServo;
    private final ServoEx grabServo;

    public Claw(HardwareMap hardwareMap){

        grabServo = new SimpleServo(hardwareMap, "grabServo",OPEN_GRAB_CLAW_ANGLE,CLOSE_GRAB_CLAW_ANGLE , AngleUnit.DEGREES);
        grabServo.setInverted(false);

        rotateServo = new SimpleServo(hardwareMap, "rotateServo", RAISED_ROTATE_CLAW_ANGLE, LOWERED_ROTATE_CLAW_ANGLE, AngleUnit.DEGREES);
        rotateServo.setInverted(false);

    }

    public void openGrabClaw() {

        grabServo.turnToAngle(OPEN_GRAB_CLAW_ANGLE);

    }

    public void closeGrabClaw() {

        grabServo.turnToAngle(CLOSE_GRAB_CLAW_ANGLE);

    }

    public void raiseRotateClaw() {

        rotateServo.turnToAngle(RAISED_ROTATE_CLAW_ANGLE);

    }

    public void lowerRotateClaw() {

        rotateServo.turnToAngle(LOWERED_ROTATE_CLAW_ANGLE);

    }

    public void hoverRotateClaw() {

        rotateServo.turnToAngle(HOVER_ROTATE_CLAW_ANGLE);

    }

    public void intake() {

        openGrabClaw();
        lowerRotateClaw();
        closeGrabClaw();
        hoverRotateClaw();

    }

    public void outtake() {

        lowerRotateClaw();
        openGrabClaw();

    }

}
