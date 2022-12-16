package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Claw {

    private final double OPEN_GRAB_CLAW_ANGLE=0;
    private final double CLOSE_GRAB_CLAW_ANGLE=60;
    private final double OPEN_ROTATE_CLAW_ANGLE=135;
    private final double CLOSE_ROTATE_CLAW_ANGLE=0;


    private final ServoEx rotateServo;
    private final ServoEx grabServo;

    public Claw(HardwareMap hardwareMap){
        grabServo = new SimpleServo(hardwareMap, "grabServo",OPEN_GRAB_CLAW_ANGLE,CLOSE_GRAB_CLAW_ANGLE , AngleUnit.DEGREES);
        grabServo.setInverted(false);

        rotateServo = new SimpleServo(hardwareMap, "rotateServo",OPEN_ROTATE_CLAW_ANGLE,CLOSE_ROTATE_CLAW_ANGLE , AngleUnit.DEGREES);
        rotateServo.setInverted(false);
    }

    public void openGrabClaw(){
        grabServo.turnToAngle(OPEN_GRAB_CLAW_ANGLE);
    }
    public void closeGrabClaw(){
        grabServo.turnToAngle(CLOSE_GRAB_CLAW_ANGLE);
    }
    public void openRotateClaw(){
        rotateServo.turnToAngle(OPEN_ROTATE_CLAW_ANGLE);
    }
    public void closeRotateClaw(){
        rotateServo.turnToAngle(CLOSE_ROTATE_CLAW_ANGLE);
    }




}
