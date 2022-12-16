package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm {

    private final double MIN_ANGLE=0;
    private final double MAX_ANGLE=150;

    private final ServoEx leftServo;
    private final ServoEx rightServo;

    public Arm(HardwareMap hardwareMap){
        leftServo = new SimpleServo(hardwareMap, "leftServo", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES );
        rightServo = new SimpleServo(hardwareMap, "rightServo", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);

        leftServo.setInverted(false);
        rightServo.setInverted(false);
    }

    public void rotateOpenPosition(){
        leftServo.turnToAngle(MAX_ANGLE);
        rightServo.turnToAngle(MAX_ANGLE);
    }

    public void rotateClosedPosition(){
        leftServo.turnToAngle(MIN_ANGLE);
        rightServo.turnToAngle(MIN_ANGLE);
    }

}
