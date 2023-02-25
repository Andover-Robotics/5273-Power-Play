package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;


public class VerticalArm {

    public enum ClawPosition {
        OPEN,
        CLOSED
    }

    public enum ArmPosition {
        IN,
        OUT
    }

    public enum RotateServoPosition {
        UP,
        DOWN
    }

    public enum PivoteServoPosition {
        PIVOTED,
        UNPIVOTED
    }

    //TODO: Tune claw values

    private final double OPEN_CLAW_POSITION = 0.20;
    private final double CLOSE_CLAW_POSITION = 0.0;

    private final double ROTATE_CLAW_DOWN_POSITION = 0.50;
    private final double ROTATE_CLAW_UP_POSITION = 0.0;

    private final double ARM_OUT_POSITION = 0.50;
    private final double ARM_IN_POSITION = 0.0;

    private final double PIVOTED_CLAW_POSITION = 0.40;
    private final double UNPIVOTED_CLAW_POSITION = 0.0;

    private final double CONE_INTERVAL = 0.04;

    private final Servo claw;
    private final Servo rotateServo;
    private final Servo armOne;
    private final Servo armTwo;
    private final Servo pivotServo;

    public ArmPosition armPosition = ArmPosition.IN;
    private ClawPosition clawPosition = ClawPosition.OPEN;
    private RotateServoPosition rotateServoPosition = RotateServoPosition.UP;
    private PivoteServoPosition pivoteServoPosition = PivoteServoPosition.UNPIVOTED;

    public VerticalArm (HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "horizontalClaw");
        armOne = hardwareMap.get(Servo.class, "verticalArmOne");
        armTwo = hardwareMap.get(Servo.class, "verticalArmTwo");
        rotateServo = hardwareMap.get(Servo.class, "horizontalRotateServo");
        pivotServo = hardwareMap.get(Servo.class, "horizontalPivotServo");
    }

    public void openClaw() {
        claw.setPosition(OPEN_CLAW_POSITION);
        clawPosition = ClawPosition.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(CLOSE_CLAW_POSITION);
        clawPosition = ClawPosition.CLOSED;
    }

    public void extendArm() {
        armOne.setPosition(ARM_OUT_POSITION);
        armTwo.setPosition(ARM_OUT_POSITION);
        armPosition = ArmPosition.OUT;
    }

    public void retractArm() {
        armOne.setPosition(ARM_IN_POSITION);
        armTwo.setPosition(ARM_OUT_POSITION);
        armPosition = ArmPosition.IN;
    }

    public void uprightClaw() {
        rotateServo.setPosition(ROTATE_CLAW_UP_POSITION);
        rotateServoPosition = RotateServoPosition.UP;
    }

    public void flipClaw() {
        rotateServo.setPosition(ROTATE_CLAW_DOWN_POSITION);
        rotateServoPosition = RotateServoPosition.DOWN;
    }

    public void rotateArmToConeStack(int cones) {
        armOne.setPosition(ARM_OUT_POSITION + CONE_INTERVAL * cones);
        armPosition = ArmPosition.OUT;
    }

    public void pivotClaw() {
        pivotServo.setPosition(PIVOTED_CLAW_POSITION);
        pivoteServoPosition = PivoteServoPosition.PIVOTED;
    }

    public void unpivotClaw() {
        pivotServo.setPosition(UNPIVOTED_CLAW_POSITION);
        pivoteServoPosition = PivoteServoPosition.UNPIVOTED;
    }
}
