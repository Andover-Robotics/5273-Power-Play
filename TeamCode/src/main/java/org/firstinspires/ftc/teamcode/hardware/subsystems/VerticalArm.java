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

    public enum PivotServoPosition {
        PIVOTED,
        UNPIVOTED
    }

    //TODO: Tune claw values
//0.05->0.25
    private final double OPEN_CLAW_POSITION = 0.75;
    private final double CLOSE_CLAW_POSITION = 0.5;

    private final double ROTATE_CLAW_DOWN_POSITION = 0.02;
    private final double ROTATE_CLAW_UP_POSITION = 0.6;

    private final double ARM_OUT_POSITION = 0.78;
    private final double ARM_IN_POSITION = 0.0;

    private final double PIVOTED_CLAW_POSITION = 0.0;
    private final double UNPIVOTED_CLAW_POSITION = 0.67;

    private final double CONE_INTERVAL = 0.04;

    private final Servo claw;
    private final Servo rotateServo;
    private final Servo armLeft;
    private final Servo armRight;
    private final Servo pivotServo;

    public ArmPosition armPosition = ArmPosition.IN;
    private ClawPosition clawPosition = ClawPosition.OPEN;
    private RotateServoPosition rotateServoPosition = RotateServoPosition.UP;
    private PivotServoPosition pivotServoPosition = PivotServoPosition.UNPIVOTED;

    public VerticalArm (HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "horizontalClaw");

        // 3(Left), 4(Right)
        armLeft = hardwareMap.get(Servo.class, "verticalArmLeft");
        armRight = hardwareMap.get(Servo.class, "verticalArmRight");
        rotateServo = hardwareMap.get(Servo.class, "verticalRotateServo");
        pivotServo = hardwareMap.get(Servo.class, "verticalPivotServo");

        armRight.setDirection(Servo.Direction.REVERSE);
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
        armLeft.setPosition(ARM_OUT_POSITION);
        armRight.setPosition(1-ARM_OUT_POSITION);
        armPosition = ArmPosition.OUT;
    }

    public void retractArm() {
        armLeft.setPosition(ARM_IN_POSITION);
        armRight.setPosition(1-ARM_IN_POSITION);
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

    public void pivotClaw() {
        pivotServo.setPosition(PIVOTED_CLAW_POSITION);
        pivotServoPosition = PivotServoPosition.PIVOTED;
    }

    public void unpivotClaw() {
        pivotServo.setPosition(UNPIVOTED_CLAW_POSITION);
        pivotServoPosition = PivotServoPosition.UNPIVOTED;
    }

    public void clawToScore() {
        closeClaw();
        pivotClaw();
        uprightClaw();
        extendArm();
    }
    public void clawToTransfer() {
        closeClaw();
        unpivotClaw();
        flipClaw();
        retractArm();
    }

}
