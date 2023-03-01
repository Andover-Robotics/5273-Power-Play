package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;


public class HorizontalArm {

    public enum ClawPosition { // 1 servo
        OPEN,
        CLOSED
    }

    public enum ArmPosition { // 2 servos
        IN,
        OUT
    }

    public enum RotateServoPosition { // 2 servos
        UP,
        DOWN
    }

    public enum PivoteServoPosition { // 1 servo
        PIVOTED,
        UNPIVOTED
    }

    //TODO: Tune claw values

    private final double OPEN_CLAW_POSITION = 0.70;
    private final double CLOSE_CLAW_POSITION = 0.95;

    private final double ROTATE_CLAW_DOWN_POSITION_RIGHT = 0.0;
    private final double ROTATE_CLAW_UP_POSITION_RIGHT = 0.32;

    private static final double ROTATE_CLAW_DOWN_POSITION_LEFT = 0.55;
    private static final double ROTATE_CLAW_UP_POSITION_LEFT = 0.70;

    private final double ARM_OUT_POSITION = 0.0;
    private final double ARM_IN_POSITION = 0.70;

    private final double PIVOTED_CLAW_POSITION = 0.70;
    private final double UNPIVOTED_CLAW_POSITION = 0.04;

    private final double CONE_INTERVAL = 0.04;

    private final Servo claw;
    private final Servo rotateServoLeft;
    private final Servo rotateServoRight;
    private final Servo armRight;
    private final Servo armLeft;
    private final Servo pivotServo;

    public ArmPosition armPosition = ArmPosition.IN;
    public ClawPosition clawPosition = ClawPosition.OPEN;
    public RotateServoPosition rotateServoPosition = RotateServoPosition.UP;
    public PivoteServoPosition pivoteServoPosition = PivoteServoPosition.UNPIVOTED;

    public HorizontalArm (HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "horizontalClaw");
        armRight = hardwareMap.get(Servo.class, "horizontalArmRight");
        armLeft = hardwareMap.get(Servo.class, "horizontalArmLeft");
        rotateServoLeft = hardwareMap.get(Servo.class, "horizontalRotateLeft");
        rotateServoRight = hardwareMap.get(Servo.class, "horizontalRotateRight");
        pivotServo = hardwareMap.get(Servo.class, "horizontalPivotServo");
    }

    public double getCloseClawPosition() { return CLOSE_CLAW_POSITION; }

    public double getOpenClawPosition() { return OPEN_CLAW_POSITION; }

    public double getClawPosition() { return claw.getPosition(); }

    public double getRotateServoPosition() { return rotateServoLeft.getPosition(); }

    public double getArmPosition() { return armRight.getPosition(); }

    public void openClaw() {
        claw.setPosition(OPEN_CLAW_POSITION);
        clawPosition = ClawPosition.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(CLOSE_CLAW_POSITION);
        clawPosition = ClawPosition.CLOSED;
    }

    public void extendArm() {
        armRight.setPosition(ARM_OUT_POSITION);
        armLeft.setPosition(ARM_OUT_POSITION);
        armPosition = ArmPosition.OUT;
    }

    public void retractArm() {
        armRight.setPosition(ARM_IN_POSITION);
        armLeft.setPosition(ARM_OUT_POSITION);
        armPosition = ArmPosition.IN;
    }

    public void uprightClaw() {
        rotateServoLeft.setPosition(ROTATE_CLAW_UP_POSITION_LEFT);
        rotateServoRight.setPosition(ROTATE_CLAW_UP_POSITION_RIGHT);
        rotateServoPosition = RotateServoPosition.UP;
    }

    public void flipClaw() {
        rotateServoLeft.setPosition(ROTATE_CLAW_DOWN_POSITION_LEFT);
        rotateServoRight.setPosition(ROTATE_CLAW_DOWN_POSITION_RIGHT);
        rotateServoPosition = RotateServoPosition.DOWN;
    }

    public void rotateArmToConeStack(int cones) {
        armRight.setPosition(ARM_OUT_POSITION + CONE_INTERVAL * cones);
        armLeft.setPosition(ARM_OUT_POSITION + CONE_INTERVAL * cones);
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
