package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

//0.00 0.60
//0.50 0.75


public class HorizontalArm extends SubsystemBase {

    //TODO: find INIT values & cone stack values then implement functions for those
    // additionally do cone righting if there is time

    public enum ClawPos {
        OPEN(0.70),
        CLOSED(0.95);

        private double pos;
        ClawPos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    public enum ArmPos { //left vals
        INIT_POS(0.0),
        TRANSFER_POS(0.55),
        INTAKE_POS(0.15);
        private double pos;
        ArmPos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    public enum HingePos { // left vals
        INIT_POS(0.0),
        TRANSFER_POS(0.9),
        IDLE_POS(0.6),
        SCORE_POS(0.7),
        INTAKE_POS(0.63);

        private double pos;
        HingePos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    public enum PivotPos {
        INIT_POS(0.0),
        TRANSFER_POS(0.7),
        INTAKE_POS(0.03);

        private double pos;
        PivotPos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    private final double CONE_ARM_ANGLE_OFFSET = 0.04;
    //use this by adding to the value of the arm and hinge to emulate 4bar behavior and add height of cones x this offset for that offset value


    private final Servo claw;
    private final Servo hingeServoL;
    private final Servo hingeServoR;
    private final Servo armServoR;
    private final Servo armServoL;
    private final Servo pivotServo;

    public ArmPos armPos = ArmPos.INIT_POS;
    public ClawPos clawPos = ClawPos.CLOSED;
    public HingePos hingePos = HingePos.INIT_POS;
    public PivotPos pivotPos = PivotPos.INIT_POS;

    public HorizontalArm(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "horizontalClaw");
        armServoL = hardwareMap.get(Servo.class, "horizontalArmLeft");
        armServoR = hardwareMap.get(Servo.class, "horizontalArmRight");
        armServoR.setDirection(Servo.Direction.REVERSE);
        hingeServoL = hardwareMap.get(Servo.class, "horizontalRotateLeft");
        hingeServoR = hardwareMap.get(Servo.class, "horizontalRotateRight");
        hingeServoR.setDirection(Servo.Direction.REVERSE);
        pivotServo = hardwareMap.get(Servo.class, "horizontalPivotServo");
    }

    public void resetJavaGCBaddies() {
        armServoR.setDirection(Servo.Direction.REVERSE);
        hingeServoR.setDirection(Servo.Direction.REVERSE);
    }

    public double getClawPosition() { return claw.getPosition(); }
    public double getRotateServoPosition() { return hingeServoL.getPosition(); }
    public double getArmPosition() { return armServoR.getPosition(); }

    public void openClaw() { clawPos = ClawPos.OPEN; setServoPoses();}
    public void closeClaw() { clawPos = ClawPos.CLOSED; setServoPoses();}

    public void setArmIntake() { armPos = ArmPos.INTAKE_POS; }
    public void setArmTransfer() { armPos = ArmPos.TRANSFER_POS; }

    public void setHingeIntake() { hingePos = HingePos.INTAKE_POS; }
    public void setHingeIdle() { hingePos = HingePos.IDLE_POS; }
    public void setHingeScore() { hingePos = HingePos.SCORE_POS; }
    public void setHingeTransfer() { hingePos = HingePos.TRANSFER_POS; }

    public void setPivotIntake() { pivotPos = PivotPos.INTAKE_POS; }
    public void setPivotTransfer() { pivotPos = PivotPos.TRANSFER_POS; }

    public void setIntake() {
        setHingeIntake();
        setArmIntake();
        setPivotIntake();
        setServoPoses();
    }

    public void setTransfer() {
        setHingeTransfer();
        setArmTransfer();
        setPivotTransfer();
        setServoPoses();
    }

    public void setIdle() {
        setHingeIdle();
        setArmTransfer();
        setPivotIntake();
        setServoPoses();
    }

    public void setReadyToScoreGround() {
        setHingeScore();
        setArmIntake();
        setPivotIntake();
        setServoPoses();
    }

    public void setScoreGround() {
        setHingeIntake();
        setArmIntake();
        setPivotIntake();
        setServoPoses();
    }

    public void setInitPoses() {
        claw.setPosition(ClawPos.CLOSED.getPos());
        pivotServo.setPosition(PivotPos.INIT_POS.getPos());
        hingeServoL.setPosition(HingePos.INIT_POS.getPos());
        hingeServoR.setPosition(HingePos.INIT_POS.getPos());
        armServoL.setPosition(ArmPos.INIT_POS.getPos());
        armServoR.setPosition(ArmPos.INIT_POS.getPos());
    }

    public void setServoPoses() {
        claw.setPosition(clawPos.getPos());
        pivotServo.setPosition(pivotPos.getPos());
        hingeServoL.setPosition(hingePos.getPos());
        hingeServoR.setPosition(hingePos.getPos());
        armServoL.setPosition(armPos.getPos());
        armServoR.setPosition(armPos.getPos());
    }
}
