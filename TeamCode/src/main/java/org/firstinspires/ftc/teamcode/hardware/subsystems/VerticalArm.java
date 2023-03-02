package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;


public class VerticalArm extends SubsystemBase {

    //TODO: find init servo pos values
    // make transfer mech actually work
    public enum ClawPos {
        INIT_POS(0.5),
        OPEN_POS(0.75),
        CLOSE_POS(0.5);

        private double pos;
        ClawPos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    public enum ArmPos {
        INIT_POS(0.0),
        TRANSFER_POS(0.0),
        OUTTAKE_POS(0.78);

        private double pos;
        ArmPos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    public enum HingePos {
        INIT_POS(0.02),
        OUTTAKE_POS(0.6),
        TRANSFER_POS(0.02);

        private double pos;
        HingePos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    public enum PivotPos {
        INIT_POS(0.67),
        OUTTAKE_POS(0.0),
        TRANSFER_POSS(0.67);

        private double pos;
        PivotPos(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }

    private final double CONE_INTERVAL = 0.04;

    private final Servo claw;
    private final Servo hinge;
    private final Servo armL;
    private final Servo armR;
    private final Servo pivot;

    public ArmPos armPos = ArmPos.INIT_POS;
    private ClawPos clawPos = ClawPos.INIT_POS;
    private HingePos hingePos = HingePos.INIT_POS;
    private PivotPos pivotPos = PivotPos.INIT_POS;

    public VerticalArm (HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "horizontalClaw");
        armL = hardwareMap.get(Servo.class, "verticalArmLeft");
        armR = hardwareMap.get(Servo.class, "verticalArmRight");
        armR.setDirection(Servo.Direction.REVERSE);
        hinge = hardwareMap.get(Servo.class, "verticalRotateServo");
        pivot = hardwareMap.get(Servo.class, "verticalPivotServo");
    }

    public void openClaw() { clawPos = ClawPos.OPEN_POS; }

    public void closeClaw() { clawPos = ClawPos.CLOSE_POS; }

    public void setArmOuttake() { armPos = ArmPos.OUTTAKE_POS; }

    public void setArmTransfer() { armPos = ArmPos.TRANSFER_POS; }

    public void setHingeOuttake() { hingePos = HingePos.OUTTAKE_POS; }

    public void setHingeTransfer() { hingePos = HingePos.TRANSFER_POS; }

    public void setPivotOuttake() { pivotPos = PivotPos.OUTTAKE_POS; }

    public void setPivotTransfer() { pivotPos = PivotPos.TRANSFER_POSS; }

    public void clawToScore() {
        closeClaw();
        setPivotOuttake();
        setHingeOuttake();
        setArmOuttake();
    }
    public void clawToTransfer() {
        closeClaw();
        setPivotTransfer();
        setHingeTransfer();
        setArmTransfer();
    }

    public void setServoPoses() {
        claw.setPosition(clawPos.getPos());
        pivot.setPosition(pivotPos.getPos());
        hinge.setPosition(hingePos.getPos());
        armL.setPosition(armPos.getPos());
        armR.setPosition(armPos.getPos());
    }

}
