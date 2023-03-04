package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.utilclasses.TimingScheduler;

public class Manipulator extends SubsystemBase {

    public final VerticalLinearSlides verticalLinearSlides;
    public final HorizontalLinearSlides horizontalLinearSlides;

    public final VerticalArm verticalArm;
    public final HorizontalArm horizontalArm;

    public Manipulator(HardwareMap hardwareMap) {
        verticalLinearSlides = new VerticalLinearSlides(hardwareMap);
        horizontalLinearSlides = new HorizontalLinearSlides(hardwareMap);
        verticalArm = new VerticalArm(hardwareMap);
        horizontalArm = new HorizontalArm(hardwareMap);
    }

    public void setToIntake() {
        horizontalLinearSlides.extendSlides();
        horizontalArm.setArmIntake();
    }

    public void setToTransfer() {
        horizontalArm.closeClaw();

    }





    public void prepareToOuttake() {
        verticalArm.closeClaw();
        verticalArm.setHingeTransfer();
        verticalArm.setPivotOuttake();
    }

    public void extendVerticalArm() {
        verticalArm.setArmOuttake();
    }

    public void openHorizontalClaw() {
        horizontalArm.openClaw();
    }

    public void closeVerticalClaw() {
        verticalArm.closeClaw();
    }

    public void openVerticalClaw() {
        verticalArm.openClaw();
    }




    public void init() {
//        verticalArm.unpivotClaw();
//        verticalArm.openClaw();
//        verticalArm.retractArm();
//        verticalArm.uprightClaw();
//
//        horizontalArm.unpivotClaw();
//        horizontalArm.openClaw();
//        horizontalArm.retractArm();
//        horizontalArm.uprightClaw();
//
//        verticalLinearSlides.retract();
//
//        horizontalLinearSlides.setPreset();
//        horizontalLinearSlides.retract();
    }

    @Override
    public void periodic() {
        if(horizontalLinearSlides.atTargetHeight()){
            horizontalArm.setIdle();
        }
    }
}
