package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    //TODO: modify methods to reflect robot

    public void intake(){
        horizontalArm.closeClaw();
        horizontalArm.flipClaw();
        horizontalArm.pivotClaw();
        horizontalLinearSlides.retract();
        horizontalArm.retractArm();
    }

    public void prepareToOuttake() {
        verticalArm.closeClaw();
        verticalArm.flipClaw();
        verticalArm.pivotClaw();
    }

    public void extendVerticalArm() {
        verticalArm.extendArm();
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

    public void resetVertical() {
        verticalArm.retractArm();
        verticalArm.uprightClaw();
        verticalArm.unpivotClaw();
        verticalArm.openClaw();
        verticalLinearSlides.retract();
    }

    public void resetHorizontal() {
        horizontalArm.retractArm();
        horizontalArm.uprightClaw();
        horizontalArm.unpivotClaw();
        horizontalArm.openClaw();
        horizontalLinearSlides.retract();
    }
    
    public void init() {
        verticalArm.unpivotClaw();
        verticalArm.openClaw();
        verticalArm.retractArm();
        verticalArm.uprightClaw();

        horizontalArm.unpivotClaw();
        horizontalArm.openClaw();
        horizontalArm.retractArm();
        horizontalArm.uprightClaw();

        verticalLinearSlides.retract();

        horizontalLinearSlides.setPreset();
        horizontalLinearSlides.retract();
    }
}
