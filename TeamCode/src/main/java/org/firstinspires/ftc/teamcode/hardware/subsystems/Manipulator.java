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

    //TODO: implement conjoined methods

    public void intake(){
        horizontalArm.closeClaw();
        horizontalArm.flipClaw();
        horizontalLinearSlides.retract();
        horizontalArm.retractArm();
    }

    public void outtake() {
        verticalArm.closeClaw();
        verticalLinearSlides.extend();
        verticalArm.extendArm();
        verticalArm.openClaw();
    }

    public void reset() {
        verticalArm.retractArm();
        horizontalArm.retractArm();
        horizontalArm.uprightClaw();
        verticalLinearSlides.retract();
        horizontalLinearSlides.retract();
    }

}
