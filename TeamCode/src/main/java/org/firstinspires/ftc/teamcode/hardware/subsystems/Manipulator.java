package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Manipulator extends SubsystemBase {
    public final ScrollLinearSlides linearSlides;
    public final Claw claw;

    public Manipulator(HardwareMap hardwareMap) {
        linearSlides = new ScrollLinearSlides(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    //TODO: implement conjoined methods

    @Override
    public void periodic() {
        //linearSlides.periodic();
    }
     public void resetEncoders(){
         linearSlides.resetEncoders();
    }

}
