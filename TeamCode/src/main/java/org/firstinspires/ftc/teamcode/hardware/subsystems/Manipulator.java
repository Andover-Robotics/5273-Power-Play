package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Manipulator extends SubsystemBase {
    public final LinearSlides linearSlides;
    public final Claw claw;

    public Manipulator(HardwareMap hardwareMap) {
        linearSlides = new LinearSlides(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    //TODO: implement conjoined methods

    @Override
    public void periodic() {
        linearSlides.periodic();
    }

}
