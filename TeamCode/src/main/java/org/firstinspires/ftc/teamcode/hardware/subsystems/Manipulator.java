package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Manipulator extends SubsystemBase {
    public final LinearSlides linearSlides;
    public final Claw claw;
    private static final int CLAW_CLEARANCE_HEIGHT = 450;

    public Manipulator(HardwareMap hardwareMap) {
        linearSlides = new LinearSlides(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    //TODO: implement conjoined methods

    public void runOuttake() {
        linearSlides.extend();
        if (linearSlides.getCurrentHeight() >= CLAW_CLEARANCE_HEIGHT) {
            claw.raiseRotateClaw();
        }
    }

}
