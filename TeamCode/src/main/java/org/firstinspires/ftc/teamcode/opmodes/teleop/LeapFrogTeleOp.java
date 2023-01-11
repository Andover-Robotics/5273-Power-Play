package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LeapFrog TeleOp", group = "Competition")
public class LeapFrogTeleOp extends BaseOpMode{

    public void subInit() {
        driveSpeed = 1.0;
    }
    public void subLoop(){
        drive();
    }
    private void drive() {
        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() , 0);
        bot.drive(
                -driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                -turnVector.getX() * driveSpeed
        );
    }

}
