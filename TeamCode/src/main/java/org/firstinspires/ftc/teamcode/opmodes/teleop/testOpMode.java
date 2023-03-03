package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalLinearSlides;

@TeleOp(name="testing", group="Testing")
public class testOpMode extends BaseOpMode{


    @Override
    protected void subInit() {
        bot.resetJavaGCCleanedThingsSoSad();
        CommandScheduler.getInstance().enable();
        bot.manipulator.horizontalArm.setArmTransfer();
        bot.manipulator.verticalArm.setArmTransfer();
        telemetry.addLine("Init done.");
    }

    @Override
    public void start() {


    }

    @Override
    protected void subLoop() {
        if(subsystemController.wasJustPressed(GamepadKeys.Button.X)) {
            bot.manipulator.horizontalArm.setIntake();
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.Y)) {
            bot.manipulator.horizontalArm.setTransfer();
        }

        if(subsystemController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            bot.manipulator.horizontalArm.openClaw();
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bot.manipulator.horizontalArm.closeClaw();
        }

        if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bot.manipulator.verticalArm.setOuttake();
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.manipulator.verticalArm.setTransfer();
        }
        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.LOW);
        }
        bot.manipulator.verticalLinearSlides.loop();

    }
}
