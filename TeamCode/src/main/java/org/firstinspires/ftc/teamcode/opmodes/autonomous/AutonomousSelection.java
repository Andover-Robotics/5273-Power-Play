package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.GlobalConfig.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@TeleOp(name = "Autonomous Selection", group = "Competition")
public class AutonomousSelection extends OpMode {

    GamepadEx gamepadEx1;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) { autonomousType = (autonomousType == GlobalConfig.AutonomousType.PARKING) ? GlobalConfig.AutonomousType.STACK : GlobalConfig.AutonomousType.PARKING; }
        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) { side = (side == GlobalConfig.Side.AUDIENCE) ? GlobalConfig.Side.STAGE : GlobalConfig.Side.AUDIENCE; }
        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) { alliance = (alliance == GlobalConfig.Alliance.RED) ? GlobalConfig.Alliance.BLUE : GlobalConfig.Alliance.RED; }
    }
}
