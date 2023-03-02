package org.firstinspires.ftc.teamcode.util.toolbox;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "TBX: Servo Position Tuner", group = "ARC Toolbox")
public class ServoPositionTuner extends OpMode {

    double currentPosition;
    int currentServoIndex=0;

    Servo currentServo;
    List<Servo> servos;
    private GamepadEx gamepadEx1;
    @Override
    public void init() {
        servos=this.hardwareMap.getAll(Servo.class);
        currentServo=servos.get(currentServoIndex);
        currentPosition=currentServo.getPosition();
        gamepadEx1= new GamepadEx(gamepad1);

    }

    @Override
    public void loop() {
        gamepadEx1.readButtons();


        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)){
            currentServoIndex=(currentServoIndex+1)%servos.size();
        }
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)){
            currentServoIndex=(currentServoIndex+10)%servos.size();
        }

        currentServo=servos.get(currentServoIndex);
        currentPosition=currentServo.getPosition();

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
            currentPosition=(currentPosition+0.10)%1.0;
        }

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
            currentPosition=(currentPosition+0.01)%1.0;
        }
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)){
            currentPosition=(currentPosition+0.99)%1.0;
        }


        currentServo.setPosition(currentPosition);

        telemetry.addData("currentServoName",currentServo.getConnectionInfo());

        for(Servo servo: servos){
            telemetry.addLine(servo.getConnectionInfo()+" "+servo.getPosition());
        }
        telemetry.update();
    }
}
