package org.firstinspires.ftc.teamcode.util.toolbox;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opmodes.teleop.BaseOpMode;

import java.util.Map;
import java.util.stream.Stream;

@TeleOp(name = "TBX: Slides Tuner", group = "ARC Toolbox")
public class LinearSlidePIDTuner extends OpMode {


    private InputColumnResponder input = new InputColumnResponderImpl();
    private GamepadEx gamepadEx1;


    private MotorEx slideMotor;

    private int targetPositions[] = {0, 1700, 2800, 4000}; // Change for your specific use case

    private double ppsvCoefficients[] = {0.5, 0.05, 0.002, 0.01};
    private double deltaCoefficients[] = {0.01, 0.01, 0.001, 0.001};

    private int currentPIDfIndex = 0;

    @Override
    public void init() {
        gamepadEx1= new GamepadEx(gamepad1);
    }

    @Override
    public void start() {
        slideMotor = new MotorEx(hardwareMap, "rightSlideMotor", Motor.GoBILDA.RPM_312);//Change name of motor

        input.clearRegistry();

        slideMotor.setTargetPosition(0);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPositionTolerance(50);
    }

    int target = 0;

    @Override
    public void loop() {
        gamepadEx1.readButtons();

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.X)) {
            slideMotor.setTargetPosition(targetPositions[0]);
            target = targetPositions[0];
        }

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
            slideMotor.setTargetPosition(targetPositions[1]);
            target = targetPositions[1];
        }

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
            slideMotor.setTargetPosition(targetPositions[2]);
            target = targetPositions[2];
        }

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.Y)) {
            slideMotor.setTargetPosition(targetPositions[3]);
            target = targetPositions[3];
        }

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            currentPIDfIndex = (currentPIDfIndex + 3) % 4;

        }

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            currentPIDfIndex  =(currentPIDfIndex + 1) % 4;
        }

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            ppsvCoefficients[currentPIDfIndex] += deltaCoefficients[currentPIDfIndex];
        }

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            ppsvCoefficients[currentPIDfIndex] -= deltaCoefficients[currentPIDfIndex];
        }


        if(target>slideMotor.getCurrentPosition()){
            slideMotor.setPositionCoefficient(ppsvCoefficients[0]);
        }

        else{
            slideMotor.setPositionCoefficient(ppsvCoefficients[1]);
        }

        if (slideMotor.atTargetPosition()){
            slideMotor.set(ppsvCoefficients[2]);
        }

        else{
            slideMotor.set(ppsvCoefficients[3]);
        }

        telemetry.addData("kP UP", ppsvCoefficients[0]);
        telemetry.addData("kp DOWN", ppsvCoefficients[1]);
        telemetry.addData("kS", ppsvCoefficients[2]);
        telemetry.addData("kV", ppsvCoefficients[3]);
        telemetry.addData("targetPosition", target);
        telemetry.addData("currentPosition", slideMotor.getCurrentPosition());
        telemetry.addData("velocity", 	slideMotor.getVelocity());
        telemetry.addData("currentdraw", slideMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
