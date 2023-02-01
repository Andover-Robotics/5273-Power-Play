package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Map;



@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode{

    private double slowPercentage = 1;
    private final double MINIMUM_SPEED = 0.25;
    public double fieldCentricOffset=0;

    public void subInit() {
        driveSpeed = 1.0;
        for (MotorEx motor: bot.driveTrainMotors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RawPower);
        }
        bot.outtake.linearSlides.initializeSlideMotor(bot.outtake.linearSlides.slideMotor);
        bot.initializeImu(bot.imu0);
        bot.initializeImu(bot.imu1);

    }
    public void subLoop(){
        slowPercentage = (1 - gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * (1 - MINIMUM_SPEED) + MINIMUM_SPEED;
        drive();

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
            bot.outtake.linearSlides.decrementLevel();
        }

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
            bot.outtake.linearSlides.incrementLevel();
        }

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bot.outtake.linearSlides.extend();
        }

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.outtake.linearSlides.retract();
        }

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
            bot.outtake.claw.openClaw();
        }

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            bot.outtake.claw.closeClaw();
        }

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            fieldCentricOffset = (bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle+bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle)/2;
        }

        telemetry.addData("imu0", bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
        telemetry.addData("imu1", bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
        telemetry.addData("slideMotor Position", bot.outtake.linearSlides.getCurrentHeight());
        telemetry.addData("slideMotor targetPosition", bot.outtake.linearSlides.getTargetHeight());
        telemetry.addData("slideMotor RunMode", bot.outtake.linearSlides.slideMotor.getMode());
    }
    private void drive() {
        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() , 0);
        bot.drive(
                driveVector.getX() * driveSpeed * slowPercentage,
                driveVector.getY() * driveSpeed * slowPercentage,
                turnVector.getX() * driveSpeed * slowPercentage,
                (bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle+bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle)/2
                - fieldCentricOffset
        );
    }

}
