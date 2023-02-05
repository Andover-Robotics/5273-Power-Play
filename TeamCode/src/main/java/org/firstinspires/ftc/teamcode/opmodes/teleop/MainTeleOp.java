package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LinearSlides;

import java.util.Map;



@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode{

    private double slowPercentage = 1;
    private final double MINIMUM_SPEED = 0.42; // = 0.25 / driveSpeed
    public double fieldCentricOffset=0;

    public void subInit() {
        driveSpeed = 0.6;
        for (MotorEx motor: bot.driveTrainMotors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RawPower);
        }
        bot.outtake.linearSlides.initializeSlideMotor(bot.outtake.linearSlides.slideMotor);
        bot.initializeImu(bot.imu0);
        bot.initializeImu(bot.imu1);

    }
    public void subLoop(){

        //============== driving and slowmode ================================================================
        slowPercentage = (1 - driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * (1 - MINIMUM_SPEED) + MINIMUM_SPEED;
        if (driveController.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            driveSpeed = 1.0;
        } else {
            driveSpeed = 0.6;
        }
        if (driveController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            slowPercentage = 0.5;
        }
        if (driveController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            fieldCentricOffset = (bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle+bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle)/2;
        }
        drive();

        //============ subsystem control =============================================================================
        if(subsystemController.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
            bot.outtake.linearSlides.decrementLevel();
        }
        if(subsystemController.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
            bot.outtake.linearSlides.incrementLevel();
        }

        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bot.outtake.linearSlides.hover();
        }
        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.outtake.linearSlides.retract();
        }

        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bot.outtake.claw.openClaw();
        }
        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bot.outtake.claw.closeClaw();
        }


        if (subsystemController.wasJustPressed(GamepadKeys.Button.A)) {
            bot.outtake.linearSlides.setLevel(LinearSlides.Level.GROUND);
        }
        if (subsystemController.wasJustPressed((GamepadKeys.Button.X))) {
            bot.outtake.linearSlides.setLevel(LinearSlides.Level.LOW);
        }
        if (subsystemController.wasJustPressed(GamepadKeys.Button.B)) {
            bot.outtake.linearSlides.setLevel(LinearSlides.Level.MEDIUM);
        }
        if (subsystemController.wasJustPressed((GamepadKeys.Button.Y))) {
            bot.outtake.linearSlides.setLevel(LinearSlides.Level.HIGH);
        }
        if(Math.abs(subsystemController.getLeftY())>0.05){
            bot.outtake.linearSlides.extend((int)(bot.outtake.linearSlides.getTargetHeight()+10*subsystemController.getLeftY()));
        }
        bot.outtake.linearSlides.loop();

        telemetry.addData("imu0", bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
        telemetry.addData("imu1", bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
        telemetry.addData("slideMotor Position", bot.outtake.linearSlides.getCurrentHeight());
        telemetry.addData("slideMotor targetPosition", bot.outtake.linearSlides.getTargetHeight());
    }
    private void drive() {
        Vector2d driveVector = new Vector2d(driveController.getLeftX(), driveController.getLeftY()),
                turnVector = new Vector2d(
                        driveController.getRightX() , 0);
        if (bot.fieldCentricRunMode) {
            bot.drive(
                    driveVector.getX() * driveSpeed * slowPercentage,
                    driveVector.getY() * driveSpeed * slowPercentage,
                    turnVector.getX() * driveSpeed * slowPercentage,
                    (bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle) / 2
                            - fieldCentricOffset
            );
        } else {
            bot.drive(driveVector.getX() * driveSpeed * slowPercentage,
                    driveVector.getY() * driveSpeed * slowPercentage,
                    turnVector.getX() * driveSpeed * slowPercentage
            );
        }
    }

}
