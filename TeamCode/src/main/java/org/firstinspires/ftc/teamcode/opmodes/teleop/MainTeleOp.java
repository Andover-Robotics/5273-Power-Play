package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalArm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalLinearSlides;
import org.firstinspires.ftc.teamcode.util.utilclasses.TimingScheduler;


@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode{

    private enum GameState {
        INTAKE,
        OUTTAKE
    }

    private double slowPercentage = 1;
    private final double MINIMUM_SPEED = 0.42; // = 0.25 / driveSpeed

    private GameState gameState = GameState.INTAKE;

    public void subInit() {
        driveSpeed = 1.0;

        for (MotorEx motor: bot.driveTrainMotors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RawPower);
        }

        bot.imu0.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        bot.manipulator.init();

        while (bot.manipulator.horizontalLinearSlides.atTargetHeight() && bot.manipulator.horizontalLinearSlides.atTargetHeight()) {
            bot.manipulator.horizontalLinearSlides.loop();
            bot.manipulator.verticalLinearSlides.loop();
        }

        bot.manipulator.horizontalLinearSlides.setManual();
    }
    public void subLoop(){
        //============== driving and slowmode ================================================================
        slowPercentage = (1 - SubsystemController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * (1 - MINIMUM_SPEED) + MINIMUM_SPEED;

        if (SubsystemController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { slowPercentage = 0.5; }

        if (SubsystemController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            telemetry.addLine("LEFT STICK PRESSED");
            bot.imu0.resetYaw();
        }

        drive();

        //============ subsystem control =============================================================================

        if (gameState == GameState.INTAKE) {
            if (Math.abs(SubsystemController.getLeftY()) > 0.05){
                bot.manipulator.horizontalLinearSlides.runManual(DriveController.getLeftY());
            }

            if (SubsystemController.wasJustPressed(GamepadKeys.Button.X)) {
                bot.manipulator.horizontalLinearSlides.setRunUsingDistanceSensor();
                bot.manipulator.prepareToIntake();
            }

            if (SubsystemController.isDown(GamepadKeys.Button.A)) {
                bot.manipulator.openHorizontalClaw();
            }

            else if (SubsystemController.wasJustReleased(GamepadKeys.Button.A)) {
                bot.manipulator.intake();
                gameState = GameState.OUTTAKE;
            }
        }

        else {
            if (SubsystemController.isDown(GamepadKeys.Button.Y)) {
                bot.manipulator.openHorizontalClaw();
                bot.manipulator.closeVerticalClaw();
            }

            else {
                if (SubsystemController.wasJustReleased(GamepadKeys.Button.Y)) {
                    bot.manipulator.prepareToOuttake();
                }

                if (SubsystemController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.manipulator.verticalLinearSlides.extend();
                }

                else if (SubsystemController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.manipulator.verticalLinearSlides.retract();
                }

                else if (SubsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.LOW);
                }

                else if (SubsystemController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.MEDIUM);
                }

                if (SubsystemController.wasJustReleased(GamepadKeys.Button.A)) {
                    bot.manipulator.extendVerticalArm();
                }

                if (bot.manipulator.verticalArm.armPosition == VerticalArm.ArmPosition.OUT) {
                    if (SubsystemController.wasJustReleased(GamepadKeys.Button.A)) {
                        bot.manipulator.openVerticalClaw();
                        gameState = GameState.INTAKE;
                        timingScheduler.defer(0.5, () -> bot.manipulator.resetVertical());
                    }
                }
            }
        }

        if (SubsystemController.wasJustReleased(GamepadKeys.Button.X)) {
            switch (gameState) {
                case INTAKE:
                    gameState = GameState.OUTTAKE;
                    break;
                case OUTTAKE:
                    gameState = GameState.INTAKE;
            }
        }

        bot.manipulator.verticalLinearSlides.loop();
        bot.manipulator.horizontalLinearSlides.loop();

        telemetry.addData("leftStickPressed?", SubsystemController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON));
        telemetry.addData("Drivetrain Current Draw (mA)", bot.getDriveCurrentDraw());

        telemetry.addData("imu0", bot.imu0.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//        telemetry.addData("imu1", bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);

        telemetry.addData("HorizontalLinearSlides Position", bot.manipulator.horizontalLinearSlides.getCurrentHeight());
        telemetry.addData("HorizontalLinearSlides targetPosition", bot.manipulator.horizontalLinearSlides.getTargetHeight());
        telemetry.addData("VerticalLinearSlides Position", bot.manipulator.verticalLinearSlides.getCurrentHeight());
        telemetry.addData("VerticalLinearSlides targetPosition", bot.manipulator.verticalLinearSlides.getTargetHeight());

    }
    private void drive() {
        Vector2d driveVector = new Vector2d(SubsystemController.getLeftX(), SubsystemController.getLeftY()),
                turnVector = new Vector2d(
                        SubsystemController.getRightX() , 0);
        if (bot.fieldCentricRunMode) {
            bot.drive(
                    driveVector.getX() * driveSpeed * slowPercentage,
                    driveVector.getY() * driveSpeed * slowPercentage,
                    turnVector.getX() * driveSpeed * slowPercentage,
                    bot.imu0.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
            );
        } else {
            bot.drive(driveVector.getX() * driveSpeed * slowPercentage,
                    driveVector.getY() * driveSpeed * slowPercentage,
                    turnVector.getX() * driveSpeed * slowPercentage
            );
        }
    }

}
