package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalArm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalLinearSlides;


@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode{

    private enum GameState {
        INTAKE,
        OUTTAKE
    }

    private double slowPercentage = 1;
    private final double MINIMUM_SPEED = 0.42; // = 0.25 / driveSpeed

    private GameState gameState = GameState.INTAKE;
    private YawPitchRollAngles yawPitchRollAngles;

    private double fieldCentricOffset;

    public void subInit() {
        driveSpeed = 1.0;

        for (MotorEx motor: bot.driveTrainMotors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RawPower);
        }

//        bot.manipulator.init();
//
//        while (bot.manipulator.horizontalLinearSlides.atTargetHeight() && bot.manipulator.horizontalLinearSlides.atTargetHeight()) {
//            bot.manipulator.horizontalLinearSlides.loop();
//            bot.manipulator.verticalLinearSlides.loop();
//        }

//        bot.manipulator.horizontalLinearSlides.setManual();
    }
    public void subLoop(){
        //============== driving and slowmode ================================================================
//        slowPercentage = (1 - DriveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * (1 - MINIMUM_SPEED) + MINIMUM_SPEED;
        yawPitchRollAngles=bot.imu1.getRobotYawPitchRollAngles();

        if (driveController.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            fieldCentricOffset=yawPitchRollAngles.getYaw(AngleUnit.RADIANS);
        }
        if (driveController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { slowPercentage = 0.5; }
        if (driveController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            telemetry.addLine("LEFT STICK PRESSED");
            bot.imu1.resetYaw();
        }

        drive();

        //============ subsystem control =============================================================================

        if (gameState == GameState.INTAKE) {
            if(subsystemController.wasJustPressed(GamepadKeys.Button.X)) {
                bot.manipulator.setToIntake();
                timingScheduler.defer(0.2, () -> bot.manipulator.horizontalArm.openClaw());
            }


        }

        else {
            if (subsystemController.isDown(GamepadKeys.Button.Y)) {
                bot.manipulator.openHorizontalClaw();
                bot.manipulator.closeVerticalClaw();
            }

            else {
                if (subsystemController.wasJustReleased(GamepadKeys.Button.Y)) {
                    bot.manipulator.prepareToOuttake();
                }

                if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.HIGH);; // high junction
                }

                else if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.GROUND); // ground junction?
                }

                else if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.LOW);
                }

                else if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.MEDIUM);
                }

                if (subsystemController.wasJustReleased(GamepadKeys.Button.A)) {
                    bot.manipulator.extendVerticalArm();
                }

                if (bot.manipulator.verticalArm.armPos == VerticalArm.ArmPos.OUTTAKE_POS) {
                    if (subsystemController.wasJustReleased(GamepadKeys.Button.A)) {
                        bot.manipulator.openVerticalClaw();
                        gameState = GameState.INTAKE;
                    }
                }
            }
        }

        if (subsystemController.wasJustReleased(GamepadKeys.Button.X)) {
            switch (gameState) {
                case INTAKE:
                    gameState = GameState.OUTTAKE;
                    break;
                case OUTTAKE:
                    gameState = GameState.INTAKE;
            }
        }


        telemetry.addData("leftStickPressed?", driveController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON));
        telemetry.addData("Drivetrain Current Draw (mA)", bot.getDriveCurrentDraw());

        telemetry.addData("imu0", bot.imu0.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//        telemetry.addData("imu1", bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);

        telemetry.addData("HorizontalLinearSlides Position", bot.manipulator.horizontalLinearSlides.curPos());
        telemetry.addData("HorizontalLinearSlides targetPosition", bot.manipulator.horizontalLinearSlides.targetPos());
        telemetry.addData("VerticalLinearSlides Position", bot.manipulator.verticalLinearSlides.getCurrentHeight());
        telemetry.addData("VerticalLinearSlides targetPosition", bot.manipulator.verticalLinearSlides.getTargetHeight());

    }
    private void drive() {
        Vector2d driveVector = new Vector2d(driveController.getLeftX()*Math.abs(driveController.getLeftX()),driveController.getLeftY()*Math.abs(driveController.getLeftY())),
                turnVector = new Vector2d(
                        driveController.getRightX()*Math.sqrt(Math.abs(driveController.getRightX())) , 0);
        if (bot.fieldCentricRunMode) {
            bot.drive(
                    driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed ,
                    turnVector.getX() * driveSpeed ,
                    yawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            );
        } else {
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        }
    }

}
