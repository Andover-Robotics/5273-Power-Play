package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalLinearSlides;

@TeleOp(name = "StatesTeleOp", group = "Competition")
public class StatesTeleOp extends BaseOpMode{
    private YawPitchRollAngles yawPitchRollAngles;


    @Override
    protected void subInit() {
        driveSpeed = 1.0;
        for (MotorEx motor: bot.driveTrainMotors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RawPower);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
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
        //drive
        yawPitchRollAngles = bot.imu1.getRobotYawPitchRollAngles();

        if (driveController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bot.imu1.resetYaw();
        }
        if(driveController.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
            bot.manipulator.horizontalArm.openClaw();
        }
        if(driveController.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
            bot.manipulator.horizontalArm.closeClaw();
        }
        drive();
        //subsystem

        if(subsystemController.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)){
            bot.manipulator.verticalLinearSlides.hover();
        }
        else if(subsystemController.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)){
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.MEDIUM);
        }
        else if(subsystemController.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.HIGH);
        }


        if(subsystemController.wasJustReleased(GamepadKeys.Button.A)){  //retract horizontal slides and setIdle
            bot.manipulator.horizontalLinearSlides.retractSlides();
            bot.manipulator.horizontalArm.setArmTransfer();

        }
        else if(subsystemController.wasJustReleased(GamepadKeys.Button.Y)){ //extend horizontal slides and setIntake
            bot.manipulator.horizontalLinearSlides.extendSlides();
            bot.manipulator.horizontalArm.setIntake();
        }
        //TODO add other controls and add automated inntake and outtake

        CommandScheduler.getInstance().run();



        telemetry.addData("Horizontal Current Position", bot.manipulator.horizontalLinearSlides.curPos());
        telemetry.addData("Horizontal Target Position", bot.manipulator.horizontalLinearSlides.targetPos());
        telemetry.addData("Horizontal getDist()", bot.manipulator.horizontalLinearSlides.getDist());
        telemetry.addData("Vertical currentHeight", bot.manipulator.verticalLinearSlides.getCurrentHeight());
        telemetry.addData("Vertical targetHeight", bot.manipulator.verticalLinearSlides.getTargetHeight());

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
