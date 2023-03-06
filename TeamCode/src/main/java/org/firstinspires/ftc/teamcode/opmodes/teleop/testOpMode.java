package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalLinearSlides;

@TeleOp(name="testing", group="Testing")
public class testOpMode extends BaseOpMode{

    private YawPitchRollAngles yawPitchRollAngles;


    @Override
    protected void subInit() {
        driveSpeed = 1.0;
        for (MotorEx motor: bot.driveTrainMotors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RawPower);
        }
        bot.manipulator.verticalLinearSlides.resetSlideEncoders();

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

        yawPitchRollAngles=bot.imu1.getRobotYawPitchRollAngles();

        if (driveController.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            bot.imu1.resetYaw();
        }
//        if (Math.abs(driveController.getLeftX())>0.05 || (Math.abs(driveController.getLeftY())>0.05)) {
//            bot.manipulator.horizontalLinearSlides.retractSlides();
//        }

//        if (bot.manipulator.horizontalLinearSlides.targetPos() == 0) {
//            bot.manipulator.horizontalLinearSlides.retractSlides();
//        }

        drive();


        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.HIGH);; // high junction
        }

        else if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.LOW);
        }

        else if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.MEDIUM);
        }
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

        if(subsystemController.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bot.manipulator.horizontalLinearSlides.retractSlides();
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            bot.manipulator.horizontalLinearSlides.extendSlides();
        }
        if (subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.LOW);
        }

        if(subsystemController.wasJustPressed(GamepadKeys.Button.A)) {
            bot.manipulator.horizontalArm.setReadyToScoreGround();
        }
        if(subsystemController.wasJustPressed(GamepadKeys.Button.B)) {
            bot.manipulator.horizontalArm.setScoreGround();
        }
        if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bot.manipulator.horizontalArm.setIdle();
        }

        bot.manipulator.horizontalLinearSlides.shiftManual((int) (subsystemController.getLeftY()*5));

        telemetry.addData("Horizontal Current Position", bot.manipulator.horizontalLinearSlides.curPos());
        telemetry.addData("Horizontal Target Position", bot.manipulator.horizontalLinearSlides.targetPos());
        telemetry.addData("Horizontal getDist()", bot.manipulator.horizontalLinearSlides.getDist());
        telemetry.addData("Vertical currentHeight", bot.manipulator.verticalLinearSlides.getCurrentHeight());
        telemetry.addData("Vertical targetHeight", bot.manipulator.verticalLinearSlides.getTargetHeight());



        CommandScheduler.getInstance().run();
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


