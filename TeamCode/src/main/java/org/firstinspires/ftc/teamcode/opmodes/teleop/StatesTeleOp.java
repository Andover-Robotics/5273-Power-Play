package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VerticalLinearSlides;
import org.firstinspires.ftc.teamcode.util.utilclasses.TimingScheduler;

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
//        bot.manipulator.verticalLinearSlides.resetSlideEncoders();
        /** TODO ===========================================================================================
         * SLIDE ENCODER PROBLEMS:
         * IF I DON'T RESET THE ENCODERS, THEN THE SLIDES DON'T GO TO THE CORRECT POSITION
         * HOWEVER, THIS MEANS THAT THE SLIDES HAVE TO START FROM THE BOTTOM AT THE START OF TELEOP
         * WHICH DOESN'T WORK BECAUSE AUTONOMOUS CAUSES THE SLIDES TO POP UP A BIT
         * ALSO, THE VERTICAL SLIDE CLAW DOESN'T CLOSE WELL
         * TODO ===========================================================================================
         */
        bot.resetJavaGCCleanedThingsSoSad();
        CommandScheduler.getInstance().enable();
        bot.manipulator.horizontalArm.closeClaw();
        bot.manipulator.horizontalArm.setIdle();
        bot.manipulator.verticalLinearSlides.hover();
        bot.manipulator.verticalArm.setTransfer();
        bot.manipulator.verticalArm.closeClaw();
        telemetry.addLine("Init done.");
    }

    @Override
    public void start() {


    }


    @Override
    protected void subLoop() {
        //drive=============================================================================
        yawPitchRollAngles = bot.imu1.getRobotYawPitchRollAngles();

        if (driveController.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bot.imu1.resetYaw();
        }
        if(driveController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            bot.manipulator.verticalArm.setArmOuttake();
            bot.manipulator.verticalArm.closeClaw();
        }
        else if(driveController.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            bot.manipulator.verticalArm.openClaw();
            timingScheduler.defer(0.2, () -> {
                bot.manipulator.verticalArm.closeClaw();
                bot.manipulator.verticalArm.setTransfer();
                bot.manipulator.verticalLinearSlides.hover();
                timingScheduler.defer(0.2, () -> {
                    bot.manipulator.verticalArm.openClaw();
                });
            });
        }

        drive();

        //subsystem===========================================================================

        if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            bot.manipulator.verticalLinearSlides.hover();
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.LOW);
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.MEDIUM);
        }
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            bot.manipulator.verticalLinearSlides.setLevel(VerticalLinearSlides.Level.HIGH);
        }
        // reset linear slides to ready intake position


        // finishes horizontal intake
        if(subsystemController.wasJustPressed(GamepadKeys.Button.X)){  //retract horizontal slides and setIdle
            bot.manipulator.horizontalArm.closeClaw();
        }
        else if(subsystemController.wasJustReleased(GamepadKeys.Button.X)) {
            bot.manipulator.horizontalArm.closeClaw();
            timingScheduler.defer(0.2, () -> {
                bot.manipulator.horizontalLinearSlides.retractSlides();
                bot.manipulator.horizontalArm.setIdle();
                timingScheduler.defer(2, () -> {
                    bot.manipulator.horizontalArm.setTransfer();
                    timingScheduler.defer(1, () -> {
                        bot.manipulator.horizontalArm.openClaw();
                        bot.manipulator.verticalLinearSlides.hover();
                        timingScheduler.defer(0.5, () -> {
                            bot.manipulator.horizontalArm.setIdle();
                            timingScheduler.defer(0.1, () -> {
                                bot.manipulator.horizontalArm.closeClaw();
                                bot.manipulator.verticalArm.closeClaw();
                                timingScheduler.defer(0.3, () -> {
                                    bot.manipulator.verticalArm.setOuttake();
                                });
                            });
                        });
                    });
                });
            });
        }

        // starts horizontal intake
        else if(subsystemController.wasJustPressed(GamepadKeys.Button.B)){ //extend horizontal slides and setIntake
            bot.manipulator.horizontalArm.closeClaw();
            bot.manipulator.horizontalLinearSlides.extendSlides();
            bot.manipulator.horizontalArm.setIntake();
            timingScheduler.defer(0.2, () -> {
                bot.manipulator.horizontalArm.openClaw();
            });
        }

        // set linear slide outtake position
        if (subsystemController.wasJustPressed(GamepadKeys.Button.Y)) {
            bot.manipulator.verticalArm.closeClaw();
            bot.manipulator.verticalArm.setOuttake();
        }

        if (subsystemController.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bot.manipulator.closeVerticalClaw();
        } else if (subsystemController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            bot.manipulator.openVerticalClaw();
        }

        // manual horizontal slide movement
        bot.manipulator.horizontalLinearSlides.shiftManual((int) (subsystemController.getLeftY()*5));

        //TODO add other controls and add automated intake and outtake

        CommandScheduler.getInstance().run();
        timingScheduler.run();


        telemetry.addData("Horizontal Current Position", bot.manipulator.horizontalLinearSlides.curPos());
        telemetry.addData("Horizontal Target Position", bot.manipulator.horizontalLinearSlides.targetPos());
        telemetry.addData("Horizontal getDist()", bot.manipulator.horizontalLinearSlides.getDist());
        telemetry.addData("Vertical currentHeight left", bot.manipulator.verticalLinearSlides.getCurrentLeftHeight());
        telemetry.addData("Vertical currentHeight right", bot.manipulator.verticalLinearSlides.getCurrentRightHeight());
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
