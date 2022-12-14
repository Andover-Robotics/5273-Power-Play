package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {

    private boolean centricity = true;
    private final double TRIGGER_CONSTANT = 0.15;
    private final double SLOW_MODE_PERCENT = 0.4;
    private double slowPercentage;
    private double fieldCentricOffset0 = 0.0;
    private double fieldCentricOffset1 = 0.0;

    // opmode vars here =========================================================================
    public void subInit() {
        driveSpeed = 1.0;
    }

    public void subLoop() {

//      Update
//      cycle = 1.0/(time-prevRead);
//      prevRead = time;
//      timingScheduler.run();

//      Movement =================================================================================================
        slowPercentage = 1 - gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        drive();

        //Subsystem Control =========================================================================================

        //ScrollLinearSlides

        bot.outtake.linearSlides.scroll(gamepadEx2.getLeftY());

        //PresetLinearSlides

        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            bot.outtake.linearSlides.decrementLevel();
        }

        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            telemetry.addData("upping!", true);
            bot.outtake.linearSlides.incrementLevel();
        }
        if(gamepadEx2.wasJustPressed(Button.DPAD_UP)){
            bot.outtake.linearSlides.extend();
        }
        if(gamepadEx2.wasJustPressed(Button.DPAD_DOWN)) {
           bot.outtake.linearSlides.retract();
        }

        // automated intake -> outtake
        if (gamepadEx2.wasJustPressed(Button.DPAD_LEFT)) {
           // bot.outtake.runOuttake();
        }

        // automated outtake + reset
        if (gamepadEx2.isDown(Button.DPAD_RIGHT)) {
            bot.outtake.claw.openGrabClaw();
        }
        // release claw and reset system to intake mode
        else if (gamepadEx2.wasJustReleased(Button.DPAD_RIGHT)) {
            bot.outtake.claw.lowerRotateClaw();
            //bot.outtake.linearSlides.retract();
        }


        //TODO: Tele-Op Automation?

        //Claw Control
        //TODO: implement claw rotation
        if(gamepadEx2.wasJustPressed(Button.Y)){
            bot.outtake.claw.raiseRotateClaw();
        }
        else if(gamepadEx2.wasJustPressed(Button.A)){
            bot.outtake.claw.lowerRotateClaw();
        }

        if(gamepadEx2.wasJustPressed(Button.X)){
            bot.outtake.claw.closeGrabClaw();
        }
        else if(gamepadEx2.wasJustPressed(Button.B)){
            bot.outtake.claw.openGrabClaw();
        }

        //Scroll Claw Arm
        bot.outtake.claw.scrollArm(gamepadEx2.getRightY());


        /*
        ==================OUT OF DATE=================
        Controller 1
        Buttons
            A: intake in
            B: hold to release claw, release to reset outtake to base position
            X: intake out
            Y: raise slides to set positions
        DPAD
            L: close claw
            D: rotate claw down
            U: rotate claw up
            R: open claw
        Joystick
            L: movement (field centric or robot centric)
            R: Set orientation / rotation (determine through practice)
        Joystick Buttons
            L: reset field centric offset
            R: switch centricity
        Trigger (unused)
        Bumper:
            L: decrement slide height selection
            R: increment slide height selection
        Other
        Start:  Back: switch between automation and driving
         */

        CommandScheduler.getInstance().run();

        telemetry.addData("X", bot.roadRunner.getPoseEstimate().getX());
        telemetry.addData("Y", bot.roadRunner.getPoseEstimate().getY());
        telemetry.addData("Heading", bot.roadRunner.getPoseEstimate().getHeading());
        telemetry.addData("Driver Left Stick", gamepadEx1.getLeftX() + " : " + gamepadEx1.getLeftY());
        telemetry.addData("Driver Right Stick", gamepadEx1.getRightX() + " : " + gamepadEx1.getRightY());
        telemetry.addData("slide positions", bot.outtake.linearSlides.leftSlideMotor.getCurrentPosition()+" , "+bot.outtake.linearSlides.rightSlideMotor.getCurrentPosition());
        telemetry.addData("slide target position", bot.outtake.linearSlides.leftSlideMotor.getTargetPosition());
        //telemetry.addData("Current Slide Preset", bot.outtake.linearSlides.currentLevel);
        telemetry.addData("Rotate Servo Position", bot.outtake.claw.getRotatePosition());
    }

    private void drive() {

        final double gyroscopeTolerance = 10;

        double temporaryAngle0 = bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                - fieldCentricOffset0;
        double temporaryAngle1 = bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                - fieldCentricOffset1;

        // set absolute value of angle always less than or equal to 180

        final double gyroscopeAngle0 = temporaryAngle0; // accounts for rotation of extension hub and center-lifts angle to -180->180

        // if imu is null, then use other imu

        final double gyroscopeAngle1 = (bot.imu1 != null) ? temporaryAngle1 :gyroscopeAngle0;
        final double averageGyroscopeAngle = (( gyroscopeAngle0 + gyroscopeAngle1)/2);

        telemetry.addData("Invert Right", bot.drive.isRightSideInverted());
        telemetry.addData("Centricity", centricity);
        telemetry.addData("Average Gyroscope Angle", averageGyroscopeAngle);

//        telemetry.addData("temporaryAngle0", temporaryAngle0);
//        telemetry.addData("temporaryAngle1", temporaryAngle1);
//        telemetry.addData("gyroAngle0", gyroscopeAngle0);
//        telemetry.addData("gyroAngle1", gyroscopeAngle1);
//        telemetry.addData("fieldCentricOffset0", fieldCentricOffset0);
//        telemetry.addData("fieldCentricOffset1", fieldCentricOffset1);

        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() , 0);
        if (bot.roadRunner.mode == RRMecanumDrive.Mode.IDLE) {
            boolean dpadPressed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    || gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT));
            boolean buttonPressed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B));

            double forwardSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : -1) : 0;
            double strafeSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : -1) : 0;
            double turnSpeed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B)) ? (gamepadEx1.getButton(GamepadKeys.Button.B) ? 1 : -1) : 0;

            // temp

            bot.fixMotors();

            // temporary drive function
            // negatived the strafe and turn speed because they were opposite
            bot.drive(
                    -driveVector.getX() * driveSpeed * slowPercentage,
                    driveVector.getY() * driveSpeed * slowPercentage,
                    -turnVector.getX() * driveSpeed * slowPercentage
                    );

//            if (centricity) {//epic java syntax
//                bot.drive.driveFieldCentric(
//                        driveVector.getX() * driveSpeed,
//                        driveVector.getY() * driveSpeed,
//                        turnVector.getX() * driveSpeed,
//                         Math.abs(gyroscopeAngle1 - gyroscopeAngle0) < gyroscopeTolerance ? averageGyroscopeAngle : gyroscopeAngle0
//
//                        //field centric W
//
//
//                        // Epic Java Syntax here
//                        /*
//                         * In theory, this check ensures that when the averageGyroscopeAngle is VERY off
//                         * due to one IMU giving  near -180, and the second giving near 180 which SHOULD be considered an angle of 0 but its actually in the opposite direction
//                         * This problem was encountered while first testing the dual IMU dependant field centric drive
//                         * the robot would run two motors on the corners of the robot in opposite directions, causing negligible movement
//                         * Because I believe the rarer incorrect averages, these ternary statements, should correct this.
//                         */
//                );
//            }
//            else if (dpadPressed || buttonPressed) {
//                double temporaryDriveSpeed = driveSpeed * SLOW_MODE_PERCENT;
//                bot.drive.driveRobotCentric(
//                        strafeSpeed * temporaryDriveSpeed,
//                        forwardSpeed * temporaryDriveSpeed,
//                        turnSpeed * temporaryDriveSpeed
//                );
//            }
//
//            else {
//                bot.drive.driveRobotCentric(
//                        driveVector.getX() * driveSpeed,
//                        driveVector.getY() * driveSpeed,
//                        turnVector.getX() * driveSpeed
//                );
//            }

        }

        /* set field centric offset
         * imu is inertial measurement unit, is in the control hubs and is set at 0 when the robot is started
         * offset is set at the angle the imu measures from where it was started, allowing calibration of field centricity
         */
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            fieldCentricOffset0 = bot.imu0.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
            fieldCentricOffset1 = bot.imu1.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
        }

        // switch centricity mode
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            centricity = !centricity;
        }
    }
}
