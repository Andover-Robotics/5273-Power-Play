package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Manipulator;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

public class Bot {

    // in TeleOp and Autonomous we should be able to call "new Bot(this)"

    public static Bot instance;

    public final Manipulator manipulator;

    // front left, front right, back left, back right

    public final MotorEx[] driveTrainMotors;

    //required subsystems

    public IMU imu0;
    public IMU imu1;
    public boolean fieldCentricRunMode = true;
    public OpMode opMode;

    /** Get the current Bot instance from somewhere other than an OpMode */

    public static Bot getInstance() {

        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
        }
        return instance;

    }

    public static Bot getInstance(OpMode opMode) {

        if (instance == null) {
            return instance = new Bot(opMode);
        }

        // left motors
        // it's inverting the second time, so maybe inverting it when instance != null might work?

        instance.driveTrainMotors[2].setInverted(false);
        instance.driveTrainMotors[0].setInverted(false);

        // right motors

        instance.driveTrainMotors[1].setInverted(false);
        instance.driveTrainMotors[3].setInverted(false);


        instance.opMode = opMode;

        return instance;

    }

    public void reset(){

        instance = new Bot(opMode);
    }

    private Bot(OpMode opMode){

        this.opMode = opMode;
        enableAutoBulkRead();

        manipulator = new Manipulator(opMode.hardwareMap);

        //this.templateSubsystem = new TemplateSubsystem(opMode);

        driveTrainMotors = new MotorEx[]{
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFL, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFR, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBL, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBR, Motor.GoBILDA.RPM_435)
        };

        //required subsystems
        for(MotorEx motor : driveTrainMotors){
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.setInverted(false);
            // temp - from lightning code
            motor.setRunMode(Motor.RunMode.RawPower);
        }
        imu0=opMode.hardwareMap.get(IMU.class, "imu0");
        imu1=opMode.hardwareMap.get(IMU.class, "imu1");
        imu0.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu1.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );
    }

    // temporary code from Lightning

    public void fixMotors() {

        // Mecanum drive is not used

//        drive.setRightSideInverted(true);

        for (MotorEx motor : driveTrainMotors) {
            motor.setInverted(false);
            motor.setRunMode(Motor.RunMode.RawPower);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

    }


    public void drive(double strafeSpeed, double forwardBackSpeed, double turnSpeed){
        double[] speeds = {
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed
        };

        double maxSpeed = 0;

        for(int i = 0; i < 4; i++){
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        if(maxSpeed > 1) {
            for (int i = 0; i < 4; i++){
                speeds[i] /= maxSpeed;
            }
        }

//        for (int i = 0; i < 4; i++) {
//            driveTrainMotors[i].set(speeds[i]);
//        }
        // manually invert the left side

        driveTrainMotors[0].set(-speeds[0]);
        driveTrainMotors[1].set(speeds[1]);
        driveTrainMotors[2].set(-speeds[2]);
        driveTrainMotors[3].set(speeds[3]);
    }

    public void drive(double strafeSpeed, double forwardBackSpeed, double turnSpeed, double heading){
        double rotateX=strafeSpeed*Math.cos(-heading)-forwardBackSpeed*Math.sin(-heading);
        double rotateY=strafeSpeed*Math.sin(-heading)+forwardBackSpeed*Math.cos(-heading);
        double speeds[]= {
                (rotateY + rotateX + turnSpeed),
                (rotateY - rotateX - turnSpeed),
                (rotateY - rotateX + turnSpeed),
                (rotateY + rotateX - turnSpeed)
        };

        double maxSpeed = 0;

        for(int i = 0; i < 4; i++){
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        if(maxSpeed > 1) {
            for (int i = 0; i < 4; i++){
                speeds[i] /= maxSpeed;
            }
        }


        driveTrainMotors[0].set(-speeds[0]);
        driveTrainMotors[1].set(speeds[1]);
        driveTrainMotors[2].set(-speeds[2]);
        driveTrainMotors[3].set(speeds[3]);
    }


    private void enableAutoBulkRead() {

        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(BulkCachingMode.AUTO);
        }

    }

    public double getDriveCurrentDraw() {
        double currentDraw=0;
        for(MotorEx motor: driveTrainMotors){
            currentDraw+=motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
        }
        return currentDraw;
    }
}
