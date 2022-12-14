package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Manipulator;

public class Bot {

    // in TeleOp and Autonomous we should be able to call "new Bot(this)"

    public static Bot instance;

    public final Manipulator outtake;

    // front left, front right, back left, back right

    public final MotorEx[] driveTrainMotors;

    //required subsystems

    public final MecanumDrive drive;
    public final RRMecanumDrive roadRunner;
    public final BNO055IMU imu0;
    public final BNO055IMU imu1;
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

        instance.initializeImu(instance.imu0);
        instance.initializeImu(instance.imu1);

        return instance;

    }

    public void reset(){

        instance = new Bot(opMode);

    }

    private Bot(OpMode opMode){

        this.opMode = opMode;
        enableAutoBulkRead();

        outtake = new Manipulator(opMode.hardwareMap);

        //this.templateSubsystem = new TemplateSubsystem(opMode);

        driveTrainMotors = new MotorEx[]{
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFL, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFR, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBL, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBR, Motor.GoBILDA.RPM_435)
        };
        // For the blue driver hub, only the middle USB 2.0 port works.
//        // left motors
//        driveTrainMotors[0].setInverted(false);
//        driveTrainMotors[2].setInverted(false);
//        driveTrainMotors[1].setInverted(false);
//        driveTrainMotors[3].setInverted(false);

        for(MotorEx motor : driveTrainMotors){
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.setInverted(false);
            // temp - from lightning code
            motor.setRunMode(Motor.RunMode.RawPower);
        }

        //required subsystems
        this.drive = new MecanumDrive(false,
                driveTrainMotors[0],
                driveTrainMotors[1],
                driveTrainMotors[2],
                driveTrainMotors[3]);

        this.roadRunner = new RRMecanumDrive(opMode.hardwareMap);

        this.imu0 = opMode.hardwareMap.get(BNO055IMU.class, "imu0");
        this.imu1 = opMode.hardwareMap.get(BNO055IMU.class, "imu1");

        this.initializeImu(imu0);
        this.initializeImu(imu1);

    }

    private void initializeImu(BNO055IMU imu) {

        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

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

    // temporary code copied from lightning to fix inverted motors problem which supposedly originates
    // from the given drive functions (driveFieldCentric in HDrive.java)

    public void drive(double strafeSpeed, double forwardBackSpeed, double turnSpeed){

        double[] speeds = {
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed
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

        driveTrainMotors[0].set(speeds[0] * -1);
        driveTrainMotors[1].set(speeds[1]);
        driveTrainMotors[2].set(speeds[2] * -1);
        driveTrainMotors[3].set(speeds[3]);
    }

    private void enableAutoBulkRead() {

        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(BulkCachingMode.AUTO);
        }

    }

}
