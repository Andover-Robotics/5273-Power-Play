package org.firstinspires.ftc.teamcode.hardware;

import android.provider.Browser;
import android.provider.Settings;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Outtake;

public class Bot {
    // in TeleOp and Autonomous we should be able to call "new Bot(this)"

    public static Bot instance;

    public final Outtake outtake;
    public final Intake intake;

    public final MotorEx[] driveTrainMotors;

    //required subsystems
    public final MecanumDrive drive;
    public final RRMecanumDrive roadRunner;
    public final BNO055IMU imu0;
    public final BNO055IMU imu1;
    //  public final Cosmetics cosmetics;
//  public Pair<ExpansionHubEx, ExpansionHubEx> hubs = null;
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
        instance.opMode = opMode;
        instance.initializeImu(instance.imu0);
        instance.initializeImu(instance.imu1);
        return instance;
    }

    public void reset(){
        //TODO: add reset code here
    }

    private Bot(OpMode opMode){
        this.opMode = opMode;
        enableAutoBulkRead();

        outtake = new Outtake(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);

        //this.templateSubsystem = new TemplateSubsystem(opMode);

        driveTrainMotors = new MotorEx[]{
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFL, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFR, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBL, Motor.GoBILDA.RPM_435),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBR, Motor.GoBILDA.RPM_435)
        };
        // For the blue driver hub, only the middle USB 2.0 port works.
        // left motors
//        driveTrainMotors[0].setInverted(true);
//        driveTrainMotors[2].setInverted(true);
        for(MotorEx motor : driveTrainMotors){
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(BulkCachingMode.AUTO);
        }
    }
}
