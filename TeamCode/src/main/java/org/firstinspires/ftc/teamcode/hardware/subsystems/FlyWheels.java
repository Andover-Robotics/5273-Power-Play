package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlyWheels {
    private final MotorEx rightMotor;
    private final MotorEx leftMotor;

    private final double RUN_POWER = 0.85;
    private final double STOP_POWER = 0;

    public FlyWheels(HardwareMap hardwareMap) {
        leftMotor = new MotorEx(hardwareMap, "leftIntakeWheel", Motor.GoBILDA.RPM_312);
        rightMotor = new MotorEx(hardwareMap, "rightIntakeWheel", Motor.GoBILDA.RPM_312);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        leftMotor.setInverted(true);
    }

    public void run() {
        runAtPower(RUN_POWER);
    }

    public void runReverse() {
        runAtPower(- RUN_POWER);
    }

    private void runAtPower(double power) {
        leftMotor.set(- power);
        rightMotor.set(- power);
    }

    public void stop() {
        runAtPower(STOP_POWER);
    }
}
