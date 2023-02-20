package org.firstinspires.ftc.teamcode.drive.localizer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.util.utilclasses.Encoder;

public class SensorFusionData {
  private final Encoder left, center, right;
  private final IMU imu1, imu2;

  public final int[] positions = new int[3];
  public final double[] velocities = new double[3];
  public double heading1, heading2;

  public SensorFusionData(HardwareMap map, IMU i1, IMU i2) {
    left = new Encoder(map.get(DcMotorEx.class, GlobalConfig.EncoderValues.leftEncoderPort));
    right = new Encoder(map.get(DcMotorEx.class, GlobalConfig.EncoderValues.rightEncoderPort));
    center = new Encoder(map.get(DcMotorEx.class, GlobalConfig.EncoderValues.centerEncoderPort));

    left.setDirection(GlobalConfig.EncoderValues.leftEncoderDirection);
    right.setDirection(GlobalConfig.EncoderValues.rightEncoderDirection);
    center.setDirection(GlobalConfig.EncoderValues.centerEncoderDirection);
    imu1 = i1;
    imu2 = i2;
  }

  public void update() {
    positions[0] = left.getCurrentPosition();
    positions[1] = center.getCurrentPosition();
    positions[2] = right.getCurrentPosition();

    velocities[0] = left.getCorrectedVelocity();
    velocities[1] = center.getCorrectedVelocity();
    velocities[2] = right.getCorrectedVelocity();

    heading1 = imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    heading2 = imu2.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
  }
}
