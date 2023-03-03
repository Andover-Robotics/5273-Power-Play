package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.util.utilclasses.TimingScheduler;

import java.util.function.Function;


public abstract class BaseOpMode extends OpMode {

  protected Bot bot;
  protected double driveSpeed;
  protected GamepadEx driveController, subsystemController;
  TimingScheduler timingScheduler;

  //button reader syntax
  // (g1 or g2)  (a, b, lt, lb, etc)

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    subsystemController = new GamepadEx(gamepad2);
    driveController = new GamepadEx(gamepad1);
    timingScheduler = new TimingScheduler(this);
    subInit();
    telemetry.addLine("Init done");
    //telemetry.addData("Init calibration status: ", bot.imu0.getCalibrationStatus().toString());
    telemetry.update();
  }

  @Override
  public void loop() {
    updateButtons();
    subLoop();
  }

  protected abstract void subInit();

  protected abstract void subLoop();

  void updateButtons(){
    driveController.readButtons();
    subsystemController.readButtons();
  }

}
