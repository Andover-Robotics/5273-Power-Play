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
  protected GamepadEx gamepadEx1, gamepadEx2;
  TimingScheduler timingScheduler;

  //button reader syntax
  // (g1 or g2)  (a, b, lt, lb, etc)

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    gamepadEx2 = new GamepadEx(gamepad2);
    gamepadEx1 = new GamepadEx(gamepad1);
    timingScheduler = new TimingScheduler(this);
    subInit();
    telemetry.addLine("Init done");
    telemetry.addData("Init calibration status: ", bot.imu0.getCalibrationStatus().toString());
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
    gamepadEx1.readButtons();
    gamepadEx2.readButtons();
  }



  boolean buttonSignal(Button button) {
    return gamepadEx1.isDown(button) || gamepadEx2.isDown(button);
  }

  double triggerSignal(Trigger trigger) {
    double in1 = gamepadEx1.getTrigger(trigger),
        in2 = gamepadEx2.getTrigger(trigger);
    return Math.max(in1, in2);
  }


  Vector2d stickSignal(Direction side) {

    Function<GamepadEx, Vector2d> toCoords = pad ->
        side == Direction.LEFT ? new Vector2d(pad.getLeftX(), pad.getLeftY()) :
            new Vector2d(pad.getRightX(), pad.getRightY());

    Vector2d v1 = toCoords.apply(gamepadEx1),
        v2 = toCoords.apply(gamepadEx2);

    return v1.magnitude() > 0.02 ? v1 : v2;
  }


  boolean justPressed(Button button) {
    return gamepadEx1.wasJustPressed(button) || gamepadEx2.wasJustPressed(button);
  }

  boolean justReleased(Button button){
    return !(gamepadEx1.isDown(button) || gamepadEx2.isDown(button)) && (gamepadEx1.wasJustReleased(button) || gamepadEx2.wasJustReleased(button));
  }
}
