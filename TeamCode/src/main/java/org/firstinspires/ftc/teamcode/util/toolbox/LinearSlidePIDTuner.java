package org.firstinspires.ftc.teamcode.util.toolbox;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.opmodes.teleop.BaseOpMode;

import java.util.Map;
import java.util.stream.Stream;

@TeleOp(name = "TBX: Slides Tuner", group = "ARC Toolbox")
public class LinearSlidePIDTuner extends OpMode {


    private InputColumnResponder input = new InputColumnResponderImpl();
    private GamepadEx gamepadEx1;


    private DcMotorEx slideMotor;

    private int targetPositions[]={0, 1700, 2800, 4000}; // Change for your specific use case

    private double pidfCoefficients[] ={1, 0, 0, 0};

    private int currentPIDfIndex=0;




    @Override
    public void init() {
        gamepadEx1= new GamepadEx(gamepad1);
    }

    @Override
    public void start() {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor"); //Change name of motor

        input.clearRegistry();
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setTargetPositionTolerance(20);
    }

    @Override
    public void loop() {
        gamepadEx1.readButtons();
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.X)){
            slideMotor.setTargetPosition(targetPositions[0]);
        }
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.A)){
            slideMotor.setTargetPosition(targetPositions[1]);
        }
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.B)){
            slideMotor.setTargetPosition(targetPositions[2]);
        }
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.Y)){
            slideMotor.setTargetPosition(targetPositions[3]);
        }

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
            currentPIDfIndex=(currentPIDfIndex+3)%4;

        }
        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
            currentPIDfIndex=(currentPIDfIndex+1)%4;
        }

        if(gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
            pidfCoefficients[currentPIDfIndex]+=0.1;
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            pidfCoefficients[currentPIDfIndex]-=0.1;
        }
        PIDFCoefficients pidfCoeffs=new PIDFCoefficients(pidfCoefficients[0], pidfCoefficients[1], pidfCoefficients[2], pidfCoefficients[3]);

        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);

        telemetry.addData("kP", pidfCoefficients[0]);
        telemetry.addData("kI", pidfCoefficients[1]);
        telemetry.addData("kD", pidfCoefficients[2]);
        telemetry.addData("kF", pidfCoefficients[3]);
        telemetry.addData("targetPosition", slideMotor.getTargetPosition());
        telemetry.addData("currentPosition", slideMotor.getCurrentPosition());
        telemetry.addData("velocity", 	slideMotor.getVelocity());

        slideMotor.setPower(1);
    }
}
