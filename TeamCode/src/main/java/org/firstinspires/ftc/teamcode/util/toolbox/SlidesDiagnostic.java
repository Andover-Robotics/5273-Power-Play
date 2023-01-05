package org.firstinspires.ftc.teamcode.util.toolbox;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;
import java.util.stream.Stream;

@TeleOp(name = "TBX: Slides Diagnostic", group = "ARC Toolbox")
public class SlidesDiagnostic extends OpMode {
    private static int TP_DELTA = 5;

    private InputColumnResponder input = new InputColumnResponderImpl();
    private DcMotor motor0;
    private DcMotor motor1;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        input.update();
    }

    @Override
    public void start() {
        motor0=hardwareMap.get(DcMotor.class, "leftSlideMotor");
        motor1=hardwareMap.get(DcMotor.class, "rightSlideMotor");
        input.clearRegistry();

        input.register(() -> gamepad1.a, modeSelector::selectNext)
                .register(() -> gamepad1.b, zpbSelector::selectNext)
                .register(() -> gamepad1.dpad_up, () -> motor0.setTargetPosition(motor0.getTargetPosition() + TP_DELTA))
                .register(() -> gamepad1.dpad_down, () -> motor0.setTargetPosition(motor0.getTargetPosition() - TP_DELTA))
                .register(() -> gamepad1.dpad_up, () -> motor1.setTargetPosition(motor1.getTargetPosition() + TP_DELTA))
                .register(() -> gamepad1.dpad_down, () -> motor1.setTargetPosition(motor1.getTargetPosition() - TP_DELTA));
    }

    private Selector modeSelector = new Selector(enumOrdinals(DcMotor.RunMode.values())),
            zpbSelector = new Selector(enumOrdinals(DcMotor.ZeroPowerBehavior.values()));

    @Override
    public void loop() {
        telemetry.addLine("Controls")
                .addData("Left stick Y", "Controls Power Value")
                .addData("A", "Switches modes")
                .addData("B", "Toggles zero power behavior")
                .addData("Dpad Up", "Increases target position")
                .addData("Dpad Down", "Decreases target position");

        telemetry.addLine("Motor Data")
                .addData("Type", motor0.getMotorType().getName())
                .addData("Power", "%.3f", motor0.getPower())
                .addData("Mode", motor0.getMode().name())
                .addData("Z.P.B.", motor0.getZeroPowerBehavior().name())
                .addData("Current Pos", "%d", motor0.getCurrentPosition())
                .addData("Target Pos", "%d", motor0.getTargetPosition())
                .addData("Busy?", motor0.isBusy() ? "Yes" : "No");

        telemetry.addLine("Selections")
                .addData("Selected Mode", modeSelector.selected())
                .addData("Selected ZPB", zpbSelector.selected());

        input.update();
        motor0.setPower(-gamepad1.left_stick_y);
        motor1.setPower(gamepad1.left_stick_y);
    }

    private <T extends Enum> Stream<String> enumOrdinals(T[] values) {
        return Stream.of(values)
                .map(Enum::name);
    }
}
