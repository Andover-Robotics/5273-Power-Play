package org.firstinspires.ftc.teamcode.util.toolbox;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "TBX: Color Sensor Test", group = "ARC Toolbox")
public class ColorSensorTest extends OpMode {
    private RevColorSensorV3 colorSensor;

    @Override
    public void init() {
        colorSensor=hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        telemetry.addData("Color Sensor Name", colorSensor.getDeviceName());
        telemetry.addData("Color Sensor Version", colorSensor.getVersion());
        telemetry.addData("Color Sensor Manufacturer", colorSensor.getManufacturer());
    }

    @Override
    public void loop() {
        telemetry.addLine("rgb tuple ("+colorSensor.red()+" ,"+colorSensor.green()+" ,"+ colorSensor.blue()+")");
        telemetry.addData("distance(in)", colorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("light detected", colorSensor.getLightDetected());
        telemetry.addData("raw light detected", colorSensor.getRawLightDetected());
        telemetry.update();
    }
}
