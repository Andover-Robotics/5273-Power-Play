package org.firstinspires.ftc.teamcode.util.toolbox;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TBX:  Distance Sensor Test", group = "ARC Toolbox")
public class DistanceSensorTest extends OpMode {
    private DistanceSensor distanceSensor;

    @Override
    public void init(){
        distanceSensor= hardwareMap.get(DistanceSensor.class, "distanceSensor");
        telemetry.addData("Sensor Version", distanceSensor.getVersion());
        telemetry.addData("Sensor Device Name", distanceSensor.getDeviceName());
    }

    @Override
    public void loop() {
        telemetry.addData("distance in inches", distanceSensor.getDistance(DistanceUnit.INCH));

    }
}
