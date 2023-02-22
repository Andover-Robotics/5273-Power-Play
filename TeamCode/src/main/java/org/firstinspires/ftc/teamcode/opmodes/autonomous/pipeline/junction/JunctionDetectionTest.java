package org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.junction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Bot;

@Autonomous(name = "Cone Detection")
public class JunctionDetectionTest extends OpMode {

    private JunctionDetector junctionDetector;
    private Bot bot;
    private static final int MINIMUM_X = 420;
    private static final int MAXIMUM_X = 540;

    @Override
    public void init() {
        junctionDetector = new JunctionDetector(this, telemetry);
        bot = Bot.getInstance(this);
    }

    @Override
    public void loop() {
        junctionDetector.currentlyDetected()
                .ifPresent((result -> {

                    telemetry.addData("Status", "Found Cone");
                    telemetry.addData("X", result.first);
                    telemetry.addData("Y", result.second);

                    while (result.first < MINIMUM_X || result.second > MAXIMUM_X) {
                        if (result.first < MINIMUM_X) {
                            telemetry.addData("Movement", "Strafing Right");
                            bot.drive(- 0.02, 0.0, 0.0);
                        }

                        else {
                            telemetry.addData("Movement", "Strafing Left");
                            bot.drive( 0.02, 0.0, 0.0);
                        }
                    }

                    telemetry.addData("Status", "Aligned with Cone");

                }));
    }
}
