package org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.junction;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Bot;

@Autonomous(name = "Junction Detection")
public class JunctionDetectionTest extends OpMode {

    private JunctionDetectionDetector junctionDetectionDetector;
    private Bot bot;
    private static final int MINIMUM_X = 420;
    private static final int MAXIMUM_X = 540;

    @Override
    public void init() {
        junctionDetectionDetector = new JunctionDetectionDetector(this, telemetry);
        bot = Bot.getInstance(this);
    }

    @Override
    public void loop() {
        junctionDetectionDetector.currentlyDetected()
                .ifPresent((result -> {

                    telemetry.addData("Status", "Found Junction");
                    telemetry.addData("X", result.first);
                    telemetry.addData("Y", result.second);

                    while (result.first < MINIMUM_X || result.second > MAXIMUM_X) {

                        if (result.first < MINIMUM_X) {

                            telemetry.addData("Movement", "Strafing Right");
                            bot.drive.driveRobotCentric(- 0.02, 0.0, 0.0);

                        }

                        else {

                            telemetry.addData("Movement", "Strafing Left");
                            bot.drive.driveRobotCentric( 0.02, 0.0, 0.0);

                        }

                    }

                    telemetry.addData("Status", "Aligned with Junction");

                }));
    }
}
