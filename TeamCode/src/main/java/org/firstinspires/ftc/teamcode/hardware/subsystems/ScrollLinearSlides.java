package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/***
 * temporary linear slide class for qualifier 1
 */
public class ScrollLinearSlides {

    private static double UP_POWER_CONSTANT=1;
    private static double DOWN_POWER_CONSTANT=0.25;
    private static int MAX_HEIGHT=2000;

    public final DcMotor leftSlideMotor;
    public final DcMotor rightSlideMotor;

    public ScrollLinearSlides(HardwareMap hardwareMap) {

        leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        initializeSlideMotor(leftSlideMotor);
        initializeSlideMotor(rightSlideMotor);

    }

    private void initializeSlideMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void resetEncoders(){
    }

    public void move(double power){
        if(power>0){
            leftSlideMotor.setPower(power*UP_POWER_CONSTANT); //invert left
            rightSlideMotor.setPower(-power*UP_POWER_CONSTANT);
        }
        else{
            leftSlideMotor.setPower(power*DOWN_POWER_CONSTANT); //invert left
            rightSlideMotor.setPower(-power*DOWN_POWER_CONSTANT);
        }


    }




}
