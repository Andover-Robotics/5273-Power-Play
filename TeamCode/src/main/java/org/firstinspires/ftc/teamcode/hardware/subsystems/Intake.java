package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Intake {


    private class Color{

        private final int COLOR_TOLERANCE=20;
        public int red,green,blue;

        public Color(int red, int green, int blue){
            this.red=red;
            this.green=green;
            this.blue=blue;
        }

        public boolean matchColor(Color c){
             return (Math.abs(this.red-c.red)<COLOR_TOLERANCE && Math.abs(this.green-green)<COLOR_TOLERANCE && Math.abs(this.blue-blue)<COLOR_TOLERANCE);
        }
    }

    private Color redCone=new Color(255,0,0);  //test and tune
    private Color blueCone=new Color(0,0,255);//test and tune


    private final int OVERHEAD= 0;//in ticks
    private final double INCHES_TO_TICKS=100.0; //tune

    private final int TOLERANCE=100;


    private final double kP=0.05;
    private final double kV=0.01;

    private DistanceSensor distanceSensor;

    private ColorSensor colorSensor;

    private MotorEx leftHorizontalMotor;
    private MotorEx rightHorizontalMotor;

    private Servo intakeClaw;

    private final double OPEN_CLAW_POSITION=0.3;

    private final double CLOSE_CLAW_POSITION=0.0;

    public Intake(HardwareMap hardwareMap){
        colorSensor=hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor=hardwareMap.get(DistanceSensor.class, "distanceSensor");
        leftHorizontalMotor=hardwareMap.get(MotorEx.class, "leftHorizontalMotor");
        rightHorizontalMotor=hardwareMap.get(MotorEx.class, "rightHorizontalMotor");
        intakeClaw=hardwareMap.get(Servo.class, "intakeClaw");
        initializeMotor(rightHorizontalMotor);
        initializeMotor(leftHorizontalMotor);
    }

    private void initializeMotor(MotorEx motor){
        motor.resetEncoder();
        motor.setTargetPosition(0);
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
        motor.setPositionCoefficient(kP);
    }

    public void intake(){
        extend();
        openClaw();
        closeClaw();
        retract();
    }





    public void extend(){
        Color detectedColor=new Color(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        if(!detectedColor.matchColor(redCone)&&!detectedColor.matchColor(blueCone)) {
            return;
        }
        double distance =distanceSensor.getDistance(DistanceUnit.INCH);
        int position= (int) (distance*INCHES_TO_TICKS+OVERHEAD);
        rightHorizontalMotor.setTargetPosition(position);
        leftHorizontalMotor.setTargetPosition(position);
        rightHorizontalMotor.set(kV);
        leftHorizontalMotor.set(kV);



    }
    public void openClaw(){
        intakeClaw.setPosition(OPEN_CLAW_POSITION);
    }
    public void closeClaw(){
        intakeClaw.setPosition(CLOSE_CLAW_POSITION);
    }

    public void retract(){
        rightHorizontalMotor.setTargetPosition(0);
        leftHorizontalMotor.setTargetPosition(0);
        rightHorizontalMotor.set(kV);
        leftHorizontalMotor.set(kV);

    }
}
