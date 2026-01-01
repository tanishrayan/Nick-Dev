package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private DcMotorEx topMotor, bottomMotor;

    // make sure one of the multipliers is positive and the other is negative
    // at all times bc otherwise youre cooked
    // test with very slow speeds first so you dont mess up motor or belt
    private double topMotorMultiplier = 1;
    private double bottomMotorMultiplier = -1;
    private Servo adjustableHood;

    public Launcher(HardwareMap hardwareMap){
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        adjustableHood = hardwareMap.get(Servo.class, "hood");

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public double calculateFlywheelVelocity(double distanceToGoal){
        // ur gonna need to start close to the goal and keep going in incrememnts of however many inches u want
        // and then for each distance away find the best hood and flywheel velocity to use and collect that data
        // once you collect data points and put them into desmos
        // youll put the regression in here with distanceToGoal being the x value
        // and you return the y value.
        // if linear regression fits the data good just use that itll be good
        return 0.0;
    }

    public double calculateHoodAngle(double distanceToGoal){
        // same thing here for the regression
        // ur gonna need to start close to the goal and keep going in incrememnts of however many inches u want
        // and then for each distance away find the best hood and flywheel velocity to use and collect that data
        // once you collect data points and put them into desmos
        // youll put the regression in here with distanceToGoal being the x value
        // and you return the y value.
        // if linear regression fits the data good just use that itll be good
        return 0.0;
    }
    public void setFlywheelVelocity(double targetVelocity){
        // since motors are reversed negate one velocity
        //test and double check to see which one needs to be negated tho
        topMotor.setVelocity(targetVelocity * topMotorMultiplier);
        bottomMotor.setVelocity(targetVelocity * bottomMotorMultiplier);

    }

    public double getFlywheelVelocity(){
        double sum = (topMotor.getVelocity() * topMotorMultiplier) + (bottomMotor.getVelocity() * bottomMotorMultiplier);
        return sum/2;
    }

    public void stopFlywheel(){
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    public void setHoodPosition(double hoodPosition){
        adjustableHood.setPosition(hoodPosition);
    }

}
