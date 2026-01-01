package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "DATA COLLECTION FOR REGRESSION")
public class CollectDataForRegression extends OpMode {

    private DcMotorEx topMotor, bottomMotor;
    private Servo hood;
    // make sure one of the multipliers is positive and the other is negative
    // at all times bc otherwise youre cooked
    // test with very slow speeds first so you dont mess up motor or belt
    private double topMotorMultiplier = 1;
    private double bottomMotorMultiplier = -1;

    private double targetVelocity = 0;
    private double velocity = 0;

    private double hoodPosition = 0;

    //TODO: PUT IN A CORRECT HOOD INCREMENT BASED ON AXON, how much should it go up or down on one click
    private double hoodIncrement = 0.1;

    @Override
    public void init() {
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        hood = hardwareMap.get(Servo.class, "hood");

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodPosition = hood.getPosition();

        telemetry.addLine("Hardware Initialized and ready to run test");
        telemetry.addLine("Start close to the goal and move back in  distance increments");
        telemetry.addLine("At each distance, find a good hood angle and flywheel velocity.");
        telemetry.addLine("To adjust the velocity use dpad up and down and look at telem data to see");
        telemetry.addLine("To adjust the hood position use y and a and look at telem data to see");


    }

    @Override
    public void loop() {
        velocity = getFlywheelVelocity();
        setFlywheelVelocity(targetVelocity);

        if (gamepad1.dpadUpWasPressed()){
            targetVelocity += 25;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetVelocity -= 25;
        }

        if (gamepad1.yWasPressed()){
            hood.setPosition(hoodPosition + hoodIncrement);
        } else if (gamepad1.aWasPressed()) {
            hood.setPosition(hoodPosition - hoodIncrement);

        }

        hoodPosition = hood.getPosition();

        telemetry.addData("Target Vel: ", targetVelocity);
        telemetry.addData("Actual Vel: ", velocity);
        telemetry.addData("Hood position : ", hoodPosition);

        telemetry.update();




    }

    private void setFlywheelVelocity(double targetVelocity){
        topMotor.setVelocity(targetVelocity * topMotorMultiplier);
        bottomMotor.setVelocity(targetVelocity * bottomMotorMultiplier);

    }

    public double getFlywheelVelocity(){
        double sum = (topMotor.getVelocity() * topMotorMultiplier) + (bottomMotor.getVelocity() * bottomMotorMultiplier);
        return sum/2;
    }

    public void setHoodPosition(double hoodPosition){
        hood.setPosition(hoodPosition);
    }
}
