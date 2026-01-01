package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo leftServo, rightServo;
    private double turretMaxAngleRight = 0; // placeholder value
    private double turretMinAngleLeft = 0; //placeholder value

    // program both servos to have 0 position while the turret is facing straight forward
    // then spin the turret to left max and right max
    // we need to know how much ticks or wtv unit the axon uses per degree of rotation
    // so divide the range by however many degrees rotated and js do 180 degrees or smt so its accurate


    public Turret(HardwareMap hardwareMap){
        // left and right based off the bot facing forward perspective
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    public void aim(double botAimErrorDegree){

    }

    public double wrap(double angle){
        if (angle > turretMaxAngleRight){
            angle -= 0; //placeholder value
        }
        if (angle < turretMinAngleLeft){
            angle += 0; //placeholder value
        }

        return angle;
    }

    public void setTurretPosition(double angleRelativeToRobot){

    }



}
