package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "zero servos")
public class ZeroServos extends OpMode {

    private Servo rightServo, leftServo;

    // port 5 and port 0
    @Override
    public void init() {
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");


    }

    @Override
    public void loop() {

        if (gamepad1.squareWasPressed()){
            leftServo.setPosition(0);

        } else if (gamepad1.circleWasPressed()) {
            leftServo.setPosition(1);
        }


        if(gamepad1.dpadLeftWasPressed()){
            rightServo.setPosition(0);
        } else if (gamepad1.dpadRightWasPressed()) {
            rightServo.setPosition(1);
        }

    }
}
