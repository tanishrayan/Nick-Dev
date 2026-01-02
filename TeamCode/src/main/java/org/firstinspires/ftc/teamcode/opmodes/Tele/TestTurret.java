package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "testing turret servo positions")
public class TestTurret extends OpMode {
    private Servo servo2, servo1;

    private double servo1Position;
    private double servo2Position;

    private double smallIncrement = 0.05;
    private double increment = 0.1;


    /*
    min:
    exactly forward:
    exactly backward:
    max:

     */



    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "rightServo");
        servo2 = hardwareMap.get(Servo.class, "leftServo");

        telemetry.addData("servo 1 position", servo1.getPosition());
        telemetry.addData("servo 2 position", servo2.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.squareWasPressed()){
            servo1.setPosition(servo1Position - smallIncrement);
            servo2.setPosition(servo2Position - smallIncrement);

            servo1Position = servo1.getPosition();
            servo2Position = servo2.getPosition();
        } else if (gamepad1.circleWasPressed()) {
            servo1.setPosition(servo1Position + smallIncrement);
            servo2.setPosition(servo2Position + smallIncrement);

            servo1Position = servo1.getPosition();
            servo2Position = servo2.getPosition();


        }

        if (gamepad1.dpadLeftWasPressed()){
            servo1.setPosition(servo1Position - increment);
            servo2.setPosition(servo2Position - increment);

            servo1Position = servo1.getPosition();
            servo2Position = servo2.getPosition();
        } else if (gamepad1.dpadRightWasPressed()) {
            servo1.setPosition(servo1Position + increment);
            servo2.setPosition(servo2Position + increment);

            servo1Position = servo1.getPosition();
            servo2Position = servo2.getPosition();
        }


        servo1Position = servo1.getPosition();
        servo2Position = servo2.getPosition();


        telemetry.addData("servo 1 position", servo1.getPosition());
        telemetry.addData("servo 2 position", servo2.getPosition());
        telemetry.update();


    }

}
