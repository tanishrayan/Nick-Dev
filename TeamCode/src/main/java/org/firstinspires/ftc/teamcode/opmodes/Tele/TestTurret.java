package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class TestTurret extends OpMode {
    private Servo servo2, servo1;

    private double servo1Position;
    private double servo2Position;

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

    }

}
