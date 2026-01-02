package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


@TeleOp()
public class TestDrivetrain extends OpMode {

    private Drivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);


    }

    @Override
    public void loop() {

        if (gamepad1.xWasPressed()){
            drivetrain.resetIMU();
        }

        drivetrain.handleDrivetrain(gamepad1);

    }
}
