package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestMotor extends OpMode {
    private DcMotor test;
    private boolean running = false;
    private boolean reversed = false;
    private boolean crossPressed = false;
    private boolean circlePressed = false;

    @Override
    public void init() {
        test = hardwareMap.get(DcMotor.class, "test");
    }

    @Override
    public void loop() {
        // Cross (X) button: toggle motor on/off
        if (gamepad1.cross && !crossPressed) {
            running = !running;
            crossPressed = true;
        } else if (!gamepad1.cross) {
            crossPressed = false;
        }

        // Circle button: reverse direction
        if (gamepad1.circle && !circlePressed) {
            reversed = !reversed;
            circlePressed = true;
        } else if (!gamepad1.circle) {
            circlePressed = false;
        }

        // Set motor power
        if (running) {
            test.setPower(reversed ? -1.0 : 1.0);
        } else {
            test.setPower(0);
        }

        telemetry.addData("Motor", running ? "RUNNING" : "STOPPED");
        telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");
        telemetry.addData("Power", test.getPower());
        telemetry.update();
    }
}