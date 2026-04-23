package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel PIDF Tuner", group = "Testing")
public class FlywheelPIDFTuning extends OpMode {
    public DcMotorEx topMotor;
    public DcMotorEx bottomMotor;

    public double highVelocity = 2000;
    public double lowVelocity = 1000;

    double curTargetVelocity = highVelocity;

    double F = 17.00;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");

        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // get all our gamepad commands
        // set target velocity
        // update telemetry

        // Toggle between high and low velocity
        if (gamepad1.y) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        // Cycle through step sizes
        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F
        if (gamepad1.dpad_left) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpad_right) {
            F += stepSizes[stepIndex];
        }

        // Adjust P
        if (gamepad1.dpad_up) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down) {
            P -= stepSizes[stepIndex];
        }

        // Set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set velocity
        topMotor.setVelocity(curTargetVelocity);
        bottomMotor.setVelocity(curTargetVelocity);

        double topVelocity = topMotor.getVelocity();
        double bottomVelocity = bottomMotor.getVelocity();
        double avgVelocity = (Math.abs(topVelocity) + Math.abs(bottomVelocity)) / 2.0;
        double error = curTargetVelocity - avgVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Top Velocity", "%.2f", topVelocity);
        telemetry.addData("Bottom Velocity", "%.2f", bottomVelocity);
        telemetry.addData("Avg Velocity", "%.2f", avgVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}