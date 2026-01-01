package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Test (Triggers+Bumpers)", group = "Test")
public class SlideTest extends LinearOpMode {

    private DcMotor slideLeft, slideRight;

    // start a bit higher; you can tweak during the run with d-pad
    private double testPower = 1.0;

    @Override
    public void runOpMode() {
        // Map motors (names must match your Robot Config)
        slideLeft  = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        // Recommended basic setup
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // You can use encoders or not; for raw power, WITHOUT is fine
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Your old code reversed the right slide — keep that if needed
        slideRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Slide Test Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("  RIGHT motor: RT = up, RB = down");
        telemetry.addLine("  LEFT  motor:  LT = up, LB = down");
        telemetry.addLine("  D-pad Up/Down = power +/- 0.05");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Tune power live
            //if (gamepad1.dpad_up)    testPower = Math.min(1.0, testPower + 0.05);
            //if (gamepad1.dpad_down)  testPower = Math.max(0.05, testPower - 0.05);

            // --- RIGHT SLIDE ---
            double rightPower = 0.0;
            if (gamepad1.right_trigger > 0.1) rightPower =  testPower;   // up
            if (gamepad1.right_bumper)        rightPower = -testPower;   // down
            slideRight.setPower(rightPower);

            // --- LEFT SLIDE ---
            double leftPower = 0.0;
            if (gamepad1.left_trigger > 0.1)  leftPower  =  testPower;   // up
            if (gamepad1.left_bumper)         leftPower  = -testPower;   // down
            slideLeft.setPower(leftPower);

            telemetry.addData("Power (adjust d-pad)", "%.2f", testPower);
            telemetry.addData("Right trig/bumper", "%.2f / %s", gamepad1.right_trigger, gamepad1.right_bumper);
            telemetry.addData("Left  trig/bumper", "%.2f / %s", gamepad1.left_trigger,  gamepad1.left_bumper);
            telemetry.addData("Pwr R/L", "%.2f / %.2f", rightPower, leftPower);
            telemetry.addData("Ticks R/L", "%d / %d", slideRight.getCurrentPosition(), slideLeft.getCurrentPosition());
            telemetry.update();

            // small loop delay is fine
            sleep(10);
        }

        // Stop motors when exiting
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }
}
