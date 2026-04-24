package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * DRIVETRAIN MOTOR TEST
 *
 * Spins one motor at a time at low power so you can confirm:
 *   1. Each motor is plugged into the correct port
 *   2. Each motor spins the correct direction
 *
 * GAMEPAD 1:
 *   Dpad UP    — test frontLeft
 *   Dpad DOWN  — test backLeft
 *   Dpad LEFT  — test frontRight
 *   Dpad RIGHT — test backRight
 *
 * Hold the button to spin that motor. Release to stop.
 * Watch which wheel spins and which direction.
 *
 * EXPECTED for a correctly wired mecanum:
 *   All four wheels should spin FORWARD (same direction)
 *   when each motor runs at positive power.
 *
 * If a wheel spins backward when tested — note it and
 * flip that motor's direction in Drivetrain.java.
 */
@TeleOp(name = "TEST: Drivetrain Motors", group = "testing")
public class DrivetrainMotorTest extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private static final double TEST_POWER = 0.3; // low power for safety

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        // NO direction set here — testing raw motor output
        // so we can see what needs to be flipped
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Drivetrain Motor Test ready.");
        telemetry.addLine("Hold a dpad button to spin that motor.");
        telemetry.addLine("DPAD UP=frontLeft  DOWN=backLeft");
        telemetry.addLine("DPAD LEFT=frontRight  RIGHT=backRight");
        telemetry.update();
    }

    @Override
    public void loop() {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
        String active = "NONE — hold a dpad button";

        if (gamepad1.dpad_up) {
            flPower = TEST_POWER;
            active  = "frontLeft";
        } else if (gamepad1.dpad_down) {
            blPower = TEST_POWER;
            active  = "backLeft";
        } else if (gamepad1.dpad_left) {
            frPower = TEST_POWER;
            active  = "frontRight";
        } else if (gamepad1.dpad_right) {
            brPower = TEST_POWER;
            active  = "backRight";
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

        telemetry.addLine("=== ACTIVE MOTOR ===");
        telemetry.addData("Testing", active);
        telemetry.addLine();
        telemetry.addLine("=== MOTOR POWERS ===");
        telemetry.addData("frontLeft",  "%.2f  (encoder: %d)", flPower, frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", "%.2f  (encoder: %d)", frPower, frontRight.getCurrentPosition());
        telemetry.addData("backLeft",   "%.2f  (encoder: %d)", blPower, backLeft.getCurrentPosition());
        telemetry.addData("backRight",  "%.2f  (encoder: %d)", brPower, backRight.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("=== WHAT TO CHECK ===");
        telemetry.addLine("Each wheel should spin FORWARD at +power");
        telemetry.addLine("If backward -> flip that motor in Drivetrain.java");
        telemetry.addLine("If wrong wheel spins -> motors plugged into wrong ports");
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}