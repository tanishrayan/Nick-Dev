package org.firstinspires.ftc.teamcode;

// FTC annotations and base classes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Hardware classes
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

// Marks this as a TeleOp program
@TeleOp(name = "Outtake Test", group = "Test")
public class NewShooterTest extends LinearOpMode {

    // Declare flywheel motors
    DcMotor flywheelLeft;
    DcMotor flywheelRight;

    // Declare hood CR servo
    CRServo hood;

    @Override
    public void runOpMode() {

        // Connect code variables to hardware config names
        flywheelLeft  = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        hood          = hardwareMap.get(CRServo.class, "hood");

        /*
         * EASY FIX IF SPIN DIRECTION IS WRONG:
         * Swap FORWARD and REVERSE above.
         * Do NOT change power values.
         */

        // Make sure everything is stopped at init
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
        hood.setPower(0);

        // Show ready message
        telemetry.addLine("Outtake Test Ready");
        telemetry.update();

        // Wait for PLAY
        waitForStart();

        // Runs repeatedly while TeleOp is active
        while (opModeIsActive()) {

            // ---------------- FLYWHEEL CONTROL ----------------

            // If RB is pressed
            if (gamepad1.right_bumper) {

                // Set flywheel speed (change this number to tune)
                flywheelLeft.setPower(-1.0);
                flywheelRight.setPower(1.0);

            } else {

                // Stop flywheels
                flywheelLeft.setPower(0);
                flywheelRight.setPower(0);
            }

            // ---------------- HOOD CONTROL (CR SERVO) ----------------

            // RT pressed → hood moves up
            if (gamepad1.right_trigger > 0.1) {
                hood.setPower(0.6);   // Positive = one direction
            }

            // LT pressed → hood moves down
            else if (gamepad1.left_trigger > 0.1) {
                hood.setPower(-0.6);  // Negative = opposite direction
            }

            // No trigger → stop hood
            else {
                hood.setPower(0);
            }

            // ---------------- TELEMETRY ----------------

            //telemetry.addData("Flywheel Power", //flywheelLeft.getPower());
            telemetry.addData("Hood Power", hood.getPower());
            telemetry.update();
        }
    }
}