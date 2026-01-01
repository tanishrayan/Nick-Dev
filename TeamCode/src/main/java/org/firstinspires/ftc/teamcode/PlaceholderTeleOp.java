package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Placeholder TeleOp", group="Z")
public class PlaceholderTeleOp extends LinearOpMode {
    private DcMotor fl, fr, bl, br;

    @Override public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        // typical mecanum: reverse left side (adjust if needed)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;   // forward
            double x = -gamepad1.left_stick_x;   // strafe
            double r = -gamepad1.right_stick_x;  // rotate

            double denom = Math.max(Math.abs(y) + Math.abs(r), 0.3);
            double flP = (y + r - x) / denom;
            double blP = (y + r + x) / denom;
            double frP = (y - r + x) / denom;
            double brP = (y - r - x) / denom;

            fl.setPower(flP * 0.5);
            bl.setPower(blP * 0.5);
            fr.setPower(frP * 0.5);
            br.setPower(brP * 0.5);
        }
    }
}
