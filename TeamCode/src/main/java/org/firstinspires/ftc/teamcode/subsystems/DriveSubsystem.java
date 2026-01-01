package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotor fl, fr, bl, br;

    public DriveSubsystem(HardwareMap hw) {
        fl = hw.get(DcMotor.class, Constants.M_FL);
        fr = hw.get(DcMotor.class, Constants.M_FR);
        bl = hw.get(DcMotor.class, Constants.M_BL);
        br = hw.get(DcMotor.class, Constants.M_BR);

        // typical orientation; flip if your bot drives backward
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** Robot-centric mecanum mix. Inputs in [-1,1]. */
    public void drive(double fwd, double strafe, double rot, double scale) {
        double denom = Math.max(Math.abs(fwd) + Math.abs(rot), 0.3);
        double flP = (fwd + rot - strafe) / denom;
        double blP = (fwd + rot + strafe) / denom;
        double frP = (fwd - rot + strafe) / denom;
        double brP = (fwd - rot - strafe) / denom;

        fl.setPower(flP * scale);
        bl.setPower(blP * scale);
        fr.setPower(frP * scale);
        br.setPower(brP * scale);
    }
}
