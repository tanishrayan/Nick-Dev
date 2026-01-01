package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;
import java.util.List;

public class Limelight {
    Limelight3A limelight;
    LLResult result;

    double tX;
    double tY;
    double distance;
    double CAM_DEG = 27.3;//25.85
    double CAM_H = 11.5;
    double TARGET_H = 29.5;
    int id = -1;
    int motifID = -1;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(0); // set this to whatver pipeline you config
        limelight.start();
    }


    public void stopLimeLight() {
        limelight.stop();
        limelight.shutdown();
    }

    public void getResult() {
        result = limelight.getLatestResult();
    }

    public boolean isResultValid() {
        return result.isValid();
    }
    public double getTX() {
        if (result != null) {
            tX = result.getTx();
        } else {
            tX = 0;
        }
        return tX;
    }

    public double getTY() {
        if (result != null) {
            tY = result.getTy();
        } else {
            tY = 0;
        }
        return tY;
    }
}