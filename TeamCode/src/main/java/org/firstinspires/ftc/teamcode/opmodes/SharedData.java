package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;

public class SharedData {
    // Static variable persists between OpMode runs
    public static Pose lastKnownPose = null;

    // Flag to know if we're coming from autonomous
    public static boolean hasAutonomousRun = false;
}