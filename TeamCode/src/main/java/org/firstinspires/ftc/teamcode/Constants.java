package org.firstinspires.ftc.teamcode;

public final class Constants {
    // --- Hardware names (match your RC config) ---
    public static final String M_FL = "frontLeft";
    public static final String M_FR = "frontRight";
    public static final String M_BL = "backLeft";
    public static final String M_BR = "backRight";

    public static final String LIMELIGHT_NAME = "limelight";
    // --- Drive ---
    public static final double DRIVE_SCALE = 0.75;

    // VISION CONSTANTS
    public static final class Vision {
        // Limelight network table name


        // Shooting lookup tables (distance → values)
        // Hood angles by distance (inches)
        public static final double[][] HOOD_LOOKUP = {
                // {distance, hood_position}
                {24.0, 0.65},  // close shot - higher arc
                {48.0, 0.50},  // medium
                {72.0, 0.35},  // far shot - lower arc
        };

        // Flywheel RPM by distance
        public static final double[][] RPM_LOOKUP = {
                // {distance, rpm}
                {24.0, 3000},  // close - slower
                {48.0, 4000},  // medium
                {72.0, 5200},  // far - faster
        };

        // Limelight mounting (for distance calculation)
        public static final double LIMELIGHT_HEIGHT = 12.0;  // inches off ground
        public static final double LIMELIGHT_ANGLE = 25.0;   // degrees up
        public static final double TARGET_HEIGHT = 36.0;     // goal height
    }
}

