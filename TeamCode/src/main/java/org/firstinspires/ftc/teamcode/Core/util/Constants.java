package org.firstinspires.ftc.teamcode.Core.util;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Constants {

    /* ----------- NAMES ------------- */





    /* ----------- TELEOP ------------- */

    /* ----------- AUTO ------------- */

    /* --- Pinpoint Fix (remove eventaully) --- */

    public static double xOffsetMM_X = 120.000014;
    public static double xOffsetMM_Y = 135.999982;
    public static double yOffsetMM_X = 120.000014;
    public static double yOffsetMM_Y = 135.999982;

    /* --- Pinpoint --- */

    public static double xOffsetMM = 120.000014;
    public static double yOffsetMM = 135.999982;
    public static double xOffsetInch = 4.72441; // 4.72441
    public static double yOffsetInch = 5.35433; //5.35433
    public static double maxPower = 0.5;
    public static Pose sampleStartPosition = new Pose(0, 0, 0);
    public static Pose specimenStartPosition = new Pose(0, 0, 0);

    /* ----------- SERVO ------------- */

    public static double clawOpenPosition = 0.55;
    public static double clawClosedPosition = 0.85;
    public static double wristStartingPosition = 0.85;

}
