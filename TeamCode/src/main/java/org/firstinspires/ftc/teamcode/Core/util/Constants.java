package org.firstinspires.ftc.teamcode.Core.util;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Constants {

    /* ----------- NAMES ------------- */





    /* ----------- TELEOP ------------- */

    /* ----------- AUTO ------------- */

    /* --- Pinpoint Fix (remove eventaully) --- */

    public static double forwardOffsetMM_X = -120.000014;
    public static double forwardOffsetMM_Y = -135.999982;
    public static double strafeOffsetMM_X = 120.000014;
    public static double strafeOffsetMM_Y = -135.999982;

    /* --- Pinpoint --- */

    public static double xOffsetMM = -120.000014;
    public static double yOffsetMM = -135.999982;
    public static double xOffsetInch = (-120.000014 / 25.4); // 4.72441
    public static double yOffsetInch = (-135.999982 / 25.4); //5.35433
    public static double maxPower = 1;
    public static Pose sampleStartPosition = new Pose(7.5, 112.5, Math.toRadians(90));
    public static Pose specimenStartPosition = new Pose(7.5, 72, Math.toRadians(0));

    /* ----------- JOINT ----------- */

    public static double jointSpecimenPlacePosition = 1000;
    public static double jointSamplePickupPosition = 1000;

    /* ----------- SLIDE ----------- */

    public static double slideMaxPosition = 3000;
    public static double slideMinPosition = 100;
    public static double slideHighBasketPosition = 2800;
    public static double slideLowBasketPosition = 2000;

    /* ----------- SERVO ------------- */

    /* ----------- CLAW ----------- */
    public static double clawOpenPosition = 0.55;
    public static double clawClosedPosition = 0.85;

    /* ----------- WRIST ----------- */
    public static double wristStartingPosition = 0.85;

}
