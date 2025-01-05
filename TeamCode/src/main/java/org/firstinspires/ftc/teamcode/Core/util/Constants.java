package org.firstinspires.ftc.teamcode.Core.util;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;

public class Constants {

    /* ----------- NAMES ----------- */

    //TODO: ADD STUFF
    public static String pinpointName = "pinpoint";

    /* ----------- TELEOP ----------- */

    public static double driveFastMultiplier = 0.65;
    public static double driveSlowMultiplier = 0.25;

    //TODO: ADD STUFF

    /* ----------- AUTO ----------- */


    /* ----------- Pinpoint ----------- */

    public static double xOffsetMM = -123; //was 120.000014
    public static double yOffsetMM = -131.3; //was -135.999982 or -125.3
    public static double xOffsetInch = (xOffsetMM / 25.4); //-4.72441
    public static double yOffsetInch = (yOffsetMM / 25.4); //-5.35433
    public static double maxPower = 1;

    /* ----------- PEDRO POSITIONS ----------- */

    public static Pose sampleStartPosition = new Pose(7.5, 112.5, Math.toRadians(270));
    public static Pose specimenStartPosition = new Pose(7.5, 62, Math.toRadians(180));
    /* ------- MOTOR SECTION ------- */

    /* ----------- JOINT ----------- */
    public static double jointThreshold = 10;
    public static double jointStartingPosition = 100;
    public static double jointSpecimenPlacePosition = 1400;
    public static double jointStraightUp = 2600;
    public static double jointSamplePickupPosition = 4900;
    public static double jointMaxPosition = 5000;
    public static double jointMinPosition = 50;
    public static double jointTransferPosition = 2300;

    /* ----------- SLIDE ----------- */

    public static double slideThreshold = 10;
    public static double slideMaxPosition = 3200;
    public static double slideMinPosition = 100;
    public static double slideHighBasketPosition = 2800;
    public static double slideLowBasketPosition = 2000;
    public static double slideMiddlePosition = 2000;
    public static double slideTransferPosition = 800;
    /* ------- SERVO SECTION ------- */

    /* ----------- CLAW ----------- */
    public static double clawOpenPosition = 0.55;
    public static double clawClosedPosition = 0.85;

    /* ----------- WRIST ----------- */
    public static double wristStartingPosition = 0.85;
    public static double wristPickupPosition = 0.40;
    public static double wristTransferPosition = 0.10;
    public static double wristPlacePosition = 0.4;

    /* ----------- BASKET ----------- */
    public static double basketStartingPosition = 0.30;
    public static double basketPlacePosition = 0.70;

}
