package org.firstinspires.ftc.teamcode.Core.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;

@Config
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

    public static double kP = 0.03, kI = 0, kD = 0.000, kF = 0.0025;
    public static double jointThreshold = 25;
    public static double jointStartingPosition = 100;
    public static double jointSpecimenPlacePosition = 1550;//was 1450
    public  static double jointSpecimenPlacePositionTop = 1600;
    public static double jointStraightUp = 2600;
    public static double jointSamplePickupPosition = 4900;
    public static double jointSampleAlmostPickupPosition = 4850;
    public static double jointSpecimenWaitPosition = 4500;
    public static double jointSpecimenPickupPosition = 4900;
    public static double jointSpecimenTempPosition = 2000;
    public static double jointMaxPosition = 5000;
    public static double jointMinPosition = 50;
    public static double jointTransferPosition = 2300;
    public static double jointSideWaysTransforePosition = 2100;
    public static double joint_park_position = 3250;

    /* ----------- SLIDE ----------- */

    public static double slideThreshold = 25;
    public static double slideMaxPosition = 2000;
    public static double slideMinPosition = 66;
    public static double slideSpeciman1Position = 1700;
    public static double slideHighBasketPosition = 1850;
    public static double slideLowBasketPosition = 1300;
    public static double slideMiddlePosition = 1300;
    public static double slideTransferPosition = 450;
    /* ------- SERVO SECTION ------- */

    /* ----------- CLAW ----------- */
    public static double clawOpenPosition = 0.50;
    public static double claw_not_so_open_position = 0.57;
    public static double clawOpenPickupPosition = 0.30;
    public static double clawClosedPosition = 0.67;
    public static double claw_almost_closed_position = 0.668;

    /* ----------- WRIST ----------- */
    public static double wristStartingPosition = 0.85;
    public static double wristPickupPosition = 0.40;
    public static double wristTransferPosition = 0.10;
    public static double wristPlacePosition = 0.4;
    public static double wristAlmostPlacePosition = 0.55;
    public static double wrist_speciman_fix_position = 0.65;
    /* ----------- BASKET ----------- */
    public static double basketStartingPosition = 0.30;
    public static double basketPlacePosition = 0.70;
    public static double basketMaxPosition = 1;
}
