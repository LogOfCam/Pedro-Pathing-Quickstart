package org.firstinspires.ftc.teamcode.Core.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.ClawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class normalAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
    public ClawSubsystem claw;
    private final Pose startingPose = new Pose(7.5,72, Math.toRadians(180)); //270 = South
    private final Pose placeSpecimenPosition1 = new Pose(36, 72, Math.toRadians(180));
    private final Pose placeSpecimenPosition2 = new Pose(36, 70, Math.toRadians(0));
    private final Pose placeSpecimenPosition3 = new Pose(36, 68, Math.toRadians(0));
    private final Pose placeSpecimenPosition4 = new Pose(36, 66, Math.toRadians(0));
    private final Pose placeSpecimenPosition5 = new Pose(36, 64, Math.toRadians(0));
    private final Pose pickupSamplePosition1 = new Pose(24, 48, Math.toRadians(320));
    private final Pose pickupSamplePosition2 = new Pose(26, 32, Math.toRadians(140));
    private final Pose pickupSamplePosition3 = new Pose(26, 24, Math.toRadians(140));
    private final Pose placeSamplePosition1 = new Pose(26, 40, Math.toRadians(20));
    private final Pose placeSamplePosition2 = new Pose(26, 32, Math.toRadians(20));
    private final Pose placeSamplePosition3 = new Pose(26, 24, Math.toRadians(20));
    private final Pose pickupSpecimenPosition = new Pose(26,24, Math.toRadians(0));
    private final Pose parkSpecimenPosition = new Pose(24, 36, Math.toRadians(30));

    /* These are our Paths and PathChains that we will define in buildPaths() */
//    private Path placeInitalSpecimen;

    private Path test;
    private PathChain placeInitalSpecimen, pickupSample1;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        test = new Path(new BezierLine(new Point(startingPose), new Point(placeSpecimenPosition1)));
        test.setConstantHeadingInterpolation(Math.toRadians(0));

        placeInitalSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startingPose), new Point(placeSpecimenPosition1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        pickupSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(placeSpecimenPosition1), new Point(pickupSamplePosition1)))
                .setLinearHeadingInterpolation(placeSpecimenPosition1.getHeading(), pickupSamplePosition1.getHeading())
                .build();
       }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(test);
                follower.followPath(test, true);
                setPathState(-1);
                break;
            case 1:

               if(follower.getPose().getX() > (placeSpecimenPosition1.getX() - 1) && follower.getPose().getY() > (placeSpecimenPosition1.getY() - 1)) {
                    /* Score Preload */
                    claw.openClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pickupSample1,true);
                    setPathState(2);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        updateTelemetry();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        //follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        buildPaths();

        claw = new ClawSubsystem(hardwareMap);

        // Set the claw to positions for init
        claw.closeClaw();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        updateTelemetry();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void updateTelemetry() {
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
