package org.firstinspires.ftc.teamcode.Core.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous
public class newAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startingPose = new Pose(7.5,72, Math.toRadians(180)); //270 = South
    private final Pose placeSpecimenPosition1 = new Pose(36, 72, Math.toRadians(180));
    private Path forwards, test;

    public void buildPaths() {
        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(40,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);

        test = new Path(new BezierLine(new Point(startingPose), new Point(placeSpecimenPosition1)));
        test.setConstantHeadingInterpolation(Math.toRadians(180));
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(test);
                follower.followPath(test, true);
                setPathState(-1);
                break;
            case 1:
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        //follower = new Follower(hardwareMap);
        follower.setPose(startingPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        buildPaths();

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
