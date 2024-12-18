package org.firstinspires.ftc.teamcode.Core.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Core.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous
public class testAuto extends OpMode {
    private Claw claw;
    private Slide slide;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(8,82.5, Math.toRadians(270)); //270 = South

    private final Pose scorePose = new Pose(14, 119, Math.toRadians(315)); //315 = -45 or towards basket

    private final Pose step2ControlPoint = new Pose(5, 80, Math.toRadians(0));

    private final Pose step2FinalPoint = new Pose(34, 81, Math.toRadians(0));

    private final Pose step2to3Point = new Pose(35, 60, Math.toRadians(0));

    private final Pose step3ControlPoint = new Pose(1, 80, Math.toRadians(180));
    private final Pose step3FinalPoint = new Pose(35, 110, Math.toRadians(180));

    private Path scorePreload, step2;

    private PathChain step3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        step2 = new Path(new BezierCurve(new Point(scorePose), new Point(step2ControlPoint), new Point(step2FinalPoint)));
        step2.setLinearHeadingInterpolation(scorePose.getHeading(), step2FinalPoint.getHeading());

        step3 = follower.pathBuilder()
                .addPath(
                new BezierLine(new Point(step2FinalPoint), new Point(step2to3Point)))
                .setLinearHeadingInterpolation(step2FinalPoint.getHeading(), step2to3Point.getHeading())
                .addPath(
                new BezierCurve(new Point(step2to3Point), new Point(step3ControlPoint), new Point(step3FinalPoint)))
                .setConstantHeadingInterpolation(step3FinalPoint.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        Robot robot = Robot.getInstance();
        robot.initialize(hardwareMap, telemetry);

        follower.setStartingPose(startPose);

        buildPaths();

        //claw.closeClaw();
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);

                break;
            case 1:

                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {

                    follower.followPath(step2);
                    setPathState(-1);

                }


                break;
            case 2:
                if(follower.getPose().getX() > (scorePose.getX() -1) && follower.getPose().getY() > (scorePose.getY() - 1)) {

                    follower.followPath(step3);
                    setPathState(3);
                }
                break;
            case 3:

                break;
        }
    }
}
