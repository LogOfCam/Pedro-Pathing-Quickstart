package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildLine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.drive.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class Specimanauto1 extends LinearOpMode {

    private Robot robot;

    public static Path[] paths = new Path[12];

    //TODO: Correct naming for all of these below.
    private final Pose specimanStart = new Pose(7.5, 62, Math.toRadians(180)); //TODO: spelling
    private final Pose placeInitel = new Pose(40.5, 62, Math.toRadians(180)); //TODO: spelling
    private final Pose backup = new Pose(38, 62, Math.toRadians(180));
    private final Pose curveToPush = new Pose(60, 26, Math.toRadians(180));
    private final Point curve1 = new Point(60, 26);
    private final Point curve2 = new Point(66, 55);
    private final Pose pushSample = new Pose(18, 26, Math.toRadians(180));

    //TODO: Upsample? Should it be capitalized?
    private final Pose lineUpsample2 = new Pose(60, 13, Math.toRadians(180));
    private final Point lineup1 = new Point(64, 34);
    private final Pose pushSample2 = new Pose(18, 15, Math.toRadians(180));

    //TODO: Why is lineUpsample3 not used?
    private final Pose lineUpsample3 = new Pose(60, 8, Math.toRadians(180));
    private final Point lineup2 = new Point(64, 22);
    private final Pose pushSample3 = new Pose(18, 8, Math.toRadians(180));

    //TODO: Capitalization?
    private final Pose lineuptopickup = new Pose(17, 46, Math.toRadians(225));
    private final Point pickup1 = new Point(57, 15);
    private final Pose placeSpeciman2 = new Pose(40.5, 65, Math.toRadians(180)); //TODO: Spelling
    public void buildPaths() {

        /* TODO: why do you have two start positions? We have one from constants
            and then the on you put in So we're driving from specimenStartPosition
            to specimenStart position? So, do you even need this line? */

        paths[0] = buildLine(Constants.specimenStartPosition, specimanStart, HeadingInterpolation.CONSTANT);

        /* TODO: If you use Constants.specimenStartPosition, what needs replaced?*/

        paths[1] = buildLine(specimanStart, placeInitel , HeadingInterpolation.CONSTANT);

        // Looks good.

        paths[2] = buildLine(placeInitel, backup, HeadingInterpolation.CONSTANT);

        /* TODO: Not sure what is going on here.
            Why is backup to curveToPush a line?
            Shouldn't it be a curve? 3 to 8 need fixed.

            Here's an example of doing a double pont curve:
            paths[X] = buildCurve(lastPosition, point1, point2, endPosition, heading)
            Single point Curve:
            paths[X] = buildCurve(lastPosition, point1, endPosition, heading)
        */

        paths[3] = buildLine(backup, curveToPush , HeadingInterpolation.CONSTANT);
        paths[4] = buildLine(curveToPush, curve1, HeadingInterpolation.CONSTANT);
        paths[5] = buildCurve(curve1, curve2, pushSample, HeadingInterpolation.LINEAR);
        paths[6] = buildLine(curve2, lineUpsample2,  HeadingInterpolation.CONSTANT);
        paths[7] = buildLine(lineUpsample2, lineup1,  HeadingInterpolation.CONSTANT);
        paths[8] = buildLine(lineup1, pushSample2, HeadingInterpolation.CONSTANT);

        paths[9] = buildCurve(pushSample2, lineup2, pushSample3, HeadingInterpolation.LINEAR);
        paths[10] = buildLine(pushSample3, lineuptopickup, HeadingInterpolation.CONSTANT);

        /* TODO: Requires Pose, Pose, not a Point. */

        paths[11] = buildLine(lineuptopickup, pickup1, HeadingInterpolation.CONSTANT);

        /* TODO: This is a Curve again. It requires a Pose, Point, Pose.
            You gave it: point, point, pose. */
        paths[12] = buildCurve(pickup1, placeSpeciman2, placeSpeciman2, HeadingInterpolation.LINEAR);
    }

    @Override
    public void runOpMode() {

        robot = Robot.getInstance();

        robot.initialize(hardwareMap, telemetry);

        CommandScheduler.getInstance().reset();

        buildPaths();

        while(!isStarted()) {


            CommandScheduler.getInstance().run();

            robot.claw.setPosition(Constants.clawClosedPosition);
            robot.wrist.setPosition(Constants.wristStartingPosition);

            robot.slide.setDefaultCommand(CommandScheduler.getInstance());
            robot.joint.setDefaultCommand(CommandScheduler.getInstance());

            updateTelemetry();
        }

        robot.setPose(Constants.specimenStartPosition);

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(

                        //TODO: ADD new WaitCommand between everything

                        new PathCommand(paths[0]),
                        new PathCommand(paths[1]),
                        new PathCommand(paths[2]),
                        new PathCommand(paths[3]),
                        new PathCommand(paths[4]),
                        new PathCommand(paths[5]),
                        new PathCommand(paths[6]),
                        new PathCommand(paths[7]),
                        new PathCommand(paths[8]),
                        new PathCommand(paths[9]),
                        new PathCommand(paths[10]),
                        new PathCommand(paths[11]),
                        new PathCommand(paths[12]),

                        new ParallelCommandGroup(

                        new SetSlide(robot.slide, Constants.slideMiddlePosition),
                        new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition)

                        )

                )


        );

        while(opModeIsActive() && !isStopRequested()) {

            CommandScheduler.getInstance().run();

            updateTelemetry();
        }

    }

    public void updateTelemetry() {
        telemetry.addData("x", robot.getPose().getX());
        telemetry.addData("y", robot.getPose().getY());
        telemetry.addData("heading", robot.getPose().getHeading());
        telemetry.addData("SlideTarget", robot.slide.getTargetPosition());
        telemetry.addData("SlideCurrent", robot.slide.getCurrentPosition());
        telemetry.addData("Claw", robot.claw.getPosition());
        telemetry.addData("Power", robot.slide.getPower());
        telemetry.update();
    }

    /* HINT: backUp will be starting, curve1 and 2,
            and then curveToPush as end position, and heading
            */
}
