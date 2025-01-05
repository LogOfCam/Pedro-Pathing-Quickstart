package org.firstinspires.ftc.teamcode.OpModes.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildLine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.PedroCommands.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetWrist;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Specimen Auto", group = "specimen")
public class specimenAuto extends LinearOpMode {

    private Robot robot;

    public static Path[] paths = new Path[16];
    private final Pose placeInitail = new Pose(39, 62, Math.toRadians(180));
    private final Pose backup = new Pose(38, 62, Math.toRadians(180));
    private final Pose curveToPush = new Pose(54, 26, Math.toRadians(180));
    private final Point curve1 = new Point(4, 14);
    private final Point curve2 = new Point(66, 53);
    private final Pose pushSample1 = new Pose(18, 26, Math.toRadians(180));
    private final Pose lineupSample2 = new Pose(54, 13, Math.toRadians(180));
    private final Point lineup1 = new Point(64, 34);
    private final Pose pushSample2 = new Pose(18, 15, Math.toRadians(180));
    private final Pose lineupSample3 = new Pose(54, 8,Math.toRadians(180));
    private final Point lineup2 = new Point(64, 22);
    private final Pose pushSample3 = new Pose(18, 8, Math.toRadians(180));
    private final Pose lineuptoPickup = new Pose(17, 46, Math.toRadians(225));
    private final Point pickup1 = new Point(36, 27);
    private final Pose placeSpeciman2 = new Pose(40, 64, Math.toRadians(180));
    private final Pose placeSpeciman3 = new Pose(40,66, Math.toRadians(180));
    private final Pose placeSpeciman4 = new Pose(40,68, Math.toRadians(180));
    private final Pose placeSpeciman5 = new Pose(40,70, Math.toRadians(180));
    public void buildPaths() {
        paths[0] = buildLine(Constants.specimenStartPosition,placeInitail, HeadingInterpolation.CONSTANT);
        paths[1] = buildLine(placeInitail ,backup, HeadingInterpolation.CONSTANT);

        paths[2] = buildCurve(backup,curve1, curve2 ,curveToPush, HeadingInterpolation.CONSTANT);
        paths[3] = buildLine(curveToPush, pushSample1,HeadingInterpolation.CONSTANT);
        paths[4] = buildCurve(pushSample1,lineup1, lineupSample2,HeadingInterpolation.CONSTANT);
        paths[5] = buildLine(lineupSample2,pushSample2, HeadingInterpolation.CONSTANT);
        paths[6] = buildCurve(pushSample2,lineup2, lineupSample3,HeadingInterpolation.CONSTANT);
        paths[7] = buildLine(lineupSample3, pushSample3, HeadingInterpolation.CONSTANT);

        paths[8] = buildCurve(pushSample3,pickup1,lineuptoPickup, HeadingInterpolation.LINEAR);

        paths[9] = buildLine(lineuptoPickup, placeSpeciman2, HeadingInterpolation.LINEAR);
        paths[10] = buildLine(placeSpeciman2, lineuptoPickup, HeadingInterpolation.LINEAR);
        paths[11] = buildLine(lineuptoPickup, placeSpeciman3,HeadingInterpolation.LINEAR);
        paths[12] = buildLine(placeSpeciman3,lineuptoPickup,HeadingInterpolation.LINEAR);
        paths[13] = buildLine(lineuptoPickup,placeSpeciman4,HeadingInterpolation.LINEAR);
        paths[14] = buildLine(placeSpeciman4,lineuptoPickup,HeadingInterpolation.LINEAR);
        paths[15] = buildLine(lineuptoPickup,placeSpeciman5,HeadingInterpolation.LINEAR);
    }

    @Override
    public void runOpMode() {
        robot = Robot.getInstance();

        robot.initialize(hardwareMap, telemetry);

        buildPaths();

        while(!isStarted()) {


            CommandScheduler.getInstance().run();

            robot.claw.setPosition(Constants.clawClosedPosition);
            robot.wrist.setPosition(Constants.wristStartingPosition);
            updateTelemetry();
        }

        robot.setPose(Constants.specimenStartPosition);

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(


                        new ParallelCommandGroup(
                                new SetSlide(robot.slide, 1000).andThen(
                                        new ParallelCommandGroup(
                                                new PathCommand(paths[0]),
                                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition),
                                                new SetSlide(robot.slide, Constants.slideMiddlePosition),
                                                new SetWrist(robot.wrist, Constants.wristPlacePosition)
                                        )
                                )
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(10),
                        new PathCommand(paths[1]),
                        new PathCommand(paths[2]),
                        new PathCommand(paths[3]),
                        new PathCommand(paths[4]),
                        new PathCommand(paths[5]),
                        new PathCommand(paths[6]),
                        new PathCommand(paths[7]),
                        new PathCommand(paths[8])
//                        new WaitCommand(10),
//                        new PathCommand(paths[9]),
//                        new WaitCommand(10),
//                        new PathCommand(paths[10]),
//                        new WaitCommand(10),
//                        new PathCommand(paths[11]),
//                        new WaitCommand(10),
//                        new PathCommand(paths[12]),
//                        new WaitCommand(10),
//                        new PathCommand(paths[13]),
//                        new WaitCommand(10),
//                        new PathCommand(paths[14]),
//                        new WaitCommand(10),
//                        new PathCommand(paths[15]),
//                        new WaitCommand(10)

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
        telemetry.addData("SlideTarget", robot.slide.getActualTargetPosition());
        telemetry.addData("SlideCurrent", robot.slide.getCurrentPosition());
        telemetry.addData("JointTarget", robot.slide.getActualTargetPosition());
        telemetry.addData("JointCurrent", robot.slide.getCurrentPosition());
        telemetry.addData("Claw", robot.claw.getPosition());
        telemetry.addData("Power", robot.slide.getPower());
        telemetry.update();
    }
}
