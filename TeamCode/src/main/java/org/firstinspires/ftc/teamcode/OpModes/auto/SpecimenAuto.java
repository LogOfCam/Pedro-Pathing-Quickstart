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

import org.firstinspires.ftc.teamcode.Core.Commands.PedroCommands.PathChainCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.PedroCommands.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetBasket;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetWrist;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Specimen Auto", group = "specimen")
public class SpecimenAuto extends LinearOpMode {
    private Robot robot;

    public static Path[] paths = new Path[16];
    private final Pose placeInitial = new Pose(39, 62, Math.toRadians(180));
    private final Pose backup = new Pose(30, 62, Math.toRadians(180));
    private final Pose curveToPush = new Pose(56, 23, Math.toRadians(180));
    private final Point curve1 = new Point(19, 8);
    private final Point curve2 = new Point(60, 44);
    private final Pose pushSample1 = new Pose(26, 23, Math.toRadians(180));
    private final Pose lineupSample2 = new Pose(54, 13, Math.toRadians(180));
    private final Point lineup1 = new Point(64, 34);
    private final Pose pushSample2 = new Pose(26, 15, Math.toRadians(180));
    private final Pose lineupSample3 = new Pose(54, 8,Math.toRadians(180));
    private final Point lineup2 = new Point(64, 22);
    private final Pose pushSample3 = new Pose(18, 8, Math.toRadians(180));
    private final Pose pickupPosition = new Pose(22, 46, Math.toRadians(225));
    private final Point pickup1 = new Point(36, 27);
    private final Pose placePosition = new Pose(36, 62, Math.toRadians(200));
    private final Pose backupForPickup = new Pose(30, 56, Math.toRadians(225));

    private final Pose placeSpecimen2 = new Pose(40, 64, Math.toRadians(180));
    private final Pose placeSpecimen3 = new Pose(40,66, Math.toRadians(180));
    private final Pose placeSpecimen4 = new Pose(40,68, Math.toRadians(180));
    private final Pose placeSpecimen5 = new Pose(40,70, Math.toRadians(180));
    public void buildPaths() {
        paths[0] = buildLine(Constants.specimenStartPosition, placeInitial, HeadingInterpolation.CONSTANT);
        paths[1] = buildLine(placeInitial,backup, HeadingInterpolation.CONSTANT);

        paths[2] = buildCurve(backup,curve1,curve2,curveToPush, HeadingInterpolation.CONSTANT);
        paths[3] = buildLine(curveToPush,pushSample1,HeadingInterpolation.CONSTANT);
        paths[4] = buildCurve(pushSample1,lineup1,lineupSample2, HeadingInterpolation.CONSTANT);
        paths[5] = buildLine(lineupSample2, pushSample2, HeadingInterpolation.LINEAR);
        paths[6] = buildLine(pushSample2, pickupPosition, HeadingInterpolation.LINEAR);

        //Place #2

        paths[7] = buildLine(pickupPosition, placePosition, HeadingInterpolation.LINEAR);
        paths[8] = buildLine(placePosition, backupForPickup, HeadingInterpolation.LINEAR);
        paths[9] = buildLine(backupForPickup, pickupPosition, HeadingInterpolation.CONSTANT);

        // Place #3
        paths[10] = buildLine(pickupPosition, placePosition, HeadingInterpolation.LINEAR);
        paths[11] = buildLine(placePosition, backupForPickup, HeadingInterpolation.LINEAR);
        paths[12] = buildLine(backupForPickup, pickupPosition, HeadingInterpolation.CONSTANT);

        // Place #4
        paths[13] = buildLine(pickupPosition, placePosition, HeadingInterpolation.LINEAR);
        paths[14] = buildLine(placePosition, backupForPickup, HeadingInterpolation.LINEAR);
        paths[15] = buildLine(backupForPickup, pickupPosition, HeadingInterpolation.CONSTANT);
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

        long clawWaitCommand = 500;
        long jointWaitCommand = 300;

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetSlide(robot.slide,1000).andThen(
                                        new ParallelCommandGroup(
                                                new PathCommand(paths[0]),
                                                new SetBasket(robot.basket, Constants.basketMaxPosition),
                                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition),
                                                new SetSlide(robot.slide, Constants.slideMiddlePosition),
                                                new SetWrist(robot.wrist, Constants.wristPlacePosition)
                                        )
                                )
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(10),
                        new ParallelCommandGroup(
                                new PathCommand(paths[1]),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new SetJoint(robot.joint, Constants.jointStraightUp)
                                ),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),
                        new ParallelCommandGroup(
                                new PathCommand(paths[2]),
                                new SetSlide(robot.slide, Constants.slideMinPosition),
                                new SetBasket(robot.basket, Constants.basketStartingPosition)
                        ),




                        new PathChainCommand(paths[3], paths[4], paths[5], paths[6]),





                        // TODO: Pickup Specimen 2
                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition),
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(clawWaitCommand),
                        new ParallelCommandGroup(
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePositionTop),
                                new SequentialCommandGroup(
                                        new WaitCommand(jointWaitCommand),
                                        new PathCommand(paths[7])
                                ),
                                new SetWrist(robot.wrist, Constants.wristAlmostPlacePosition)
                        ),
                        new SetWrist(robot.wrist,Constants.wristPlacePosition),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new PathCommand(paths[8]),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetClaw(robot.claw,Constants.clawOpenPosition)
                                )
                        ),
                        new ParallelCommandGroup(
                                new PathCommand(paths[9]),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition)
                                ),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),




                        // TODO: Pickup Specimen 3
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(clawWaitCommand),
                        new ParallelCommandGroup(
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePositionTop),
                                new SequentialCommandGroup(
                                        new WaitCommand(jointWaitCommand),
                                        new PathCommand(paths[7])
                                ),
                                new SetWrist(robot.wrist, Constants.wristAlmostPlacePosition)
                        ),
                        new SetWrist(robot.wrist,Constants.wristPlacePosition),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new PathCommand(paths[8]),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetClaw(robot.claw,Constants.clawOpenPosition)
                                )
                        ),
                        new ParallelCommandGroup(
                                new PathCommand(paths[9]),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition)
                                ),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),




                        // TODO: Pickup Specimen 4
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(clawWaitCommand),
                        new ParallelCommandGroup(
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePositionTop),
                                new SequentialCommandGroup(
                                        new WaitCommand(jointWaitCommand),
                                        new PathCommand(paths[7])
                                ),
                                new SetWrist(robot.wrist, Constants.wristAlmostPlacePosition)
                        ),
                        new SetWrist(robot.wrist,Constants.wristPlacePosition),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new PathCommand(paths[8]),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetClaw(robot.claw,Constants.clawOpenPosition)
                                )
                        ),
                        new ParallelCommandGroup(
                                new PathCommand(paths[9]),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition)
                                ),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
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
        telemetry.addData("SlideTarget", robot.slide.getActualTargetPosition());
        telemetry.addData("SlideCurrent", robot.slide.getCurrentPosition());
        telemetry.addData("JointTarget", robot.joint.getTargetPosition());
        telemetry.addData("JointCurrent", robot.joint.getCurrentPosition());
        telemetry.addData("Claw", robot.claw.getPosition());
        telemetry.addData("Power", robot.slide.getPower());
        telemetry.update();
    }
}
