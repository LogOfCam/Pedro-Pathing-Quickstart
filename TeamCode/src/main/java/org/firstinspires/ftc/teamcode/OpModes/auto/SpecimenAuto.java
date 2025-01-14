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
    private final Pose placeInitial = new Pose(39, 64, Math.toRadians(180));
    private final Pose backup = new Pose(30, 62, Math.toRadians(180));
    private final Pose curveToPush = new Pose(53, 23, Math.toRadians(180));
    private final Point curve1 = new Point(19, 8);
    private final Point curve2 = new Point(56, 44); // was 60, 44
    private final Pose pushSample1 = new Pose(26, 23, Math.toRadians(180));
    private final Pose lineupSample2 = new Pose(54, 13, Math.toRadians(180));
    private final Point lineup1 = new Point(60, 27);
    private final Pose pushSample2 = new Pose(26, 15, Math.toRadians(180));
    private final Pose pickupPosition = new Pose(22, 46, Math.toRadians(225));
    private final Pose placePosition = new Pose(36, 62, Math.toRadians(200));
    private final Pose backupForPickup = new Pose(30, 60, Math.toRadians(200));
    private final Pose park = new Pose(16, 40, Math.toRadians(245));

    public void buildPaths() {
        paths[0] = buildLine(Constants.specimenStartPosition, placeInitial, HeadingInterpolation.CONSTANT);
        paths[1] = buildLine(placeInitial,backup, HeadingInterpolation.CONSTANT);

        paths[2] = buildCurve(backup,curve1,curve2,curveToPush, HeadingInterpolation.CONSTANT);
        paths[3] = buildLine(curveToPush,pushSample1, HeadingInterpolation.CONSTANT);
        paths[4] = buildCurve(pushSample1,lineup1,lineupSample2, HeadingInterpolation.CONSTANT);
        paths[5] = buildLine(lineupSample2, pushSample2, HeadingInterpolation.LINEAR);
        paths[6] = buildLine(pushSample2, pickupPosition, HeadingInterpolation.LINEAR);

        //Place #2

        paths[7] = buildLine(pickupPosition, placePosition, HeadingInterpolation.LINEAR);
        paths[8] = buildLine(placePosition, backupForPickup, HeadingInterpolation.CONSTANT);
        paths[9] = buildLine(backupForPickup, pickupPosition, HeadingInterpolation.LINEAR);

        // Place #3
        paths[10] = buildLine(pickupPosition, placePosition, HeadingInterpolation.LINEAR);
        paths[11] = buildLine(placePosition, backupForPickup, HeadingInterpolation.LINEAR);
        paths[12] = buildLine(backupForPickup, pickupPosition, HeadingInterpolation.CONSTANT);

        // Place #4
        paths[13] = buildLine(pickupPosition, placePosition, HeadingInterpolation.LINEAR);
        paths[14] = buildLine(placePosition, backupForPickup, HeadingInterpolation.LINEAR);
        //paths[15] = buildLine(backupForPickup, pickupPosition, HeadingInterpolation.CONSTANT);

        paths[15] = buildLine(backupForPickup, park, HeadingInterpolation.LINEAR);
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
        long claw_placement_delay = 625;
        long joint_wait_after_placement_delay = 300;
        long jointWaitCommand = 300;

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetSlide(robot.slide,700).andThen(
                                        new ParallelCommandGroup(
                                                new PathCommand(paths[0]),
                                                new SetBasket(robot.basket, Constants.basketMaxPosition),
                                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition),
                                                new SetSlide(robot.slide, Constants.slideMiddlePosition),
                                                new SetWrist(robot.wrist, Constants.wristPlacePosition)
                                        )
                                )
                        ),
                        new SetClaw(robot.claw, Constants.claw_not_so_open_position),
                        new WaitCommand(10),
                        new ParallelCommandGroup(
                                new PathCommand(paths[1]),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new SetJoint(robot.joint, Constants.jointStraightUp)
                                )
                        ),
                        new ParallelCommandGroup(
                                new PathCommand(paths[2]),
                                new SetSlide(robot.slide, Constants.slideMinPosition),
                                new SetBasket(robot.basket, Constants.basketStartingPosition),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),


                        new PathChainCommand(paths[3], paths[4], paths[5], paths[6]),


                        // TODO: Pickup Specimen 2
                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition),
                        new SetClaw(robot.claw, Constants.claw_almost_closed_position),
                        new WaitCommand(clawWaitCommand),
                        new ParallelCommandGroup(
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePositionTop),
                                new SequentialCommandGroup(
                                        new SetWrist(robot.wrist, Constants.wrist_speciman_fix_position),
                                        new WaitCommand(300),
                                        new SetClaw(robot.claw, Constants.clawClosedPosition)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(jointWaitCommand),
                                        new PathCommand(paths[7])
                                )
                        ),
                        new SetWrist(robot.wrist,Constants.wristPlacePosition),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new PathCommand(paths[8]),
                                new SequentialCommandGroup(
                                        new WaitCommand(claw_placement_delay),
                                        new SetClaw(robot.claw,Constants.clawOpenPosition)
                                )
                        ),
                        new WaitCommand(50),
                        new ParallelCommandGroup(
                                new PathCommand(paths[9]),
                                new SequentialCommandGroup(
                                        new WaitCommand(joint_wait_after_placement_delay),
                                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition)
                                ),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),




                        // TODO: Pickup Specimen 3
                        new SetClaw(robot.claw, Constants.claw_almost_closed_position),
                        new WaitCommand(clawWaitCommand),
                        new ParallelCommandGroup(
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePositionTop),
                                new SequentialCommandGroup(
                                    new SetWrist(robot.wrist, Constants.wrist_speciman_fix_position),
                                    new WaitCommand(300),
                                    new SetClaw(robot.claw, Constants.clawClosedPosition)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(jointWaitCommand),
                                        new PathCommand(paths[7])
                                )
                        ),
                        new SetWrist(robot.wrist,Constants.wristPlacePosition),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new PathCommand(paths[8]),
                                new SequentialCommandGroup(
                                        new WaitCommand(claw_placement_delay),
                                        new SetClaw(robot.claw,Constants.clawOpenPosition)
                                )
                        ),
                        new WaitCommand(50),
                        new ParallelCommandGroup(
                                new PathCommand(paths[9]),
                                new SequentialCommandGroup(
                                        new WaitCommand(joint_wait_after_placement_delay),
                                        new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition)
                                ),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),




                        // TODO: Pickup Specimen 4
                        new SetClaw(robot.claw, Constants.claw_almost_closed_position),
                        new WaitCommand(clawWaitCommand),
                        new ParallelCommandGroup(
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePositionTop),
                                new SequentialCommandGroup(
                                        new SetWrist(robot.wrist, Constants.wrist_speciman_fix_position),
                                        new WaitCommand(300),
                                        new SetClaw(robot.claw, Constants.clawClosedPosition)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(jointWaitCommand),
                                        new PathCommand(paths[7])
                                )
                        ),
                        new SetWrist(robot.wrist,Constants.wristPlacePosition),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new PathCommand(paths[8]),
                                new SequentialCommandGroup(
                                        new WaitCommand(claw_placement_delay),
                                        new SetClaw(robot.claw,Constants.clawOpenPosition)
                                )
                        ),
                        new WaitCommand(50),
                        new ParallelCommandGroup(
                                new PathCommand(paths[15]),
                                new SetJoint(robot.joint, Constants.jointSpecimenPickupPosition)
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
