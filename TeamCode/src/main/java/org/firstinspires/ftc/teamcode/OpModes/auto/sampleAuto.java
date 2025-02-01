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
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetBasket;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetWrist;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Sample Auto", group = "sampl,e")
public class sampleAuto extends LinearOpMode {

    private Robot robot;
    public static Path[] paths = new Path[14];

    //360 or 0 = left, 90 = DOWN, 180 = RIGHT, 270 = up

    private final Pose placePosition = new Pose(14, 127, Math.toRadians(325));
    private final Pose placeCorrectionPosition = new Pose(10, 129, Math.toRadians(325));
    private final Pose pickupPosition1 = new Pose(21.5, 117.5, Math.toRadians(0));
    private final Pose pickupPosition2 = new Pose(22,127.25, Math.toRadians(0));
    private final Pose pickupPosition25 = new Pose(16,127.25, Math.toRadians(0));
    private final Pose LineupPosition1 = new Pose(14.5,117.5, Math.toRadians(0));
    private final Pose LineupPosition2 = new Pose(15.5,127.25, Math.toRadians(0));
    private final Pose LineupPosition3 = new Pose(47.5,111.5, Math.toRadians(90));
    private final Pose pickupPosition3 = new Pose(47.5,119.5, Math.toRadians(90));
    private final Pose parkPosition = new Pose(60,100, Math.toRadians(270));
    private final Point parkPoint = new Point(61,122);
    private final Point pickUp3Curve = new Point(36.5,90);
    public void buildPaths() {
        paths[0] = buildLine(Constants.sampleStartPosition, placeCorrectionPosition, HeadingInterpolation.LINEAR);//place initial sample
        paths[1] = buildLine(placePosition, placeCorrectionPosition, HeadingInterpolation.CONSTANT); // Drive forward an inch for basket
        paths[2] = buildLine(placeCorrectionPosition, LineupPosition1, HeadingInterpolation.LINEAR);// pickup sample 1
        paths[13] = buildLine(LineupPosition1, pickupPosition1, HeadingInterpolation.LINEAR);
        paths[3] = buildLine(pickupPosition1, placePosition, HeadingInterpolation.LINEAR);// place sample 1
        paths[4] = buildLine(placePosition, LineupPosition2, HeadingInterpolation.LINEAR);
        paths[12] = buildLine(LineupPosition2, pickupPosition25, HeadingInterpolation.LINEAR);
        paths[5] = buildLine(pickupPosition25, pickupPosition2, HeadingInterpolation.LINEAR);// pickup sample 2
        paths[6] = buildLine(pickupPosition2, placePosition, HeadingInterpolation.LINEAR);// place sample 2
        paths[7] = buildCurve(placePosition, pickUp3Curve, LineupPosition3, HeadingInterpolation.LINEAR);// pickup sample 3
        paths[8] = buildLine(LineupPosition3, pickupPosition3, HeadingInterpolation.LINEAR);
        paths[9] = buildCurve(pickupPosition3,pickUp3Curve, placePosition, HeadingInterpolation.LINEAR);// place sample 3
        paths[10] = buildCurve(placePosition, parkPoint, parkPosition, HeadingInterpolation.LINEAR);// park
        paths[11] = buildLine(placeCorrectionPosition, placePosition, HeadingInterpolation.LINEAR);
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
            robot.basket.setPosition(Constants.basketStartingPosition);

            updateTelemetry();
        }

        robot.setPose(Constants.sampleStartPosition);

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(

                        new SetSlide(robot.slide, 1000).andThen(
                                        new ParallelCommandGroup(
                                                new SetJoint(robot.joint, Constants.jointStraightUp),
                                                new SetSlide(robot.slide, Constants.slideMaxPosition)
                                        )
                        ),
                        new ParallelCommandGroup(
                                new PathCommand(paths[0]),
                                new SetJoint(robot.joint, Constants.jointSampleAlmostPickupPosition),
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),
                        new SetBasket(robot.basket, Constants.basketPlacePosition),
                        new WaitCommand(300),
                        new PathCommand(paths[11]),
                        new ParallelCommandGroup(
                                new PathCommand(paths[2]),
                                new SetBasket(robot.basket, Constants.basketStartingPosition),
                                new SetSlide(robot.slide, Constants.slideTransferPosition)
                        ),
                        new PathCommand(paths[13]),
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new PathCommand(paths[3]),
                                new SetWrist(robot.wrist, Constants.wristTransferPosition),
                                new SetJoint(robot.joint, Constants.jointTransferPosition)
                        ),
                new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetJoint(robot.joint, Constants.jointSampleAlmostPickupPosition),
                                new SetSlide(robot.slide, Constants.slideMaxPosition)
                        ),
                new PathCommand(paths[1]),
                new SetBasket(robot.basket, Constants.basketPlacePosition),
                new WaitCommand(300),
                        new PathCommand(paths[11]),
                        new ParallelCommandGroup(
                                new PathCommand(paths[4]),
                                new SetBasket(robot.basket, Constants.basketStartingPosition),
                                new SetSlide(robot.slide, Constants.slideTransferPosition),
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetClaw(robot.claw, Constants.clawOpenPosition)
                        ),
                        new PathChainCommand(paths[12], paths[5]),
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(400),
                        new ParallelCommandGroup(
                                new PathCommand(paths[6]),
                                new SetWrist(robot.wrist, Constants.wristTransferPosition),
                                new SetJoint(robot.joint, Constants.jointTransferPosition)
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetJoint(robot.joint, Constants.jointStraightUp),
                                new SetSlide(robot.slide, Constants.slideMaxPosition)
                        ),
                        new PathCommand(paths[1]),
                        new SetBasket(robot.basket, Constants.basketPlacePosition),
                        new WaitCommand(300),
                        new PathCommand(paths[11]),
                        new ParallelCommandGroup(
                                new PathChainCommand(paths[7], paths[8]),
                                new SetBasket(robot.basket, Constants.basketStartingPosition),
                                new SetSlide(robot.slide, Constants.slideTransferPosition),
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetJoint(robot.joint, Constants.jointSampleAlmostPickupPosition),
                                new SetClaw(robot.claw, Constants.clawOpenPickupPosition)
                        ),
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(400),
                        new ParallelCommandGroup(
                                new PathCommand(paths[9]),
                                new SetWrist(robot.wrist, Constants.wristTransferPosition),
                                new SetJoint(robot.joint, Constants.jointSideWaysTransforePosition)
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(150),
                        new ParallelCommandGroup(
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetJoint(robot.joint, Constants.jointStraightUp),
                                new SetSlide(robot.slide, Constants.slideMaxPosition)

                        ),
                        new PathCommand(paths[1]),
                        new SetBasket(robot.basket, Constants.basketPlacePosition),
                        new WaitCommand(300),
                        new PathCommand(paths[11]),
                        new ParallelCommandGroup(
                                new PathCommand(paths[10]),
                                new SetSlide(robot.slide, Constants.slideTransferPosition)
                        ),
                        new SetJoint(robot.joint, Constants.joint_park_position)
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