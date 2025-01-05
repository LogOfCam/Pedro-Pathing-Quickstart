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
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetBasket;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Sample Auto", group = "sampl,e")
public class sampleAuto extends LinearOpMode {

    private Robot robot;
    public static Path[] paths = new Path[9];

    //360 or 0 = left, 90 = DOWN, 180 = RIGHT, 270 = up

    private final Pose placePosition = new Pose(12, 129, Math.toRadians(325));
    private final Pose placeCorrectionPosition = new Pose(11, 130, Math.toRadians(325));
    private final Pose pickupPosition1 = new Pose(22, 117.5, Math.toRadians(0));
    private final Pose pickupPosition2 = new Pose(22,128, Math.toRadians(0));
    private final Pose pickupPosition3 = new Pose(47.5,118.5, Math.toRadians(90));
    private final Pose parkPosition = new Pose(60,96, Math.toRadians(90));
    private final Point parkPoint = new Point(61,122);

    public void buildPaths() {
        paths[0] = buildLine(Constants.sampleStartPosition, placePosition, HeadingInterpolation.LINEAR);//place initial sample
        paths[1] = buildLine(placePosition, placeCorrectionPosition, HeadingInterpolation.CONSTANT); // Drive forward an inch for basket
        paths[2] = buildLine(placeCorrectionPosition, pickupPosition1, HeadingInterpolation.LINEAR);// pickup sample 1
        //paths[2] = buildLine(pickupPosition1, placePosition, HeadingInterpolation.LINEAR);// place sample 1
        //paths[3] = buildLine(placePosition, pickupPosition2, HeadingInterpolation.LINEAR);// pickup sample 2
        //paths[4] = buildLine(pickupPosition2, placePosition, HeadingInterpolation.LINEAR);// place sample 2
        //paths[5] = buildLine(placePosition, pickupPosition3, HeadingInterpolation.LINEAR);// pickup sample 3
        //paths[6] = buildLine(pickupPosition3, placePosition, HeadingInterpolation.LINEAR);// place sample 3
        //paths[7] = buildCurve(placePosition, parkPoint, parkPosition, HeadingInterpolation.LINEAR);// park
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
                        new ParallelCommandGroup(
                                new PathCommand(paths[0]),
                                new SetSlide(robot.slide, 1000).andThen(
                                        new ParallelCommandGroup(
                                                new SetJoint(robot.joint, Constants.jointStraightUp),
                                                new SetSlide(robot.slide, Constants.slideMaxPosition)
                                        )
                                )
                        ),
                        new WaitCommand(1000),
                        new PathCommand(paths[1]),
                        new SetBasket(robot.basket, Constants.basketPlacePosition)
//                        new WaitCommand(1000),
//                        new ParallelCommandGroup(
//                                new PathCommand(paths[1]),
//                                new SetBasket(robot.basket, Constants.basketStartingPosition),
//                                new SetSlide(robot.slide, Constants.slideTransferPosition),
//                                new SetJoint(robot.joint, Constants.jointSamplePickupPosition),
//                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
//                                new SetClaw(robot.claw, Constants.clawOpenPosition)
//
//                        ),
//                        new SetClaw(robot.claw, Constants.clawClosedPosition),
//                        new WaitCommand(1000),
//                        new ParallelCommandGroup(
//                                new PathCommand(paths[2]),
//                                new SetWrist(robot.wrist, Constants.wristTransferPosition),
//                                new SetJoint(robot.joint, Constants.jointTransferPosition).andThen(
//                                        new SequentialCommandGroup(
//                                                new SetClaw(robot.claw, Constants.clawOpenPosition)
//                                        )
//                                )
//                        ),
//                        new WaitCommand(1000),
//                        new ParallelCommandGroup(
//                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
//                                new SetJoint(robot.joint, Constants.jointStraightUp),
//                                new SetSlide(robot.slide, Constants.slideMaxPosition).andThen(
//                                        new SequentialCommandGroup(
//                                                new SetBasket(robot.basket, Constants.basketPlacePosition)
//                                        )
//                                )
//                        )

//                        new WaitCommand(1000),
//                        new PathCommand(paths[1]),
//                        new WaitCommand(1000),
//                        new PathCommand(paths[2]),
//                        new WaitCommand(1000),
//                        new PathCommand(paths[3]),
//                        new WaitCommand(1000),
//                        new PathCommand(paths[4]),
//                        new WaitCommand(1000),
//                        new PathCommand(paths[5]),
//                        new WaitCommand(1000),
//                        new PathCommand(paths[6]),
//                        new WaitCommand(1000),
//                        new PathCommand(paths[7])


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