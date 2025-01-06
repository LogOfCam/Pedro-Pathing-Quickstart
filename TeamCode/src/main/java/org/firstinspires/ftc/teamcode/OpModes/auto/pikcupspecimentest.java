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
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Specimen Pickup (NO SLIDE)", group = "specimen")
public class pikcupspecimentest extends LinearOpMode {

    private Robot robot;

    public static Path[] paths = new Path[6];
    private final Pose placeInitial = new Pose(39, 62, Math.toRadians(180));
    private final Pose backup = new Pose(30, 62, Math.toRadians(180));
    private final Pose pickupSample1 = new Pose(28,36,Math.toRadians(315));
    private final Pose pickupSample2 = new Pose(28,23,Math.toRadians(315));
    private final Pose throwSample1 = new Pose(28,36,Math.toRadians(130));
    private final Pose throwSample2 = new Pose(28,23,Math.toRadians(130));


    private final Pose placePosition = new Pose(36, 62, Math.toRadians(200));
    private final Pose backupForPickup = new Pose(30, 62, Math.toRadians(225));

    public void buildPaths() {
        paths[0] = buildLine(Constants.specimenStartPosition, placeInitial, HeadingInterpolation.CONSTANT);
        paths[1] = buildLine(placeInitial, backup, HeadingInterpolation.CONSTANT);
        paths[2] = buildLine(backup,pickupSample1, HeadingInterpolation.LINEAR);
        paths[3] = buildLine(pickupSample1,throwSample1, HeadingInterpolation.LINEAR);
        paths[4] = buildLine(throwSample1,pickupSample2, HeadingInterpolation.LINEAR);
        paths[5] = buildLine(pickupSample2,throwSample2, HeadingInterpolation.LINEAR);




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
                                new PathCommand(paths[0]),
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition),
                                new SetWrist(robot.wrist, Constants.wristPlacePosition)
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(100),
                        new PathCommand(paths[1]),
                        new ParallelCommandGroup(
                                new PathCommand(paths[2]),
                                new SetJoint(robot.joint, Constants.jointSamplePickupPosition),
                                new SetWrist(robot.wrist, Constants.wristPickupPosition),
                                new SetClaw(robot.claw, Constants.clawOpenPosition)
                        ),
                        new SetClaw(robot.claw, Constants.clawClosedPosition),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new PathCommand(paths[3]),
                                new SetClaw(robot.claw, Constants.clawOpenPosition)
                                ),
                        new PathCommand(paths[3]),
                        new PathCommand(paths[4]),
                        new PathCommand(paths[5])
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
