package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildLine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.drive.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

@Autonomous
public class specimenAuto extends LinearOpMode {
    public static Path[] paths = new Path[1];
    private final Pose startPose = new Pose(8,82.5, Math.toRadians(270)); //270 = South
    private final Pose scorePose = new Pose(14, 119, Math.toRadians(315)); //315 = -45 or towards basket
    public void buildPaths() {
        paths[0] = buildLine(
                startPose,
                scorePose,
                HeadingInterpolation.LINEAR
        );
    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        while(!isStarted()) {
            CommandScheduler.getInstance().run();

            robot.claw.setPosition(Constants.clawClosedPosition);

            telemetry.addLine("INITIALIZED");
            telemetry.update();
        }

        robot.setStartingPose(startPose);

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new PathCommand(paths[0]),
                                new SetSlide(robot.slide, 2000),
                                new SetJoint(robot.joint, 1000)
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition)
                )
        );

        robot.slide.setDefaultCommand(CommandScheduler.getInstance());
        robot.joint.setDefaultCommand(CommandScheduler.getInstance());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("SlideTarget", robot.slide.getTargetPosition());
            telemetry.addData("SlideCurrent", robot.slide.getCurrentPosition());
            telemetry.addData("Claw", robot.claw.getPosition());
            telemetry.addData("Power", robot.slide.getPower());
            telemetry.update();
        }

        //CommandScheduler.getInstance().reset();
    }
}
