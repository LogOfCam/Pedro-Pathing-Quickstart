package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.wrappers.AutonomousHelpers.buildLine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.drive.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.wrappers.AutonomousHelpers;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class funAuto extends OpMode {
    private Robot robot;



    public static Path[] paths = new Path[1];
    private final Pose startPose = new Pose(8,82.5, Math.toRadians(270)); //270 = South
    private final Pose scorePose = new Pose(14, 119, Math.toRadians(315)); //315 = -45 or towards basket
    public void buildPaths() {
        paths[0] = buildLine(
                startPose,
                scorePose,
                AutonomousHelpers.HeadingInterpolation.LINEAR
        );
    }


    @Override
    public void init() {
        Robot robot = Robot.getInstance();

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        robot.setStartingPose(startPose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PathCommand(paths[0]),
                        //new SetSlide(robot.slide, 1000),
                        new SetClaw(robot.claw, 0.85)
                )
        );

        robot.claw.setPosition(0.45);
    }

    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
