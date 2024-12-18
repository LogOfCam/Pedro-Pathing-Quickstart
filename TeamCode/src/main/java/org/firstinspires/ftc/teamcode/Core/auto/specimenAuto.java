package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildLine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.drive.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class specimenAuto extends LinearOpMode {
    public static Path[] paths = new Path[2];
    private final Pose startPose = new Pose(24,36, Math.toRadians(180)); //270 = South
    private final Pose placePose = new Pose(33, 72, Math.toRadians(0));
    private final Point placePoint = new Point(0, 75);
    private final Pose resetPose = new Pose(24, 36, Math.toRadians(180));

    //private final Pose scorePose = new Pose(14, 119, Math.toRadians(315)); //315 = -45 or towards basket
    public void buildPaths() {
//        paths[0] = buildLine(
//                startPose,
//                scorePose,
//                HeadingInterpolation.LINEAR
//        );
        paths[0] = buildCurve(
                startPose,
                placePoint,
                placePose,
                HeadingInterpolation.LINEAR
        );
        paths[1] = buildLine(
                placePose,
                resetPose,
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
            robot.wrist.setPosition(Constants.wristStartingPosition);

            telemetry.addLine("INITIALIZED");
            telemetry.update();
        }

        robot.setStartingPose(startPose);
        //robot.setPose(startPose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//
//                                new SetSlide(robot.slide, 2000),
//                                new SetJoint(robot.joint, 1000)
//                        ),
//                        new WaitCommand(5000),
//                        new SetClaw(robot.claw, Constants.clawOpenPosition)
                        new PathCommand(paths[0])
                        //new PathCommand(paths[1])
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
