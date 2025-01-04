package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint.HoldJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class jointTest extends LinearOpMode {
    private Robot robot;

    public static Path[] paths = new Path[2];
    private final Pose forwardStart = new Pose(0, 0, Math.toRadians(270));
    private final Point midPoint = new Point(0, 30);
    private final Pose end = new Pose(0, 60, Math.toRadians(270));

    public void buildPaths() {
        paths[0] = buildCurve(forwardStart, midPoint, end, HeadingInterpolation.CONSTANT);
        paths[1] = buildCurve(end, midPoint, forwardStart, HeadingInterpolation.CONSTANT);
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

        robot.setPose(forwardStart);

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(


                        new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition), // 1400
                        new WaitCommand(5000),
                        new SetJoint(robot.joint, Constants.jointStraightUp), // 2600
                        new WaitCommand(5000),
                        new SetJoint(robot.joint, 3000) // 2300

                )
        );

        //CommandScheduler.getInstance().setDefaultCommand(robot.joint, new HoldJoint(robot.joint));

        //robot.slide.setDefaultCommand(CommandScheduler.getInstance());
        //robot.joint.setDefaultCommand(CommandScheduler.getInstance());

        while(opModeIsActive() && !isStopRequested()) {

            CommandScheduler.getInstance().run();

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        telemetry.addData("LastPosition", robot.joint.getHoldPosition());
        telemetry.addData("CurrentPosition", robot.joint.getCurrentPosition());
        telemetry.addData("TargetPosition", robot.joint.getTargetPosition());
        telemetry.addData("ActualTargetPosition", robot.joint.getActualTargetPosition());
        telemetry.addData("Power", robot.joint.getPower());
        telemetry.update();
    }
}
