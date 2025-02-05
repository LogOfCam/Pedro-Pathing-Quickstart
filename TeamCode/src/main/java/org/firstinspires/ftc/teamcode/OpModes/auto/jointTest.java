package org.firstinspires.ftc.teamcode.OpModes.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Disabled
@Autonomous(name = "TestAuto", group = "test")
public class jointTest extends LinearOpMode {
    private Robot robot;

    public static Path[] paths = new Path[2];
    private final Pose forwardStart = new Pose(0, 0, Math.toRadians(270));
    private final Point midPoint = new Point(0, 30);
    private final Pose end = new Pose(0, 60, Math.toRadians(270));

    double t1;

    public void buildPaths() {
        paths[0] = buildCurve(forwardStart, midPoint, end, HeadingInterpolation.CONSTANT);
        paths[1] = buildCurve(end, midPoint, forwardStart, HeadingInterpolation.CONSTANT);
    }

    @Override
    public void runOpMode() {
        robot = Robot.getInstance();

        robot.initialize(hardwareMap, telemetry);

        buildPaths();

        ElapsedTime timer = new ElapsedTime();

        while(!isStarted()) {

            CommandScheduler.getInstance().run();

            robot.claw.setPosition(Constants.clawClosedPosition);
            robot.wrist.setPosition(Constants.wristStartingPosition);

            telemetry.addLine("INITIALIZED");
            telemetry.addLine("");
            updateTelemetry();

            timer.reset();
        }

        robot.setPose(forwardStart);

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new SetJoint(robot.joint, 500),
                        new WaitCommand(3000),
                        new SetJoint(robot.joint, 1500),
                        new WaitCommand(3000),
                        new SetJoint(robot.joint, 3000)
                )
        );

        while(opModeIsActive() && !isStopRequested()) {

            CommandScheduler.getInstance().run();

            telemetry.addLine("RUNNING");
            telemetry.addLine("");
            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        telemetry.addData("CurrentPosition", robot.joint.getCurrentPosition());
        telemetry.addData("TargetPosition", robot.joint.getTargetPosition());
        telemetry.addData("ActualTargetPosition", robot.joint.getActualTargetPosition());
        telemetry.addData("Power", robot.joint.getPower());
        telemetry.update();
    }
}
