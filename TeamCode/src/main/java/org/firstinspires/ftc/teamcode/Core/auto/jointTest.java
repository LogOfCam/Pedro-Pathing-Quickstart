package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

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
//                        new SetSlide(robot.slide, Constants.slideMaxPosition),
//                        new WaitCommand(3000),
//                        new SetJoint(robot.joint, Constants.jointStraightUp), // 1400
//                        new WaitCommand(3000),
//                        new SetSlide(robot.slide, Constants.slideMinPosition),
//                        new WaitCommand(3000),
//                        new SetJoint(robot.joint, Constants.jointTransferPosition), // 2600
//                        new WaitCommand(3000),
//                        new SetJoint(robot.joint, Constants.jointStraightUp), // 2300
//                        new WaitCommand(3000),
//                        new SetSlide(robot.slide, Constants.slideHighBasketPosition)

                        new SetJoint(robot.joint, 1400),
                        new WaitCommand(3000),
                        new SetJoint(robot.joint, 2300),
                        new WaitCommand(3000),
                        new SetJoint(robot.joint, 3000 )
                )
        );

        int cycles;
        cycles = 0;

        while(opModeIsActive() && !isStopRequested()) {

            cycles++;

            CommandScheduler.getInstance().run();

            telemetry.addLine("RUNNING");
            telemetry.addLine("");
            t1 = timer.milliseconds() / cycles;
            telemetry.addData("Cycles", cycles);
            telemetry.addData("Loop", t1);
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
