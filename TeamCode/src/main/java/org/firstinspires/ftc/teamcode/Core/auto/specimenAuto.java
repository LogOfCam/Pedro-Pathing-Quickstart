package org.firstinspires.ftc.teamcode.Core.auto;

import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.HeadingInterpolation;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildCurve;
import static org.firstinspires.ftc.teamcode.Core.util.AutonomousHelpers.buildLine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.drive.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

@Autonomous
public class specimenAuto extends LinearOpMode {
    private Robot robot;

    public static Path[] paths = new Path[16];
    private final Pose placeSpecimenPosition1 = new Pose(36, 72, Math.toRadians(180));
    private final Pose placeSpecimenPosition2 = new Pose(36, 70, Math.toRadians(0));
    private final Pose placeSpecimenPosition3 = new Pose(36, 68, Math.toRadians(0));
    private final Pose placeSpecimenPosition4 = new Pose(36, 66, Math.toRadians(0));
    private final Pose placeSpecimenPosition5 = new Pose(36, 64, Math.toRadians(0));
    private final Pose pickupSamplePosition1 = new Pose(26, 40, Math.toRadians(320));
    private final Pose pickupSamplePosition2 = new Pose(26, 32, Math.toRadians(140));
    private final Pose pickupSamplePosition3 = new Pose(26, 24, Math.toRadians(140));
    private final Pose placeSamplePosition1 = new Pose(26, 40, Math.toRadians(20));
    private final Pose placeSamplePosition2 = new Pose(26, 32, Math.toRadians(20));
    private final Pose placeSamplePosition3 = new Pose(26, 24, Math.toRadians(20));
    private final Pose pickupSpecimenPosition = new Pose(26,24, Math.toRadians(0));
    private final Pose parkSpecimenPosition = new Pose(24, 36, Math.toRadians(30));

    public void buildPaths() {

        // Place initial specimen
        paths[0] = buildLine(
                Constants.specimenStartPosition,
                new Pose(36, 72, Math.toRadians(180)),
                HeadingInterpolation.CONSTANT
        );

        // Pickup first sample for HP
        paths[1] = buildLine(
                new Pose(36, 72, Math.toRadians(180)),
                new Pose(24, 48, Math.toRadians(310)),
                HeadingInterpolation.LINEAR
        );

        // Place first sample for HP
//        paths[2] = buildLine(
//                pickupSamplePosition1,
//                placeSamplePosition1,
//                HeadingInterpolation.LINEAR
//        );
//
//        // Pickup second sample for HP
//        paths[3] = buildLine(
//                placeSamplePosition1,
//                pickupSamplePosition2,
//                HeadingInterpolation.LINEAR
//        );
//
//        // Place second sample for HP
//        paths[4] = buildLine(
//                pickupSamplePosition2,
//                placeSamplePosition2,
//                HeadingInterpolation.LINEAR
//        );
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

            updateTelemetry();
        }

        robot.setStartingPose(Constants.specimenStartPosition);
        //robot.setPose(Constants.specimenStartPosition);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PathCommand(paths[0]),
                        new WaitCommand(500),
                        new PathCommand(paths[1])
                )
        );

        robot.slide.setDefaultCommand(CommandScheduler.getInstance());
        robot.joint.setDefaultCommand(CommandScheduler.getInstance());

        while(opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            updateTelemetry();
        }

        //CommandScheduler.getInstance().reset();
    }

    public void updateTelemetry() {
        telemetry.addData("x", robot.getPose().getX());
        telemetry.addData("y", robot.getPose().getY());
        telemetry.addData("heading", robot.getPose().getHeading());
        telemetry.addData("SlideTarget", robot.slide.getTargetPosition());
        telemetry.addData("SlideCurrent", robot.slide.getCurrentPosition());
        telemetry.addData("Claw", robot.claw.getPosition());
        telemetry.addData("Power", robot.slide.getPower());
        telemetry.update();
    }
}
