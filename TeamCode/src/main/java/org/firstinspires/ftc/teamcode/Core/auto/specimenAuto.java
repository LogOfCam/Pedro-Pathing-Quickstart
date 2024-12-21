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

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Core.Commands.drive.PathCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class specimenAuto extends LinearOpMode {
    public static Path[] paths = new Path[16];
    private final Pose placeSpecimenPosition1 = new Pose(36, 72, Math.toRadians(0));
    private final Pose placeSpecimenPosition2 = new Pose(36, 70, Math.toRadians(0));
    private final Pose placeSpecimenPosition3 = new Pose(36, 68, Math.toRadians(0));
    private final Pose placeSpecimenPosition4 = new Pose(36, 66, Math.toRadians(0));
    private final Pose placeSpecimenPosition5 = new Pose(36, 64, Math.toRadians(0));
    private final Pose pickupSamplePosition1 = new Pose(26, 40, Math.toRadians(310));
    private final Pose pickupSamplePosition2 = new Pose(26, 32, Math.toRadians(310));
    private final Pose pickupSamplePosition3 = new Pose(26, 24, Math.toRadians(310));
    private final Pose placeSamplePosition1 = new Pose(26, 40, Math.toRadians(210));
    private final Pose placeSamplePosition2 = new Pose(26, 32, Math.toRadians(210));
    private final Pose placeSamplePosition3 = new Pose(26, 24, Math.toRadians(210));
    private final Pose pickupSpecimenPosition = new Pose(26,24, Math.toRadians(180));

    public void buildPaths() {

        // Place initial specimen
        paths[0] = buildLine(
                Constants.specimenStartPosition,
                placeSpecimenPosition1,
                HeadingInterpolation.CONSTANT
        );

        // Pickup first sample for HP
        paths[1] = buildLine(
                placeSpecimenPosition1,
                pickupSamplePosition1,
                HeadingInterpolation.LINEAR
        );

        // Place first sample for HP
        paths[2] = buildLine(
                pickupSamplePosition1,
                placeSamplePosition1,
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

        //robot.setStartingPose(Constants.specimenStartPosition);
        robot.setPose(Constants.specimenStartPosition);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new PathCommand(paths[0]),
                                new SetSlide(robot.slide, Constants.slideMaxPosition),
                                new SetJoint(robot.joint, Constants.jointSpecimenPlacePosition)
                        ),
                        new SetClaw(robot.claw, Constants.clawOpenPosition),
                        new WaitCommand(200),
                        new PathCommand(paths[1])
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
