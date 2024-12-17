package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.Commands.slide.SlideTo;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous
public class specimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot();

        SequentialCommandGroup group = new SequentialCommandGroup(
              //new PickupSampleForHuman(bot),
                new SlideTo(robot.slide, 1000),

                new ParallelCommandGroup(
                    new SlideTo(robot.slide, 1000)
                )

        );

        group.schedule();

        telemetry.addLine("INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}
