package org.firstinspires.ftc.teamcode.auto.commands;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Core.Commands.slide.SlideTo;
public class PickupSampleForHuman extends SequentialCommandGroup {

    public PickupSampleForHuman(Robot robot) {
        addCommands(
                new SlideTo(robot.slide, 2500)
        );
    }
}
