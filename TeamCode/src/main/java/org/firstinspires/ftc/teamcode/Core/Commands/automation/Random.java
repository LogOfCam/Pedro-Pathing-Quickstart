package org.firstinspires.ftc.teamcode.Core.Commands.automation;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class Random extends SequentialCommandGroup {

    public Random(Robot robot) {
        addCommands(
                new SetSlide(robot.slide, Constants.slideMaxPosition),
                new SetJoint(robot.joint, Constants.jointStraightUp),
                new WaitCommand(3000),
                new SetSlide(robot.slide, Constants.slideMinPosition)
        );
    }
}
