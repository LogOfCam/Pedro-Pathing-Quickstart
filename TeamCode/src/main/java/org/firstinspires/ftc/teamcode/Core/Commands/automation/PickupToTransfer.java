package org.firstinspires.ftc.teamcode.Core.Commands.automation;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetBasket;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetWrist;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetSlide;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class PickupToTransfer extends SequentialCommandGroup {

    public PickupToTransfer(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        new SetJoint(robot.joint, Constants.jointTransferPosition).andThen(
                                new SetClaw(robot.claw, Constants.clawOpenPosition)
                        ),
                        new SetSlide(robot.slide, Constants.slideTransferPosition),
                        new SetWrist(robot.wrist, Constants.wristTransferPosition),
                        new SetBasket(robot.basket, Constants.basketStartingPosition)

                )
        );
    }
}
