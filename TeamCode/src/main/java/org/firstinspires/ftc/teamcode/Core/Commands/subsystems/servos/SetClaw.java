package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.Claw;

public class SetClaw extends CommandBase {

    private final Claw claw;
    private double targetPosition;

    public SetClaw(Claw claw, double targetPosition) {
        this.claw = claw;
        this.targetPosition = targetPosition;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
