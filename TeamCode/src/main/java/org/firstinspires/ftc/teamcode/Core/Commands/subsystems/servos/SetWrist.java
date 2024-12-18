package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.servos.Wrist;

public class SetWrist extends CommandBase {

    private final Wrist wrist;
    private double targetPosition;

    public SetWrist(Wrist wrist, double targetPosition) {
        this.wrist = wrist;
        this.targetPosition = targetPosition;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
