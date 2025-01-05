package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Drive;

public class SetDriveMultiplier extends CommandBase {
    private final Drive drive;
    private double multiplier = 0;
    public SetDriveMultiplier(Drive drive, double multiplier) {

        this.multiplier = multiplier;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() { drive.setMultiplier(multiplier); }

    @Override
    public boolean isFinished() {
        return true;
    }
}
