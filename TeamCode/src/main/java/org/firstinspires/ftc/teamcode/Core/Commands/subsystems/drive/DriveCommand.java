package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Drive;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier y, x, rx;

    public DriveCommand(Drive drive, DoubleSupplier y, DoubleSupplier x, DoubleSupplier rx) {
        this.drive = drive;
        this.y = y;
        this.x = x;
        this.rx = rx;

        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        drive.teleopDrive(y.getAsDouble(), x.getAsDouble(), rx.getAsDouble());
    }
}
