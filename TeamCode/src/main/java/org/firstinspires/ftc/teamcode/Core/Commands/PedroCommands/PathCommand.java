package org.firstinspires.ftc.teamcode.Core.Commands.PedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class PathCommand extends CommandBase {
    private final Path path;
    private final Robot robot = Robot.getInstance();
    private final double speed;
    private boolean holdEnd = false;

    public PathCommand(Path path) {
        this.path = path;
        this.speed = 1;
    }

    public PathCommand(Path path, double speed) {
        this.path = path;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robot.setMaxPower(speed);
        robot.followPath(path, true);
    }

    @Override
    public void execute() {
        robot.update();
        robot.getDashboardPoseTracker().update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !robot.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        robot.setMaxPower(Constants.maxPower);
    }
}
