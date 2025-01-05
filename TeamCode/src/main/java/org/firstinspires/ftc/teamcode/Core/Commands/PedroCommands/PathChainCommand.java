package org.firstinspires.ftc.teamcode.Core.Commands.PedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class PathChainCommand extends CommandBase {
    private final PathChain pathChain;
    private final Robot robot = Robot.getInstance();
    private final double speed;
    private boolean holdEnd = true;

    public PathChainCommand(Path... paths) {
        this.pathChain = new PathChain(paths);
        this.speed = 1;
    }

    public PathChainCommand(double speed, boolean holdEnd ,Path... paths) {
        this.pathChain = new PathChain(paths);
        this.speed = speed;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        robot.setMaxPower(Constants.maxPower);
        robot.followPath(pathChain, true);
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
