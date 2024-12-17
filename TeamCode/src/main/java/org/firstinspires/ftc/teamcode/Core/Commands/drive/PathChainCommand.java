package org.firstinspires.ftc.teamcode.Core.Commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Core.Bot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class PathChainCommand extends CommandBase {
    private final PathChain pathChain;
    private final Bot bot;

    private final double speed;

    public PathChainCommand(Path... paths) {
        this.pathChain = new PathChain(paths);
        this.speed = 1;
    }

    public PathChainCommand(double speed, Path... paths) {
        this.pathChain = new PathChain(paths);
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
