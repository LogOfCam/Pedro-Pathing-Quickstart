package org.firstinspires.ftc.teamcode.Core.Commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Core.Subsystems.Slide;

public class SlideTo extends CommandBase {
    private final Slide slide;
    private final double targetPosition;
    private final double threshold = 10;

    public SlideTo(Slide slide, double target) {
        this.targetPosition = target;
        this.slide = slide;

        addRequirements(slide);
    }

    @Override
    public void initialize(){
        slide.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(slide.getTargetPosition() - targetPosition) < threshold;
    }
}
