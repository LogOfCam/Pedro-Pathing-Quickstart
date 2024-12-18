package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Core.Subsystems.Slide;

public class SetSlide extends CommandBase {
    private final Slide slide;
    private final double targetPosition;
    private final double threshold = 10;

    public SetSlide(Slide slide, double targetPosition) {
        this.targetPosition = targetPosition;
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