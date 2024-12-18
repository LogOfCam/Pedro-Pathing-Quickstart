package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;

public class SetSlide extends CommandBase {
    private final Slide slide;
    private double targetPosition = 0;
    private final double threshold = 10;

    public SetSlide(Slide slide, double targetPosition) {
        this.targetPosition = targetPosition;
        this.slide = slide;

        addRequirements(slide);
    }

    @Override
    public void execute() {
        slide.updateSlide();
    }

    @Override
    public void initialize(){
        slide.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(slide.getCurrentPosition() - targetPosition) < threshold;
    }
}