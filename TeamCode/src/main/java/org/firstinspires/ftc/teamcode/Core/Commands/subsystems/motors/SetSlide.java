package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors;

import static org.firstinspires.ftc.teamcode.Core.util.Constants.slideMaxPosition;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class SetSlide extends CommandBase {
    private final Slide slide;
    private double targetPosition = 0;
    public SetSlide(Slide slide, double targetPosition) {
        if(targetPosition > slideMaxPosition) {targetPosition = slideMaxPosition; }

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
        return Math.abs(slide.getCurrentPosition() - targetPosition) <= Constants.slideThreshold;
    }
}