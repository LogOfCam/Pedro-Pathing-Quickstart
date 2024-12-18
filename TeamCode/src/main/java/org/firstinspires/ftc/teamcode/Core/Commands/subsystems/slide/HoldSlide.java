package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;

public class HoldSlide extends CommandBase {
    private final Slide slide;

    public HoldSlide(Slide slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize(){
        slide.setTargetPosition(slide.getCurrentPosition());
    }

    @Override
    public void execute() {
        slide.updateSlide();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted) {
            slide.setPower(0);
        }
    }
}
