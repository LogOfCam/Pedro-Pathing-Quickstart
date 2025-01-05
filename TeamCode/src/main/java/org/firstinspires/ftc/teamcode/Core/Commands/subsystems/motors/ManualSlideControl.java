package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

import java.util.function.DoubleSupplier;

public class ManualSlideControl extends CommandBase {

    private final Slide slide;
    private final DoubleSupplier key;

    public ManualSlideControl(Slide slide, DoubleSupplier key) {
        this.slide = slide;
        this.key = key;
        addRequirements(this.slide);
    }

    @Override
    public void execute() {
        if(key.getAsDouble() > 0 && slide.getCurrentPosition() < Constants.slideMaxPosition) {
            slide.setTargetPosition(slide.getTargetPosition() + (10 * key.getAsDouble()));
        } else if (key.getAsDouble() < 0 && slide.getCurrentPosition() > Constants.slideMinPosition) {
            slide.setTargetPosition(slide.getTargetPosition() + (10 * key.getAsDouble()));
        }
    }


}
