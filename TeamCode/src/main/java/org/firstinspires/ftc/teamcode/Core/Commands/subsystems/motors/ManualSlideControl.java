package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

import java.util.function.DoubleSupplier;

public class ManualSlideControl extends CommandBase {

    private final Slide slide;
    private final DoubleSupplier up, down;

    public ManualSlideControl(Slide slide, DoubleSupplier up, DoubleSupplier down) {
        this.slide = slide;
        this.up = up;
        this.down = down;
        addRequirements(this.slide);
    }

    @Override
    public void execute() {
        if(up.getAsDouble() > 0 && slide.getCurrentPosition() < Constants.slideMaxPosition) {
            slide.setTargetPosition(slide.getCurrentPosition() + (20 * up.getAsDouble()));
            //slide.setPower(1);
        } else if (down.getAsDouble() > 0 && slide.getCurrentPosition() > Constants.slideMinPosition) {
            slide.setTargetPosition(slide.getCurrentPosition() - (20 * down.getAsDouble()));
            //slide.setPower(-1);
        }
    }

}
