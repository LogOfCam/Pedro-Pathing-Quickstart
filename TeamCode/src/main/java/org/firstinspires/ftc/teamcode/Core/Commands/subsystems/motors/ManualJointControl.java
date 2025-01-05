package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

import java.util.function.DoubleSupplier;

public class ManualJointControl extends CommandBase {

    private final Joint joint;
    private final DoubleSupplier up, down;
    public ManualJointControl(Joint joint, DoubleSupplier up, DoubleSupplier down) {
        this.joint = joint;
        this.up = up;
        this.down = down;
        addRequirements(this.joint);
    }

    @Override
    public void execute() {
        if(up.getAsDouble() > 0 && joint.getCurrentPosition() < Constants.jointMaxPosition) {
            joint.setTargetPosition(joint.getTargetPosition() + (2 * up.getAsDouble()));
        } else if (down.getAsDouble() > 0 && joint.getCurrentPosition() > Constants.jointMinPosition) {
            joint.setTargetPosition(joint.getTargetPosition() - (2 * down.getAsDouble()));
        }
    }
}
