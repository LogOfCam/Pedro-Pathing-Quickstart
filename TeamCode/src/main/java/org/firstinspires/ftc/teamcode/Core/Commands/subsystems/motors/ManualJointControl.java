package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

import java.util.function.DoubleSupplier;

public class ManualJointControl extends CommandBase {

    private final Joint joint;
    private final DoubleSupplier key;

    public ManualJointControl(Joint joint, DoubleSupplier key) {
        this.joint = joint;
        this.key = key;
        addRequirements(this.joint);
    }

    @Override
    public void execute() {
        if(key.getAsDouble() > 0 && joint.getCurrentPosition() < Constants.jointMaxPosition) {
            joint.setTargetPosition(joint.getCurrentPosition() + (20 * key.getAsDouble()));
        } else if (key.getAsDouble() < 0 && joint.getCurrentPosition() > Constants.jointMinPosition) {
            joint.setTargetPosition(joint.getCurrentPosition() - (20 * key.getAsDouble()));
        }
    }

}
