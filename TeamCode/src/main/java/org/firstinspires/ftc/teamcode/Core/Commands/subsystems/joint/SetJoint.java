package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;

public class SetJoint extends CommandBase {
    private final Joint joint;
    private double targetPosition = 0;
    private final double threshold = 10;

    public SetJoint(Joint joint, double targetPosition) {
        this.targetPosition = targetPosition;
        this.joint = joint;

        addRequirements(joint);
    }

    @Override
    public void execute() {
        joint.updateJoint();
    }

    @Override
    public void initialize(){
        joint.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(joint.getCurrentPosition() - targetPosition) < threshold;
    }
}
