package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors;

import static org.firstinspires.ftc.teamcode.Core.util.Constants.jointMaxPosition;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class SetJoint extends CommandBase {
    private final Joint joint;
    private double targetPosition = 0;
    public SetJoint(Joint joint, double targetPosition) {
        if(targetPosition > jointMaxPosition) {targetPosition = jointMaxPosition; }

        this.targetPosition = targetPosition;
        this.joint = joint;

        addRequirements(joint);
    }

    @Override
    public void initialize() { joint.setTargetPosition(targetPosition); }

    @Override
    public boolean isFinished() {
        return Math.abs(joint.getCurrentPosition() - targetPosition) <= Constants.jointThreshold;
    }
}