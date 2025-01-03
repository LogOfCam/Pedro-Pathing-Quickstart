package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.joint;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;

public class HoldJoint extends CommandBase {
    private final Joint joint;

    public HoldJoint(Joint joint) {
        this.joint = joint;
        addRequirements(joint);
    }

    @Override
    public void initialize(){
        joint.setTargetPosition(joint.getHoldPosition());
    }

    @Override
    public void execute() {

        joint.updateJoint();
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted) {
            joint.setPower(0);
        }
    }
}

