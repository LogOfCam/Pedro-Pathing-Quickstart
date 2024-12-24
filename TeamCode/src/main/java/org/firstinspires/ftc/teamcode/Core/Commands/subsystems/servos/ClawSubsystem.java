package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class ClawSubsystem {

    private Servo claw;

    public ClawSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "clawServo");
    }

    public void closeClaw() {
        claw.setPosition(Constants.clawClosedPosition);
    }

    public void openClaw() {
        claw.setPosition(Constants.clawOpenPosition);
    }

}
