package org.firstinspires.ftc.teamcode.Core.Subsystems.servos;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private final Servo claw;
    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class,"clawServo");
    }

    public void setPosition(double targetPosition) {
        claw.setPosition(targetPosition);
    }

    public double getPosition() {
        return claw.getPosition();
    }
}