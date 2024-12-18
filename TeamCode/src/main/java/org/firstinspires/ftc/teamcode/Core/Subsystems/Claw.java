package org.firstinspires.ftc.teamcode.Core.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private final Servo claw;
    private double closedPosition = 0.85;
    private double openPosition = 0.55;

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