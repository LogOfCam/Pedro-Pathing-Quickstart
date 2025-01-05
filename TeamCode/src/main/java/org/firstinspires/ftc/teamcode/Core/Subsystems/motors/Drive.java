package org.firstinspires.ftc.teamcode.Core.Subsystems.motors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class Drive extends SubsystemBase {
    private final DcMotor frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor;
    double multiplier = Constants.driveFastMultiplier;
    double x;
    double bias = 1.1;
    private boolean state;

    public Drive(HardwareMap hardwareMap) {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        state = true;
    }

    public void teleopDrive(double y, double x, double rx) {
        this.x = x * bias;

        double denominator = Math.max(Math.abs(y) + Math.abs(this.x) + Math.abs(rx), 1);
        double backRightPower = ((y + this.x - rx) / denominator);
        double frontLeftPower = ((y + this.x + rx) / denominator);
        double frontRightPower = ((y - this.x - rx) / denominator);
        double backLeftPower = ((y - this.x + rx) / denominator);

        frontRightMotor.setPower(frontRightPower * multiplier);
        backRightMotor.setPower(backRightPower * multiplier);
        frontLeftMotor.setPower(frontLeftPower * multiplier);
        backLeftMotor.setPower(backLeftPower * multiplier);
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }

    public double getMultiplier() { return multiplier; }

    public void toggle() { state = !state; }

    public boolean getState() { return state; }
}
