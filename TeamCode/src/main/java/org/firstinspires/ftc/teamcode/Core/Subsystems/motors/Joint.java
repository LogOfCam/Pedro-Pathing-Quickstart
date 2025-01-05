package org.firstinspires.ftc.teamcode.Core.Subsystems.motors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Joint extends SubsystemBase {
    private final DcMotorEx jointMotor;
    private final PIDFController controller;
    double p = 0.0035, i = 0, d = 0.0002; // Was .004
    double f = 0.03;
    double ticksInDegrees = 285.0 / 180;
    private double targetPosition;
    public Joint(HardwareMap hardwareMap){
        jointMotor = hardwareMap.get(DcMotorEx.class,"jointMotor");
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initializeLynxModule(hardwareMap);

        controller = new PIDFController(p,i,d,f);
    }

    public void setTargetPosition(double targetPosition) {

        this.targetPosition = targetPosition;
    }

    @Override
    public void periodic() {
        updateJoint();
    }

    public void updateJoint() {
        int currentPosition = jointMotor.getCurrentPosition();

        //if (Math.abs(currentPosition - targetPosition) <= 300) { p = 0.002; } else { p = 0.004; }
        //controller.setPID(p,i,d);

        //double pid = controller.calculate(currentPosition, targetPosition);
        //double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;
        double power = controller.calculate(jointMotor.getCurrentPosition(), targetPosition * ticksInDegrees);
        setPower(power);

    }

    public void initializeLynxModule(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module: allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public double getTargetPosition() {
        return targetPosition;
    }
    public double getActualTargetPosition() {return jointMotor.getTargetPosition(); }
    public double getCurrentPosition() {
        return jointMotor.getCurrentPosition();
    }
    public double getPower() {
        return jointMotor.getPower();
    }

    public void setPower(double power) {
        jointMotor.setPower(power);
    }
}
