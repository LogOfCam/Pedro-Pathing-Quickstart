package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Subsystems.servos.Basket;
import org.firstinspires.ftc.teamcode.Core.Subsystems.servos.Claw;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Joint;
import org.firstinspires.ftc.teamcode.Core.Subsystems.motors.Slide;
import org.firstinspires.ftc.teamcode.Core.Subsystems.servos.Wrist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class Robot extends Follower {

    public Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public Slide slide;
    public Claw claw;
    public Joint joint;
    public Wrist wrist;
    public Basket basket;

    private static Robot instance = null;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {
        initializePedro(hardwareMap); // initialize Pedro

        telemetry = telemetry_;

        this.slide = new Slide(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.joint = new Joint(hardwareMap);
        this.wrist = new Wrist(hardwareMap);
        this.basket = new Basket(hardwareMap);

        CommandScheduler.getInstance().reset();
    }

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        return instance;
    }
}
