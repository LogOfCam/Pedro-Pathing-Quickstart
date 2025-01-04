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

    /**
     * This creates a new Follower given a HardwareMap.
     *
     * @param hardwareMap HardwareMap required
     */
//    public Robot(HardwareMap hardwareMap) {
//        super(hardwareMap);
//    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {
        initializePedro(hardwareMap); // initialize Pedro

        telemetry = telemetry_;

        CommandScheduler.getInstance().reset(); /* RESETS COMMANDS ( BEFORE REGISTER ) */
        RegisterSubsystems(hardwareMap);

    }

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
            //return instance;
        }
        return instance;
    }

    public void RegisterSubsystems(HardwareMap hardwareMap) {
        this.slide = new Slide(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.joint = new Joint(hardwareMap);
        this.wrist = new Wrist(hardwareMap);
        this.basket = new Basket(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(joint);
        CommandScheduler.getInstance().registerSubsystem(slide);
    }
}
