package org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Core.Subsystems.servos.Basket;

public class SetBasket extends CommandBase {

    private final Basket basket;
    private double targetPosition;

    public SetBasket(Basket basket, double targetPosition) {
        this.basket = basket;
        this.targetPosition = targetPosition;

        addRequirements(basket);
    }

    @Override
    public void initialize() {
        basket.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
