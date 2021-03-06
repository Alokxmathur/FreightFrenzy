package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class TurnOperation extends Operation {

    public enum Direction {
        LEFT, RIGHT
    }

    public double getDegrees() {
        return degrees;
    }

    public void setDegrees(double degrees) {
        this.degrees = degrees;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    protected double degrees;
    private double speed;
    private Direction direction;
    MecanumDriveTrain driveTrain;

    public TurnOperation(double degrees, double speed, Direction direction, MecanumDriveTrain driveTrain, String title) {
        this.title = title;
        this.degrees = degrees;
        this.speed = speed;
        this.direction = direction;
        this.driveTrain = driveTrain;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"Turn degrees: %.2f@%.2f,%s --%s",
                this.degrees, this.speed, this.direction, this.title);
    }

    public boolean isComplete() {
        if (direction == Direction.RIGHT) {
            return driveTrain.rightWithinRange();
        }
        else {
            return driveTrain.leftWithinRange();
        }
    }

    @Override
    public void startOperation() {
        driveTrain.handleOperation(this);
    }

    @Override
    public void abortOperation() {
        driveTrain.stop();
    }
}
