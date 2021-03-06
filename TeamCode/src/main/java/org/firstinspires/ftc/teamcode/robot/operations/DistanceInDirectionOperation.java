package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DistanceInDirectionOperation extends DistanceOperation {
    private double distance;
    private double speed;
    double direction;
    MecanumDriveTrain driveTrain;

    public DistanceInDirectionOperation(double travelDistance, double heading,
                                        double speed, MecanumDriveTrain driveTrain, String title) {
        super(travelDistance, travelDistance, driveTrain, title);
        this.distance = travelDistance;
        this.speed = speed;
        this.direction = heading;
        this.driveTrain = driveTrain;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StraightLine: %.2f(%.2f\")@%.2f --%s",
                this.distance, (this.distance / Field.MM_PER_INCH), this.direction,
                this.title);
    }

    public boolean isComplete() {
        double currentBearing = Math.toDegrees(driveTrain.getExternalHeading());
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            return true;
        } else {
            // adjust relative speed based on heading error.
            double bearingError = AngleUnit.normalizeDegrees(direction - currentBearing);
            double steer = MecanumDriveTrain.getSteer(bearingError, MecanumDriveTrain.P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;
            double speedToUse = new Date().getTime() - this.getStartTime().getTime() < 500 ? 0.1 : speed;
            double leftSpeed = speedToUse - steer;
            double rightSpeed = speedToUse + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            driveTrain.setLeftFrontPower(leftSpeed);
            driveTrain.setLeftRearPower(leftSpeed);
            driveTrain.setRightFrontPower(rightSpeed);
            driveTrain.setRightRearPower(rightSpeed);

            return false;
        }
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getDistance() {
        return this.distance;
    }
}

