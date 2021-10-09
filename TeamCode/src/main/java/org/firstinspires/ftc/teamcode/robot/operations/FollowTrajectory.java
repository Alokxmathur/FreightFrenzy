package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * An operation to follow the specified road-runner trajectory
 */

public class FollowTrajectory extends Operation {
    public static final int DEFAULT_CORRECTION_COUNT = 1;

    protected Trajectory trajectory;
    MecanumDriveTrain driveTrain;
    protected int correctionCount = DEFAULT_CORRECTION_COUNT, retries;

    public FollowTrajectory(Trajectory trajectory, MecanumDriveTrain driveTrain, int retries, String title) {
        this(trajectory, driveTrain, title);
        this.correctionCount = retries;
    }

    public FollowTrajectory(Trajectory trajectory, MecanumDriveTrain driveTrain, String title) {
        this.trajectory = trajectory;
        this.driveTrain = driveTrain;
        this.title = title;
    }
    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public void setCorrectionCount(int correctionCount) {
        this.correctionCount = correctionCount;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Trajectory: %s->%s --%s",
                this.trajectory.start().toString(),
                this.trajectory.end().toString(),
                this.title);
    }

    public boolean isComplete() {
        driveTrain.update();
        //Match.log("Trajectory position: " + driveTrain.getPoseEstimate().toString());
        boolean busy = driveTrain.isBusy();
        if (!busy) {
            driveTrain.update();
            Pose2d currentPose = driveTrain.getPoseEstimate();
            String error = getError(currentPose, trajectory, retries);
            if (retries++ < correctionCount && (Math.abs(trajectory.end().getX()-currentPose.getX()) > MecanumDriveTrain.ACCEPTABLE_ERROR
                || Math.abs(trajectory.end().getY()-currentPose.getY())  > MecanumDriveTrain.ACCEPTABLE_ERROR))
            {
                trajectory = driveTrain.accurateTrajectoryBuilder(currentPose)
                        .splineToLinearHeading(trajectory.end(), 0)
                        .build();
                driveTrain.followTrajectory(trajectory);
                Match.log("Correcting error: " + error + " after trajectory");
                return false;
            }
            Match.log(String.format("Finished trajectory %s with error %s, at %s",
                    this.title,
                    error,
                    currentPose.toString()));
         }
        return !busy;
    }

    @Override
    public void startOperation() {
        this.driveTrain.handleOperation(this);
    }

    @Override
    public void abortOperation() {
        this.driveTrain.stop();
    }

    public static String getError(Pose2d currentPose, Trajectory trajectory, int retries) {
        return String.format("%.2f, %.2f, %.2f, retry: %d",
                        trajectory.end().getX()-currentPose.getX(),
                        trajectory.end().getY()-currentPose.getY(),
                        Math.toDegrees(
                                AngleUnit.normalizeRadians(trajectory.end().getHeading())
                                        - AngleUnit.normalizeRadians(currentPose.getHeading())),
                        retries);
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}
