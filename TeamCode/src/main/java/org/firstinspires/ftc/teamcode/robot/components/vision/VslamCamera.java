package org.firstinspires.ftc.teamcode.robot.components.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.jetbrains.annotations.NotNull;

import java.util.Locale;
import java.util.function.Consumer;

/**
 * This class implements our VSLAM camera. It holds on to an instance of the camera in a
 * static class member. It also maintains other static variables like the current position,
 * velocity etc.
 *
 * One should call start and stop on this class to start and stop the camera. The constructor
 * starts the camera.
 *
 * This class implements the localizer interface of RoadRunner to be able to be used in that
 * library.
 *
 * It also implements the T265Camera.CameraUpdate interface to accept position updates from the
 * camera in an asynchronous manner.
 */

public class VslamCamera implements Localizer, Consumer<T265Camera.CameraUpdate> {
    //Remember all measurements for ftc-lib geometry are in meters

    //Our camera sticks out about 1.75 inches beyond the center of the front wheels
    public static final double CENTER_OFFSET_X =
            -(MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 + 1.75*Field.MM_PER_INCH) / 1000;
    //How far left of the camera the center is
    public static final double CENTER_OFFSET_Y = 3*Field.M_PER_INCH;
    public static final double T265_ROTATION = 0;
    private static final Transform2d robotCenterOffsetFromCamera =
            new Transform2d(new Translation2d(CENTER_OFFSET_X, CENTER_OFFSET_Y),
                    new Rotation2d(Math.toRadians(T265_ROTATION)));

    //we maintain a static camera instance
    private static T265Camera t265Camera = null;
    private Pose2d currentPose = new Pose2d();
    private static T265Camera.CameraUpdate lastCameraUpdate;
    private Pose2d pose2dVelocity = new Pose2d();
    private final Object synchronizationObject = new Object();
    private volatile boolean isInitialized = false;

    /**
     * Constructor. Note that you call this constructor with the hardwareMap to get an instance
     * of the vslam camera. The constructor creates a thread to create the instance and then to
     * start it.
     * @param hardwareMap - hardware map
     */
    public VslamCamera(HardwareMap hardwareMap) {
        Thread cameraInitializationThread = new Thread(() -> {
            synchronized (synchronizationObject) {
                if (t265Camera == null) {
                    Match.log("Creating new T265 camera");
                    t265Camera = new T265Camera(robotCenterOffsetFromCamera, 1.0, hardwareMap.appContext);
                }
                start();
                isInitialized = true;
            }
        });
        Match.log("Starting vslam initialization thread");
        cameraInitializationThread.start();
    }

    /**
     * Set the current position of the robot. We ask the t265 camera to set the pose
     * @param pose the pose to set
     */
    public synchronized boolean setCurrentPose(Pose2d pose) {
        synchronized (synchronizationObject) {
            if (lastCameraUpdate == null) {
                Match.log("Unable to set pose as no update has been received from camera");
                return true;
            }
            else if (lastCameraUpdate.confidence == T265Camera.PoseConfidence.Low){
                Match.log("Unable to set pose as confidence level is "
                        + lastCameraUpdate.confidence.toString());
                return false;
            }
            else {
                t265Camera.setPose(Field.roadRunnerToCameraPose(pose));
                currentPose = pose;
                Match.log(
                        "Set pose to " + pose
                                + ", while confidence was: " + lastCameraUpdate.confidence.toString());
                return true;
            }
        }
    }

    public synchronized boolean isInitialized() {
        synchronized (synchronizationObject) {
            return isInitialized;
        }
    }

    //Methods to implement roadrunner localizer

    /**
     * Returns the current pose. The pose units of measure are inches in x and y coordinates and
     * radians for the heading.
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    /**
     * Implementation of the update method required by the Localizer interface.
     * We don't do anything here as the updates are handled in an asynchronous manner by implementing
     * the accept method of the T265Camera interface
     */
    @Override
    public void update() {
    }
    /**
     * Method to set current pose estimate, as required by the RoadRunner interface. Since we
     * are using a VSLAM camera, we don't do anything here as we are totally dependent on the
     * camera provided positions.
     */
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
    }

    @Override
    public Pose2d getPoseVelocity() {
        return pose2dVelocity;
    }

    public void stop() {
        if (t265Camera.isStarted()) {
            Match.log("Stopping T265 camera");
            t265Camera.stop();
        }
        else {
            Match.log("T265 Camera was already stopped");
        }
    }

    /**
     * Method to start the VSLAM camera. We catch the "Camera is already started" exception just in
     * case this method gets called while the camera was already running.
     */
    public void start() {
        synchronized (synchronizationObject) {
            try {//start our camera
                if (!t265Camera.isStarted()) {
                    Match.log("Starting T265 camera");
                    t265Camera.start(this);
                } else {
                    Match.log("T265 Camera was already running");
                }
            }
            catch(Throwable e){
                RobotLog.logStackTrace("SilverTitans: Error starting real sense", e);
                throw e;
            }
        }
    }


    /**
     * Accept the asynchronous update sent by the camera.
     * We calculate our current pose by adding the poseTranslation to the camera provided
     * pose. Our current pose is kept in inches and radians. We also maintain our pose velocity
     * by converting the velocity provided to inches/second.
     * @param cameraUpdate - the camera update provided by the camera
     */
    @Override
    public void accept(T265Camera.CameraUpdate cameraUpdate) {
        synchronized (synchronizationObject) {
            //process latest received update
            try {
                lastCameraUpdate = cameraUpdate;
                //only update our position if the confidence level is not low
                if (cameraUpdate.confidence != T265Camera.PoseConfidence.Low) {
                    pose2dVelocity = new Pose2d(
                            cameraUpdate.velocity.vxMetersPerSecond / Field.M_PER_INCH,
                            cameraUpdate.velocity.vyMetersPerSecond / Field.M_PER_INCH,
                            cameraUpdate.velocity.omegaRadiansPerSecond);
                    //set last pose to be the current one
                    currentPose = Field.cameraToRoadRunnerPose(cameraUpdate.pose);
                }
                else {
                    Match.log("Did not accept camera update because the confidence was "
                            + cameraUpdate.confidence.toString());
                }
            } catch (Throwable e) {
                RobotLog.ee("Vslam", e, "Error getting position");
            }
        }
    }

    public String getStatus() {
        synchronized (synchronizationObject) {
            if (havePosition()) {
                return String.format(Locale.getDefault(), "Pose:%s, Confidence:%s",
                        Field.cameraToRoadRunnerPose(lastCameraUpdate.pose).toString(),
                        lastCameraUpdate.confidence.toString());
            }
            else {
                return "No camera update yet";
            }
        }
    }


    public boolean havePosition() {
        return lastCameraUpdate != null;
    }
}
