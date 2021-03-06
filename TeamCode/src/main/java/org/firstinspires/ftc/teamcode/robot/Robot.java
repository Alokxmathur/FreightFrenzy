package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.CappingArm;
import org.firstinspires.ftc.teamcode.robot.components.CarouselSpinner;
import org.firstinspires.ftc.teamcode.robot.components.InOutTake;
import org.firstinspires.ftc.teamcode.robot.components.LED;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.robot.components.vision.VslamCamera;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;

/**
 * This class represents our robot.
 * The config on the robot needs to have the following entries defined:
 * *
 * rightDrive: the right motor of the drive train
 * leftDrive: the left motor of the drive tpenrain
 */

public class Robot {

    Telemetry telemetry;
    private HardwareMap hardwareMap;
    Match match;

    OperationThread operationThreadPrimary;
    OperationThread operationThreadSecondary;
    OperationThread operationThreadTertiary;

    MecanumDriveTrain mecanumDriveTrain;
    CarouselSpinner carouselSpinner;
    InOutTake inOutTake;
    CappingArm cappingArm;
    org.firstinspires.ftc.teamcode.robot.components.LED led;
    //WebCam webcam;
    OpenCVWebcam webcam;
    VslamCamera vslamCamera;

    boolean everythingButCamerasInitialized = false;

    //Our sensors etc.

    //our state
    String state = "pre-initialized";

    public Robot() {
        Log.d("SilverTitans", "Robot: got created");
    }

    /**
     * Initialize our robot
     * We set our alliance and our starting position based on finding a VuMark
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Match match) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.match = match;

        //initialize our components
        initCameras(match.getAlliance(), match.getStartingPosition());
        initDriveTrain();
        this.carouselSpinner = new CarouselSpinner(hardwareMap);
        this.inOutTake = new InOutTake(hardwareMap);
        this.cappingArm = new CappingArm(hardwareMap);
        this.led = new LED(hardwareMap);
        if (match.getAlliance() == Alliance.Color.RED) {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        telemetry.addData("Status", "Creating operations thread, please wait");
        telemetry.update();

        Match.log("Started operations threads");
        this.operationThreadPrimary = new OperationThread(this, "Primary");
        operationThreadPrimary.start();
        this.operationThreadSecondary = new OperationThread(this, "Secondary");
        operationThreadSecondary.start();
        this.operationThreadTertiary = new OperationThread(this, "Tertiary");
        operationThreadTertiary.start();

        this.everythingButCamerasInitialized = true;
    }

    public void initDriveTrain() {
        //Create our drive train
        telemetry.addData("Status", "Initializing drive train, please wait");
        telemetry.update();
        this.mecanumDriveTrain = new MecanumDriveTrain(hardwareMap, telemetry, vslamCamera);
    }

    public void initCameras(Alliance.Color allianceColor, Field.StartingPosition startingPosition) {
        //initialize webcam
        Match.log("Initializing Webcam");
        telemetry.addData("Status", "Initializing Webcam, please wait");
        telemetry.update();
        this.webcam = new OpenCVWebcam();
        this.webcam.init(hardwareMap, telemetry, OpenCVWebcam.ELEMENT_COLOR_MIN, OpenCVWebcam.ELEMENT_COLOR_MAX);

        //initialize Vslam camera
        Match.log("Initializing VSLAM");
        telemetry.addData("Status", "Initializing VSLAM, please wait");
        telemetry.update();
        this.vslamCamera = new VslamCamera(hardwareMap);
    }


    /**
     * Stop the robot
     */
    public void stop() {
        //Stop all of our motors
        Match.log("Stopping robot");
        this.operationThreadPrimary.abort();
        this.operationThreadSecondary.abort();
        this.operationThreadTertiary.abort();
        this.mecanumDriveTrain.stop();

        this.cappingArm.stop();
        this.inOutTake.stop();
        Match.log(("Robot stopped"));
    }

    /**
     * Returns a string representing the status of the motors of the robot
     *
     * @return Motor Status
     */
    public String getMotorStatus() {
        if (this.mecanumDriveTrain == null) {
            return "Drivetrain not initialized";
        }
        else {
            return this.mecanumDriveTrain.getStatus();
        }
    }

    public void queuePrimaryOperation(Operation operation) {
        this.operationThreadPrimary.queueUpOperation(operation);
    }
    public void queueSecondaryOperation(Operation operation) {
        this.operationThreadSecondary.queueUpOperation(operation);
    }
    public void queueTertiaryOperation(Operation operation) {
        this.operationThreadTertiary.queueUpOperation(operation);
    }

    /**
     * Returns the current x value of robot's center in mms
     * @return the current x position in mms
     */
    public double getCurrentX() {
        return this.vslamCamera.getPoseEstimate().getX()*Field.MM_PER_INCH;
    }

    /**
     * Returns the current y value of robot's center in mms
     * @return the current y position in mms
     */
    public double getCurrentY() {
        return this.vslamCamera.getPoseEstimate().getY()*Field.MM_PER_INCH;
    }

    /**
     * Returns the current heading of the robot in radians
     * @return the heading in radians
     */
    public double getCurrentTheta() {
        return AngleUnit.normalizeRadians(this.vslamCamera.getPoseEstimate().getHeading());}

    public boolean allOperationsCompleted() {
        return primaryOperationsCompleted() && secondaryOperationsCompleted() && tertiaryOperationsCompleted();
    }

    public boolean primaryOperationsCompleted() {
        return !this.operationThreadPrimary.hasEntries();
    }

    public boolean secondaryOperationsCompleted() {
        return !this.operationThreadSecondary.hasEntries();
    }

    public boolean tertiaryOperationsCompleted() {
        return !this.operationThreadTertiary.hasEntries();
    }

    public String getPosition() {
        return this.vslamCamera.getPoseEstimate().toString();
    }

    public boolean havePosition() {
        return vslamCamera.havePosition();
    }

    public String getState() {
        return this.state;
    }

    public void setState(String state) {
        this.state = state;
    }

    public boolean fullyInitialized() {
        return this.everythingButCamerasInitialized && this.vslamCamera.isInitialized();
    }

    public void handleDriveTrain(Gamepad gamePad1) {
        if (this.primaryOperationsCompleted()) {
            double multiplier = 0.6;
            double x = Math.pow(gamePad1.left_stick_x, 3) * multiplier; // Get left joystick's x-axis value.
            double y = -Math.pow(gamePad1.left_stick_y, 3) * multiplier; // Get left joystick's y-axis value.

            double rotation = Math.pow(gamePad1.right_stick_x, 3) * 0.5; // Get right joystick's x-axis value for rotation

            this.mecanumDriveTrain.drive(Math.atan2(x, y), Math.hypot(x, y), rotation);
        }
    }

    public void handleGameControllers(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.x) {
            this.operationThreadPrimary.abort();
            this.operationThreadSecondary.abort();
            this.operationThreadTertiary.abort();
        }
        this.carouselSpinner.setSpeed(-gamePad2.left_stick_y);
        this.inOutTake.setSpeed(-gamePad2.right_stick_y);
        this.handleCappingArm(gamePad2);
        this.handleDriveTrain(gamePad1);
    }

    public void handleCappingArm(Gamepad gamepad) {
        //raise arm if right stick is pushed forward enough
        if (gamepad.dpad_up) {
            cappingArm.raiseArm();
        }
        else if (gamepad.dpad_down) {
            cappingArm.lowerArm();
        }

        if (gamepad.left_trigger > 0) {
            cappingArm.windServo();
        }
        else if (gamepad.right_trigger > 0) {
            cappingArm.unwindServo();
        }
        else {
            cappingArm.stopServo();
        }
    }

    public boolean setInitialPose(Pose2d pose) {
        this.mecanumDriveTrain.setLocalizer(vslamCamera);
        return this.vslamCamera.setCurrentPose(pose);
    }

    public void reset() {
        if (this.mecanumDriveTrain != null) {
            this.mecanumDriveTrain.ensureWheelDirection();
        }
        startVSLAM();
        this.webcam.start();
    }

    public Pose2d getPose() {
        return this.vslamCamera.getPoseEstimate();
    }

    public void startVSLAM() {
        this.vslamCamera.start();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return this.mecanumDriveTrain.trajectoryBuilder(startPose);
    }
    public TrajectoryBuilder accurateTrajectoryBuilder(Pose2d startPose) {
        return this.mecanumDriveTrain.accurateTrajectoryBuilder(startPose);
    }

    public OpenCVWebcam getWebcam() {
        return this.webcam;
    }

    public MecanumDriveTrain getDriveTrain() {
        return this.mecanumDriveTrain;
    }

    public void setCarouselSpinnerSpeed(double speed) {
        this.carouselSpinner.setSpeed(speed);
    }

    public String getCappingArmStatus() {
        return this.cappingArm.getStatus();
    }

    public String getCarouselStatus() {
        return this.carouselSpinner.getStatus();
    }

    public String getInoutStatus() {
        return this.inOutTake.getStatus();
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.led.setPattern(pattern);
    }
    public CarouselSpinner getCarouselSpinner() {
        return this.carouselSpinner;
    }

    public String getVSLAMStatus() {
        return this.vslamCamera.getStatus();
    }

    public int getBarCodeLevel() {
        return this.webcam.getBarCodeLevel();
    }
}
