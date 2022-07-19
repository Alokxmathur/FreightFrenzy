package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;

public class RobotConfig {
    //drive train motors
    public static final String LEFT_FRONT_DRIVE = "leftFrontDrive";
    public static final String LEFT_REAR_DRIVE = "leftRearDrive";
    public static final String RIGHT_REAR_DRIVE = "rightRearDrive";
    public static final String RIGHT_FRONT_DRIVE = "rightFrontDrive";
    public static final String CAROUSEL_MOTOR = "carousel";
    public static final String WEBCAM_ID = "Webcam 1";
    public static final String BLINKIN = "blinkin";

    public static final String IN_MOTOR = "inMotor";
    public static final String IN_SERVO = "inServo";
    public static final String IN_SENSOR = "inSensor";

    public static final String OUT_SHOULDER = "outShoulder";
    public static final String OUT_ELBOW = "outElbow";
    public static final String BUCKET_LID = "outGripper";
    public static final String TURRET_MOTOR = "turretMotor";
    public static final String WINCH = "winch";

    public static double ROBOT_CENTER_FROM_BACK = (2.75 + DriveConstants.TRACK_LENGTH/2) *Field.MM_PER_INCH;
    public static double ROBOT_CENTER_FROM_FRONT = (3.5 + DriveConstants.TRACK_LENGTH/2) *Field.MM_PER_INCH;
    public static final double ROBOT_WIDTH = 11.325 * Field.MM_PER_INCH;
    public static final double ROBOT_LENGTH = ROBOT_CENTER_FROM_BACK + ROBOT_CENTER_FROM_FRONT;
    public static final double ALLOWED_BEARING_ERROR = 0.5;
    public static final double ALLOWED_POSITIONAL_ERROR = .25;
    public static final double SUPER_CAUTIOUS_SPEED = 0.4;
    public static final double REGULAR_SPEED = .8;

    public static final double INTAKE_SPEED = -1.0;
    public static final double INTAKE_SERVO_INCREMENT = .001;
    public static final double INTAKE_LOWERED_POSITION = .615;
    public static final double INTAKE_RAISED_POSITION = .32;
    public static final double INTAKE_CONSUME_POSITION = .5;
    public static final double MAX_IN_SPEED = 1.0;

    public static final double MAX_CAROUSEL_SPEED = 0.08;
    public static final long CAROUSEL_SPINNER_REQUIRED_TIME = 2000; //2 seconds to spin carousel for delivery

    public static final int OUTPUT_SHOULDER_INCREMENT = 20;
    public static final double OUT_SHOULDER_SPEED = 1.0;

    public static final int OUTPUT_SHOULDER_INITIAL_POSITION = 0;
    public static final int OUTPUT_SHOULDER_INTAKE_POSITION = 74;
    public static final int OUTPUT_SHOULDER_TOP_POSITION = 565;
    public static final int OUTPUT_SHOULDER_MIDDLE_POSITION = 420;
    public static final int OUTPUT_SHOULDER_BOTTOM_POSITION = 250;
    public static final int OUTPUT_SHOULDER_LLAMA_POSITION = 746;
    public static final int OUTPUT_SHOULDER_VERTICAL_POSITION = 800;
    public static final int OUTPUT_SHOULDER_PICKUP_POSITION = 300;

    public static final double OUTPUT_ELBOW_INCREMENT = .01;
    public static final double OUTPUT_ELBOW_INITIAL_POSITION = .8;
    public static final double OUTPUT_ELBOW_INTAKE_POSITION = .92;
    public static final double OUTPUT_ELBOW_TOP_POSITION = 0;
    public static final double OUTPUT_ELBOW_MIDDLE_POSITION = 0.07;
    public static final double OUTPUT_ELBOW_BOTTOM_POSITION = 0.27;
    public static final double OUTPUT_ELBOW_LLAMA_POSITION = 1;
    public static final double OUTPUT_ELBOW_PICKUP_POSITION = .01;
    public static final double OUTPUT_ELBOW_VERTICAL_POSITION = .6;
    public static final double OUTPUT_ELBOW_CAPPING_POSITION = 0.2;

    public static final double OUTPUT_LID_CLOSED_POSITION = .26;
    public static final double OUTPUT_LID_OPEN_POSITION = .6;
    public static final double OUTPUT_LID_CAPPING_POSITION = 0;
    public static final double OUTPUT_LID_INCREMENT = .01;

    public static final long SERVO_REQUIRED_TIME = 200; //200 milli-seconds for servo to function
}
