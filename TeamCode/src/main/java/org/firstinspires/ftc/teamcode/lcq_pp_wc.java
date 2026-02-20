package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PoseStorage.currentPose;
import static java.lang.Math.PI;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
// Import the PIDController
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "lcq_pp_wc", group = "lcq")
public class lcq_pp_wc extends OpMode {
    private MotorEx launcher;
    private Motor intake;
    private ServoImplEx light;
    private ServoEx pitch;
    private ServoEx trigger;
    private ServoEx agitator;
    int currentAlliance = 0;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    IMU imu;
    private MotorEx yaw1 = null;
    double theta;
    public static double kS = 0;
    public static double kV = 0;
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    InterpLUT lut;
    double distanceToTarget;
    // Ticks per revolution for your yaw motor. Adjust if necessary.
    private final double TICKS_PER_REV_YAW = 2000.0;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            5, -5, 0, 14);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -85, 0, 0);
    boolean visionAvailable = false;
    boolean visionActive = false;
    Pose2D scorePose = new Pose2D(DistanceUnit.INCH, -58.3727, 55.6425, AngleUnit.DEGREES, 45);
    SparkFunOTOS.Pose2D startingPose;
    SparkFunOTOS myOtos;
    MecanumDrive mecanum;
    private boolean teleopPoseHasBeenSet = false;

    // --- NEW: PID Controller for Yaw ---
    private PIDController yawController;
    // --- TUNE THESE VALUES ---
    public static double yawKP = 0.03; // Proportional gain - start here
    public static double yawKI = 0;     // Integral gain - can often be left at 0
    public static double yawKD = 0;     // Derivative gain - helps with overshoot
    // --- END TUNE ---

    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        light = hardwareMap.get(ServoImplEx.class, "light");
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new MotorEx(hardwareMap, "yaw1");
        trigger = new ServoEx(hardwareMap, "trigger");
        agitator = new ServoEx(hardwareMap, "agitator");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");
        mecanum = new MecanumDrive(false, frontLeft, frontRight,
                backLeft, backRight);

        configureOtos();

        // --- NEW: Configure Yaw Motor for PID control ---
        yaw1.stopAndResetEncoder();
        // Set to RawPower to allow PID controller to set motor output
        yaw1.setRunMode(Motor.RunMode.RawPower);
        // Brake to hold position when not actively moving
        yaw1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // --- NEW: Initialize PID Controller ---
        yawController = new PIDController(yawKP, yawKI, yawKD);
        // This is the magic! It tells the PID controller that the input (angle) is continuous and wraps around.
        // It will now always choose the shortest path.
        // Set a tolerance for how close it needs to be to the target to be considered "at the setpoint".
        yawController.setTolerance(1.0); // Tolerance of 1 degree

        initAprilTag();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(11);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        frontLeft.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);
        frontRight.setInverted(true);

        launcher.setVeloCoefficients(0, .4, 0);

        intake.setInverted(true);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);

        lut = new InterpLUT();

        lut.add(0.0,   30.0);
        lut.add(60.0,  30.0);
        lut.add(85.0,  40.0);
        lut.add(110.0, 55.0);
        lut.add(135.0, 65.0);
        lut.add(204, 70.0);

        lut.createLUT();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

        telemetry.addLine("Press right for red alliance");
        telemetry.addLine("Press left for blue alliance");
        telemetry.addData("Selected Alliance", currentAlliance == 0 ? "Red" : "Blue");

        if (currentPose != null) {
            telemetry.addData("Current Pose", currentPose);
        } else {
            telemetry.addLine("pose not passed from auto");
        }


        if (gamepad1.dpadRightWasPressed()) {
            scorePose = new Pose2D(DistanceUnit.INCH, -58.3727, 55.6425, AngleUnit.DEGREES, 45);
            currentAlliance = 0;
            light.setPosition(.277);
        } else if (gamepad1.dpadLeftWasPressed()) {
            scorePose = new Pose2D(DistanceUnit.INCH, -58.3727, -55.6425, AngleUnit.DEGREES, 45);
            currentAlliance = 1;
            light.setPosition(.611);
        }

        if (visionAvailable && !visionActive && visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionActive = true;
            telemetry.addLine("Vision active, searching for tags to set backup pose...");
        }

        if (!teleopPoseHasBeenSet) {
            if (visionActive) {
                for (AprilTagDetection detection : aprilTag.getDetections()) {
                    if (detection.metadata != null) {
                        telemetry.addLine("SUCCESS: Found AprilTag, setting pose from vision!");

                        Pose3D tagRobotPose = detection.robotPose;
                        SparkFunOTOS.Pose2D visionStartPose = new SparkFunOTOS.Pose2D(
                                tagRobotPose.getPosition().x,
                                tagRobotPose.getPosition().y,
                                tagRobotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                        );

                        myOtos.setPosition(visionStartPose);
                        teleopPoseHasBeenSet = true;

                        break;
                    }
                }
            }

            if (!teleopPoseHasBeenSet) {
                telemetry.addLine("Waiting for Auto Pose or Vision Backup...");
            }

        } else {
            telemetry.addLine("TeleOp pose has been set. Ready to start!");
        }
    }

    @Override
    public void start() {

        if (visionAvailable && !visionActive && visionPortal != null) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                visionActive = true;
                telemetry.addLine("vision active");
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(200);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
            }
        }
    }

    @Override
    public void loop() {

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        double deltaX = scorePose.getX(DistanceUnit.INCH) - pos.x;
        double deltaY = scorePose.getY(DistanceUnit.INCH) - pos.y;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        AprilTagDetection tag24 = null;
        AprilTagDetection tag20 = null;

        double alpha = .5;

        boolean isRobotStopped;
        if (myOtos.getVelocity().x <= 1 && myOtos.getVelocity().y <= 1) {
            isRobotStopped = true;
        } else {
            isRobotStopped = false;
        }

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == 24 && detection.metadata != null && currentAlliance == 0 && visionActive) {
                tag24 = detection;

                if (isRobotStopped) {
                    if (Math.hypot(tag24.robotPose.getPosition().x - pos.x, tag24.robotPose.getPosition().y - pos.y) < 6.0) {
                        double newX = alpha * tag24.robotPose.getPosition().x + (1.0 - alpha) * pos.x;
                        double newY = alpha * tag24.robotPose.getPosition().y + (1.0 - alpha) * pos.y;
                        SparkFunOTOS.Pose2D newPose = new SparkFunOTOS.Pose2D(newX, newY, orientation.getYaw());
                        pos.set(newPose);
                    }
                }
            }

            if (detection.id == 20 && detection.metadata != null && currentAlliance == 1 && visionActive) {
                tag20 = detection;

                if (isRobotStopped) {
                    if (Math.hypot(tag20.robotPose.getPosition().x - pos.x, tag20.robotPose.getPosition().y - pos.y) < 6.0) {
                        double newX = alpha * tag20.robotPose.getPosition().x + (1.0 - alpha) * pos.x;
                        double newY = alpha * tag20.robotPose.getPosition().y + (1.0 - alpha) * pos.y;
                        SparkFunOTOS.Pose2D newPose = new SparkFunOTOS.Pose2D(newX, newY, orientation.getYaw());
                        pos.set(newPose);
                    }
                }
            }
        }

// --- FINAL, CORRECTED YAW CONTROL LOGIC ---

// 1. Calculate the absolute target angle for the turret in the world frame (degrees)
        double worldTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

// 2. Adjust for robot's heading to get the target angle relative to the robot's front
        double robotRelativeTargetAngle = AngleUnit.normalizeDegrees(worldTargetAngle - myOtos.getPosition().h);

// 3. Get the current angle of the turret from its encoder, also normalized
        double currentYawAngle = AngleUnit.normalizeDegrees(yaw1.getCurrentPosition() * (360.0 / TICKS_PER_REV_YAW));

// 4. THIS IS THE KEY: Calculate the error and normalize it to the shortest path (-180 to 180)
// This is the manual "continuous input" logic.
        double error = AngleUnit.normalizeDegrees(robotRelativeTargetAngle - currentYawAngle);

// 5. Calculate the setpoint for the PID controller.
// We tell the controller to move from the current angle to a new setpoint that represents the shortest path.
        double shortestPathSetpoint = currentYawAngle + error;

// 6. Calculate the power needed to get to that new, short-path setpoint
        double yawPower = yawController.calculate(currentYawAngle, shortestPathSetpoint);

// 7. Set the motor power
        yaw1.set(yawPower);

// --- End of Yaw Logic ---



        if (tag24 != null && currentAlliance == 0 && visionActive) {
            distanceToTarget = tag24.ftcPose.x;
            telemetry.addLine("tag detected");
            telemetry.addData("distance to target (in)", distanceToTarget);
        } else if (tag20 != null && currentAlliance == 1 && visionActive) {
            distanceToTarget = tag20.ftcPose.x;
            telemetry.addLine("tag detected");
            telemetry.addData("distance to target (in)", distanceToTarget);
        } else if (tag24 == null && tag20 == null) {
            distanceToTarget = Math.hypot(deltaX, deltaY);
        }

        theta = lut.get(distanceToTarget);
        pitch.set(theta);

        mecanum.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, myOtos.getPosition().h, true);

        if (gamepad1.dpadDownWasPressed()) {
            imu.resetYaw();
            myOtos.calibrateImu();
        }

        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV);

        if (gamepad1.left_trigger != 0) {
            launcher.setVelocity(1500);
            if (launcher.getVelocity() <= 1520 && launcher.getVelocity() >= 1480) {
                light.setPosition(.46);
            } else {
                light.setPosition(.666);
            }
        } else {
            launcher.set(0);
            light.setPosition(0);
        }

        if (gamepad1.right_bumper) {
            intake.set(1);
        } else if (gamepad1.a) {
            intake.set(-1);
        } else {
            intake.set(0);
        }

        if (gamepad1.leftBumperWasPressed()) {
            agitator.set(1);
        } else {
            agitator.set(0);
        }

        if (gamepad1.right_trigger != 0) {
            trigger.set(0);
            intake.set(1);
        } else if (isRobotInTriangle(pos.x, pos.y, 0, 0, -72, 72, -72, 72) && (launcher.getVelocity() <= 1520) && (launcher.getVelocity() >= 1480)) {
            trigger.set(0);
        } else if (isRobotInTriangle(pos.x, pos.y, 24, -24, 48, 0, 24, 24) && (launcher.getVelocity() <= 1520) && (launcher.getVelocity() >= 1480)){


            trigger.set(0);
        } else {
            intake.set(0);
            trigger.set(.2);
        }

        telemetry.addData("distance to target (in)", distanceToTarget);
        telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        telemetry.addData("LUT angle", theta);

        telemetry.setAutoClear(true);

        // Telemetry
        panelsTelemetry.addData("distance to target (in)", distanceToTarget);
        panelsTelemetry.addData("kS", kS);
        panelsTelemetry.addData("kV", kV);

        telemetry.update();

    }

    // Helper to check which side of a line a point is on
    private double crossProduct(double x1, double y1, double x2, double y2, double x3, double y3) {
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    }

    public boolean isRobotInTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
        double d1, d2, d3;
        boolean has_neg, has_pos;

        d1 = crossProduct(px, py, x1, y1, x2, y2);
        d2 = crossProduct(px, py, x2, y2, x3, y3);
        d3 = crossProduct(px, py, x3, y3, x1, y1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }
    private void initAprilTag() {
        try {

            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .build();

            // Create the WEBCAM vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

            visionAvailable = true;

        } catch (Exception e) {
            visionAvailable = false;
            telemetry.addData("Webcam not initialized", e.getMessage());
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.5, 3.75, Math.toDegrees(PI));
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(.984);
        myOtos.setAngularScalar(.999);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0,0,0);

        if (PoseStorage.currentPose != null) {
            telemetry.addLine("Pose received from Autonomous. Converting coordinates...");

            Pose pedroPose = PoseStorage.currentPose;
            Pose ftcPose = pedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);

            SparkFunOTOS.Pose2D otosStartPose = new SparkFunOTOS.Pose2D(
                    ftcPose.getX(),
                    ftcPose.getY(),
                    Math.toDegrees(ftcPose.getHeading())
            );

            myOtos.setPosition(otosStartPose);
            telemetry.addData("Successfully set OTOS start pose to", otosStartPose.toString());

            PoseStorage.currentPose = null;
            teleopPoseHasBeenSet = true;
        } else {
            telemetry.addLine("No pose passed from auto. Resetting OTOS tracking.");
            myOtos.setPosition(currentPosition);
            myOtos.resetTracking();
        }

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

}
