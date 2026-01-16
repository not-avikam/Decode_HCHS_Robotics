package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "new bot axon", group = "lm2 2025")
public class NEW_BOT_axon extends OpMode {

    // Adjustable shot parameters
    public static double distanceToTarget = 60; // inches
    public static double launchAngleDeg = 45;   // degrees
    double hueIntake;
    private double adder = 200;
    NormalizedColorSensor colorSensorIntake;
    private MotorEx launcher;
    private CRServoEx intake;
    private Servo agigtator;
    private ServoEx indexer;
    private ServoEx pitch;
    private double goalX = 132;
    private double goalY = 136;

    double[] INTAKE_POS = {0, 130, 270};
    double[] SHOOT_POS = {162, 228, 295.5};

    int i = 0;
    int s = 0;
    int currentAlliance = 0;

    double velocityIPS = 0;
    double velocityTPS = 0;
    private TelemetryManager telemetryM;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Motor frontLeftDrive = null;
    private Motor backLeftDrive = null;
    private Motor frontRightDrive = null;
    private Motor backRightDrive = null;
    IMU imu;
    private MotorEx yaw1 = null;
    double height;
    public static double TICKS_PER_REV = 384.5; // adjust for gearing
    public static double TURRET_DEADBAND_DEG = 1.0;
    public double turretTargetDeg = 0;
    private Follower follower;
    private Pose startPose;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    double atX;
    double atY;
    double imuHeading;
    double pitchAngle = .12;
    MecanumDrive mecanum;
    GamepadEx driverOp;
    boolean visionAvailable = false;
    boolean visionActive = false;
    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new CRServoEx(hardwareMap, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch");
        frontLeftDrive = hardwareMap.get(Motor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(Motor.class, "backLeft");
        frontRightDrive = hardwareMap.get(Motor.class, "frontRight");
        backRightDrive = hardwareMap.get(Motor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new MotorEx(hardwareMap, "yaw1");
        driverOp = new GamepadEx(gamepad1);

        // input motors exactly as shown below
        mecanum = new MecanumDrive(frontLeftDrive, frontRightDrive,
                backLeftDrive, backRightDrive);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // set the run mode
        yaw1.setRunMode(MotorEx.RunMode.RawPower);
        yaw1.stopAndResetEncoder();
        yaw1.setPositionCoefficient(0.05);

        frontLeftDrive.setInverted(false);
        frontRightDrive.setInverted(true);
        backLeftDrive.setInverted(false);
        backRightDrive.setInverted(true);

        intake.setInverted(false);
        //agigtator.setDirection(Servo.Direction.REVERSE);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(0.6, 0, 0);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        startPose = new Pose(119.53,70.2, Math.toRadians(90));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        indexer.set(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        initAprilTag();

        //purpleSoundID = hardwareMap.appContext.getResources().getIdentifier("purple", "raw", hardwareMap.appContext.getPackageName());
        //greenSoundID = hardwareMap.appContext.getResources().getIdentifier("green",   "raw", hardwareMap.appContext.getPackageName());
        //gongID = hardwareMap.appContext.getResources().getIdentifier("velocity", "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        //if (greenSoundID != 0)
        //  greenFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, greenSoundID);

        //if (purpleSoundID != 0)
        //  purpleFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, purpleSoundID);

        //if (gongID != 0)
        //  gongFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, gongID);

        // Display sound status
        //telemetry.addData("green resource",   greenFound ?   "Found" : "NOT found\n Add green.wav to /src/main/res/raw" );
        //telemetry.addData("purple resource", purpleFound ? "Found" : "NOT found\n Add purple.wav to /src/main/res/raw" );
        //telemetry.addData("gong resource", gongFound ? "Found": "NOT found\n Add gong.wav to /src/main/res/raw");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

        telemetry.addLine("Press right for red alliance");
        telemetry.addLine("Press left for blue alliance");
        telemetry.addData("Selected Alliance", currentAlliance == 0 ? "Red" : "Blue");

        if (gamepad1.dpadRightWasPressed()) {
            goalX = 132;
            goalY = 136;
            currentAlliance = 0;
        } else if (gamepad1.dpadLeftWasPressed()) {
            goalX = 12;
            goalY = 136;
            currentAlliance = 1;
        }

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        if (visionAvailable && !visionActive && visionPortal != null) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                visionActive = true;
                telemetry.addLine("vision active");
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(255);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            }
        }

        follower.update();

        mecanum.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRobotYawPitchRollAngles().getYaw(),   // gyro value passed in here must be in degrees
                false
        );

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetryM.update();
        //distanceToTarget = Math.sqrt((goalX - follower.getPose().getX()) * (goalX - follower.getPose().getX()) + (goalY - follower.getPose().getY()) * (goalY - follower.getPose().getY()));
        height = 42;

        AprilTagDetection tag24 = null;
        AprilTagDetection tag20 = null;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == 24 && detection.metadata != null && currentAlliance == 0) {
                tag24 = detection;
                break; // lock onto ONLY tag 24
            }

            if (detection.id == 20 && detection.metadata != null && currentAlliance == 1) {
                tag20 = detection;
                break;
            }

        }

        if (tag24 != null && currentAlliance == 0) {
            atX = tag24.ftcPose.x;
            atY = tag24.ftcPose.y;
            imuHeading = orientation.getYaw(AngleUnit.DEGREES);

            yaw1.set(-tag24.ftcPose.bearing*.02);

            distanceToTarget = tag24.ftcPose.range;
            if (gamepad1.left_trigger !=0) {
                pitchAngle = .12;
            } else {
                pitchAngle = tag24.ftcPose.elevation;
            }
            follower.setPose(getRobotPoseFromCamera());
        } else if (tag20 != null && currentAlliance == 1) {
            atX = tag20.ftcPose.x;
            atY = tag20.ftcPose.y;
            imuHeading = orientation.getYaw(AngleUnit.DEGREES);

            yaw1.set(-tag20.ftcPose.bearing*.02);

            distanceToTarget = tag20.ftcPose.range;
            if (gamepad1.left_trigger !=0) {
                pitchAngle = .12;
            } else {
                pitchAngle = tag20.ftcPose.elevation;
            }
            follower.setPose(getRobotPoseFromCamera());
        } else {
            yaw1.set(driverOp.getRightY());
        }

        pitch.set(pitchAngle);

        if (driverOp.wasJustPressed(GamepadKeys.Button.B)) {
            imu.resetYaw();
        }

        // Convert angle to radians
        double theta = Math.toRadians(launchAngleDeg);

        // Physics denominator
        double denom =
                2 * Math.pow(Math.cos(theta), 2) *
                        (distanceToTarget * Math.tan(theta) - height);

        // Compute required IPS safely
        if (denom > 0 && distanceToTarget > 0) {
            velocityIPS = Math.sqrt(
                    (386.4 * distanceToTarget * distanceToTarget) / denom
            );
        } else {
            velocityIPS = 0;
        }

        // IPS -> TPS
        if (velocityIPS > 0) {
            velocityTPS = Math.log(velocityIPS / 69.9) / 0.000821;
        } else {
            velocityTPS = 0;
        }

        // Fire launcher
        if (gamepad1.left_trigger != 0 && Double.isFinite(velocityTPS)) {
            launcher.setVelocity((velocityTPS + adder)*driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else if (tag24 == null && tag20 == null){
            launcher.setVelocity(2000*(driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        }


        // Agitator
        if (gamepad1.right_trigger != 0) {
            agigtator.setPosition(0);
        } else {
            agigtator.setPosition(0.3);
        }

        // Adjust distance
        if (gamepad1.yWasPressed()) {
            adder += 5;
        } else if (gamepad1.aWasPressed()) {
            adder -= 5;
        }

        // Indexer control
        if (gamepad1.dpadRightWasPressed()) {
            if (i < INTAKE_POS.length - 1) i++;
            indexer.set(INTAKE_POS[i]);
        } else if (gamepad1.dpadLeftWasPressed()) {
            if (i > 0) i--;
            indexer.set(INTAKE_POS[i]);
        } else if (gamepad1.dpadUpWasPressed()) {
            if (s < SHOOT_POS.length - 1) s++;
            indexer.set(SHOOT_POS[s]);
        } else if (gamepad1.dpadDownWasPressed()) {
            if (s > 0) s--;
            indexer.set(SHOOT_POS[s]);
        }

        if (gamepad1.x) {
            intake.set(1);
        } else {
            intake.set(0);
        }

        telemetry.addData("distance to target (in)", distanceToTarget);
        telemetry.addData("target velocity IPS", velocityIPS);
        telemetry.addData("target velocity TPS", velocityTPS);
        telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        telemetry.addData("additive", adder);
        telemetry.addData("imu data", imu.getRobotYawPitchRollAngles());
        telemetry.addData("hue", hueIntake);
        telemetry.addData("bearing", currentAlliance == 0 ? Objects.requireNonNull(tag24).ftcPose.bearing : Objects.requireNonNull(tag20).ftcPose.bearing);


        // Telemetry
        panelsTelemetry.addData("Distance (in)", distanceToTarget);
        panelsTelemetry.addData("Velocity IPS", velocityIPS);
        panelsTelemetry.addData("Velocity TPS", velocityTPS);
        panelsTelemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        panelsTelemetry.addData("Adder", adder);
        panelsTelemetry.addData("Color Sensor Reading", hueIntake);
        panelsTelemetry.update(telemetry);

        telemetry.update();

    }
    private double getTurretAngleDeg() {
        return (yaw1.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(atX, atY, imuHeading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    private void initAprilTag() {
        try {

            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            // Create the WEBCAM vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

        } catch (Exception e) {
            visionAvailable = false;
            telemetry.addData("Error", e.getMessage());
        }
    }

}