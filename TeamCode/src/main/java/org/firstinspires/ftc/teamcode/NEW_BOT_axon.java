package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

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

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "new bot axon", group = "lm2 2025")
public class NEW_BOT_axon extends OpMode {
    // Adjustable shot parameters
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
    public static double height = 58;
    public static double TICKS_PER_REV = 384.5; // adjust for gearing
    public static double TURRET_DEADBAND_DEG = 1.0;
    public double turretTargetDeg = 0;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    double pitchAngle = 165;
    MecanumDrive mecanum;
    GamepadEx driverOp;
    boolean visionAvailable = false;
    boolean visionActive = false;
    private Follower follower;
    private Pose scorePose;
    private Pose startingPose = new Pose(109.620, 94.685, Math.toRadians(270));;
    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new CRServoEx(hardwareMap, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        frontLeftDrive = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        backLeftDrive = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);
        frontRightDrive = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        backRightDrive = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new MotorEx(hardwareMap, "yaw1");
        driverOp = new GamepadEx(gamepad1);

        frontLeftDrive.setRunMode(Motor.RunMode.RawPower);
        frontRightDrive.setRunMode(Motor.RunMode.RawPower);
        backLeftDrive.setRunMode(Motor.RunMode.RawPower);
        backRightDrive.setRunMode(Motor.RunMode.RawPower);

        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);

        mecanum = new MecanumDrive(
                false,
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive
        );


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

        intake.setInverted(false);
        //agigtator.setDirection(Servo.Direction.REVERSE);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(0.6, 0, 0);
        //launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        indexer.set(0);
        pitch.set(0);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
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
            scorePose = new Pose(132.12651646447142, 136.18370883882147);
            currentAlliance = 0;
        } else if (gamepad1.dpadLeftWasPressed()) {
            scorePose = new Pose(11.585788561525153, 136.4332755632582);
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
                gainControl.setGain(200);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
            }
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Robot Centric
        );

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetryM.update();

        double theta = Math.toRadians(45);
        double deltaX = follower.getPose().getX() - scorePose.getX();
        double deltaY = follower.getPose().getY() - scorePose.getY();
        double distanceToTarget = Math.sqrt(Math.pow(deltaX, 2) - Math.pow(deltaY, 2));

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

        if (velocityIPS > 0) {
            velocityTPS = Math.log(velocityIPS / 69.9) / 0.000821;
        } else {
            velocityTPS = 0;
        }

        if (gamepad1.left_trigger !=0) {
            launcher.setVelocity(velocityTPS*gamepad1.left_trigger);
            pitch.set(pitchAngle);
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
        telemetry.addData("stick left Y", driverOp.getLeftY());
        telemetry.addData("stick left X", driverOp.getLeftX());
        telemetry.addData("stick right X", driverOp.getRightX());
        //telemetry.addData("bearing", currentAlliance == 0 ? Objects.requireNonNull(tag24).ftcPose.bearing : Objects.requireNonNull(tag20).ftcPose.bearing);


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

            visionAvailable = true;

        } catch (Exception e) {
            visionAvailable = false;
            telemetry.addData("Webcam not initialized", e.getMessage());
        }
    }

}