package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Configurable
@TeleOp(name = "lm2 teleop drive", group = "lm2 2025")
public class launcher_velocity_calculator extends OpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static double velocity_servo = 0;
    public static double velocity_intake = 0;
    public static double pitch_position = 0;

    // Adjustable shot parameters
    public static double distanceToTarget = 60; // inches
    public static double launchAngleDeg = 45;   // degrees
    double hueIntake, hueShoot;
    private double adder = 200;
    NormalizedColorSensor colorSensorIntake, colorSensorShoot;
    private MotorEx launcher;
    private CRServoEx yaw1;
    private CRServoEx yaw2;
    private DcMotor intake;
    private Servo agigtator;
    private ServoEx indexer;
    private ServoEx pitch;

    double[] INTAKE_POS = {0, 130, 270};
    double[] SHOOT_POS = {170, 240, 300};

    int i = 0;
    int s = 0;

    double velocityIPS = 0;
    double velocityTPS = 0;
    private Follower follower;
    private TelemetryManager telemetryM;
    private Pose startingPose;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        yaw2 = new CRServoEx(hardwareMap, "yaw2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        colorSensorShoot = hardwareMap.get(NormalizedColorSensor.class, "sensor_shoot");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        follower = Constants.createFollower(hardwareMap);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(0.6, 0, 0);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        initAprilTag();

        startingPose = new Pose(0,0, Math.toRadians(0));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {

        follower.setStartingPose(startingPose);
        follower.update();
        follower.startTeleopDrive();

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        //for red goal
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 && detection.metadata !=null) {
                yaw1.set(.02*detection.ftcPose.bearing);
                yaw2.set(.02*detection.ftcPose.bearing);

                distanceToTarget = detection.ftcPose.range;
            } else if (currentDetections.isEmpty()) {
                yaw1.set(0);
                yaw2.set(0);
            }
        }   // end for() loop

        if (gamepad1.xWasPressed()) {
            slowMode = !slowMode;
        }

        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
            //This is how it looks with slowMode on
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                true // Robot Centric
        );

        // Convert angle to radians
        double theta = Math.toRadians(launchAngleDeg);

        // Physics denominator
        double denom =
                2 * Math.pow(Math.cos(theta), 2) *
                        (distanceToTarget * Math.tan(theta) - 39);

        // Compute required IPS safely
        if (denom > 0 && distanceToTarget > 0) {
            velocityIPS = Math.sqrt(
                    (386.4 * distanceToTarget * distanceToTarget) / denom
            );
        } else {
            velocityIPS = 0;
        }

        // IPS -> TPS (your regression, inverted)
        if (velocityIPS > 0) {
            velocityTPS = Math.log(velocityIPS / 69.9) / 0.000821;
        } else {
            velocityTPS = 0;
        }

        // Fire launcher
        if (gamepad1.left_trigger != 0 && Double.isFinite(velocityTPS)) {
            launcher.setVelocity(velocityTPS+adder);
        } else {
            launcher.setVelocity(0);
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

        if (gamepad1.bWasPressed()) {
            indexer.set(0);
        }

        pitch.set(pitch_position);

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

        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA colorsShoot = colorSensorShoot.getNormalizedColors();
        hueIntake = JavaUtil.colorToHue(colorsIntake.toColor());
        hueShoot = JavaUtil.colorToHue(colorsShoot.toColor());

        if (hueShoot >90 && hueShoot <225) {
            //SoundPlayer.getInstance().stopPlayingLoops();
            telemetry.addLine("GREEN");

            //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, greenSoundID);
        } else if (hueShoot >225 && hueShoot <350) {
            //SoundPlayer.getInstance().stopPlayingLoops();
            telemetry.addLine("PURPLE");
            //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, purpleSoundID);
        } else {
            telemetry.addLine("EMPTY");
            //SoundPlayer.getInstance().stopPlayingLoops();
        }

        if (gamepad1.x) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        telemetry.addData("Distance (in)", distanceToTarget);
        telemetry.addData("Velocity IPS", velocityIPS);
        telemetry.addData("Velocity TPS", velocityTPS);
        telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        telemetry.addData("additive", adder);
        telemetry.addData("hue", hueShoot);
        telemetry.addData("distance to tag", distanceToTarget);


        // Telemetry
        panelsTelemetry.addData("Distance (in)", distanceToTarget);
        panelsTelemetry.addData("Velocity IPS", velocityIPS);
        panelsTelemetry.addData("Velocity TPS", velocityTPS);
        panelsTelemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        panelsTelemetry.addData("Adder", adder);
        panelsTelemetry.addData("Color Sensor Reading", hueShoot);
        panelsTelemetry.update(telemetry);

        telemetry.update();

    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
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
    }

}
