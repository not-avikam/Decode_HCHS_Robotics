package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
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

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "TAP THIS ONE lm2 teleop drive", group = "lm2 2025")
public class launcher_velocity_calculator extends OpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
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
    int currentAlliance = 0;

    double velocityIPS = 0;
    double velocityTPS = 0;
    private Follower follower;
    private TelemetryManager telemetryM;
    private Pose startingPose;
    SimpleMotorFeedforward launcherfeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private boolean slowMode = false;
    //private boolean greenFound, purpleFound, gongFound;
    //int purpleSoundID;
    //int greenSoundID;
    //int gongID;
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

        yaw2.setInverted(true);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(0.6, 0, 0);

        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        initAprilTag();

        startingPose = new Pose(0,0, Math.toRadians(0));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pitch.set(.14);

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

        if (gamepad1.dpadRightWasPressed()) {
            currentAlliance = 0;
        } else if (gamepad1.dpadLeftWasPressed()) {
            currentAlliance = 1;
        }

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
            double bearing = tag24.ftcPose.bearing;

            yaw1.set(0.02 * bearing);
            yaw2.set(0.02 * bearing);

            distanceToTarget = tag24.ftcPose.range;
        } else if (tag20 != null && currentAlliance == 1) {
            double bearing = tag20.ftcPose.bearing;

            yaw1.set(0.02 * bearing);
            yaw2.set(0.02 * bearing);

            distanceToTarget = tag20.ftcPose.range;
        } else {
            yaw1.set(0);
            yaw2.set(0);
        }


        if (gamepad1.bWasPressed()) {
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
            launcher.setVelocity(launcherfeedforward.calculate(velocityTPS));
        } else {
            launcher.setVelocity(0);
        }

        /*
        if (launcher.getVelocity() >= (velocityTPS -100) && launcher.getVelocity() <= (velocityTPS +100)) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, gongID);
        } else {
            SoundPlayer.getInstance().stopPlayingLoops();
        }

         */

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

        /*
        if (hueShoot >90 && hueShoot <225) {
            SoundPlayer.getInstance().stopPlayingLoops();
            telemetry.addLine("GREEN");
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, greenSoundID);
        } else if (hueShoot >225 && hueShoot <350) {
            SoundPlayer.getInstance().stopPlayingLoops();
            telemetry.addLine("PURPLE");
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, purpleSoundID);
        } else {
            telemetry.addLine("EMPTY");
            SoundPlayer.getInstance().stopPlayingLoops();
        }

         */

        if (gamepad1.x) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        pitch.set(.14);

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

        //return theta;
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
