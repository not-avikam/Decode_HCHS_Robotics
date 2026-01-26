package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.app.Activity;
import android.view.View;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "RED Decode LM2", group = "HSI LM2")
public class LM2_HSI_RED extends OpMode {
    Path pathToLine;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    double hueIntake, hueShoot;
    View relativeLayout;
    NormalizedColorSensor colorSensorIntake, colorSensorShoot;
    private ServoEx pitch, agigtator, indexer = null;
    private CRServoEx yaw1, yaw2 = null;
    private DcMotorEx intake = null;
    private MotorEx launcher = null;

    private Follower follower;
    private double velocity;
    public Pose startingPose = new Pose(0,0, Math.toRadians(0)) ; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private int shootBall;
    private Timer actionTimer, shootTimer;
    private boolean greenFound, purpleFound, gongFound;      // Sound file present flags
    private enum IndexIntake {
        INTAKE_1,
        INTAKE_2,
        INTAKE_3,
        INTAKE_4,
        INTAKE_5,
        INTAKE_6
    }

    private enum IndexShoot {
        SHOOT_1,
        SHOOT_2,
        SHOOT_3,
        SHOOT_4,
        SHOOT_5,
        SHOOT_6
    }

    private IndexIntake indexIntake = IndexIntake.INTAKE_1;
    private IndexShoot indexShoot = IndexShoot.SHOOT_1;
    double[] INTAKE_POS = {0, 130, 270};
    double[] SHOOT_POS = {170, 240, 300};
    int i = 0;
    int s = 0;
    double goalX = 137;
    double goalY = 142;

    double pitchAngleDegrees;

    @Override
    public void init() {

        //initializes hardware
        launcher = new MotorEx(hardwareMap, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        yaw2 = new CRServoEx(hardwareMap, "yaw2");
        agigtator = new ServoEx(hardwareMap, "agigtator");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        colorSensorShoot = hardwareMap.get(NormalizedColorSensor.class, "sensor_shoot");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);

        yaw2.setInverted(true);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //initializes follower for pedropathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        follower.update();

        //initializes telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        agigtator.setInverted(true);

        initAprilTag();

        actionTimer = new Timer();
        shootTimer = new Timer();

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);


        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        //ensures that all servos start at the correct position
        agigtator.set(0);
        indexer.set(0);
        pitch.set(1462.5);


        //sets pidf values for the launcher
        launcher.setVeloCoefficients(.03, .8, .05);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {

        //updates follower and telemetry at the start of each loop
        follower.update();
        telemetryM.update();

        //slow mode turns on when a is pressed
        //turns back off when a is released;
        //works by multiplying the speed of the robot by .5
        if (gamepad1.aWasPressed()) {
            slowMode = true;
            slowModeMultiplier = 0.5;
        } else if (gamepad1.aWasReleased()) {
            slowMode = false;
            slowModeMultiplier = 1;
        }

        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Robot Centric
        );
            //This is how it looks with slowMode on
        else if (slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                false // Robot Centric
        );

        //adds position and velocity to telemetry
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA colorsShoot = colorSensorShoot.getNormalizedColors();
        hueIntake = JavaUtil.colorToHue(colorsIntake.toColor());
        hueShoot = JavaUtil.colorToHue(colorsShoot.toColor());


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 && detection.metadata !=null) {

                yaw1.set(.02*detection.ftcPose.bearing);
                yaw2.set(.02*detection.ftcPose.bearing);

                if (detection.ftcPose.pitch > 110 && detection.ftcPose.pitch < 140) {
                    pitchAngleDegrees = detection.ftcPose.pitch*11.25;
                } else if (detection.ftcPose.pitch > 140){
                    pitchAngleDegrees = 140;
                } else if (detection.ftcPose.pitch < 110) {
                    pitchAngleDegrees = 110;
                } else {
                    pitchAngleDegrees = 125;
                }

                pitch.set(pitchAngleDegrees);

                double theta = Math.toRadians(pitchAngleDegrees);
                double R = detection.ftcPose.range;
                double h = detection.ftcPose.elevation;
                double g = 9.8;
                double numerator = g * R * R;
                double denominator = 2 * Math.pow(Math.cos(theta), 2) * (R * Math.tan(theta) - h);
                velocity = (Math.sqrt(numerator / denominator))*142.239908137;

                if (gamepad1.left_trigger != 0) {
                    launcher.set(velocity);
                } else {
                    launcher.set(0);
                }

                telemetry.addData("target velocity", velocity);
                telemetry.addData("current velocity", launcher.getVelocity());
                telemetry.addData("pitch angle", pitchAngleDegrees);

            } else {
                yaw1.set(0);
                yaw2.set(0);
            }
        }   // end for() loop

        if (gamepad1.x) {
            intake.setPower(1);
        }

        //sets the position of the agigtator when the right trigger is pressed
        //basically, when right trigger is pressed, the ball shoots out
        if (gamepad1.right_trigger != 0) {
            agigtator.set(.3);
        } else {
            agigtator.set(0);
        }

        if (gamepad1.dpadRightWasPressed()) {
            if (i < INTAKE_POS.length - 1) {
                i++;
            }
            indexer.set(INTAKE_POS[i]);
        }
        else if (gamepad1.dpadLeftWasPressed()) {
            if (i > 0) {
                i--;
            }
            indexer.set(INTAKE_POS[i]);
        }


        if (gamepad1.dpadUpWasPressed()) {
            if (s < SHOOT_POS.length - 1) {
                s++;
            }
            indexer.set(SHOOT_POS[s]);
        }
        else if (gamepad1.dpadDownWasPressed()) {
            if (s > 0) {
                s--;
            }
            indexer.set(SHOOT_POS[s]);
        }


            /*
        if (gamepad1.dpadRightWasPressed()) {
            switch (indexIntake) {
                case INTAKE_1:
                    indexer.set(0);
                    indexIntake = IndexIntake.INTAKE_2;
                    break;
                case INTAKE_2:
                    indexIntake = IndexIntake.INTAKE_3;
                    indexer.set(130);
                    break;
                case INTAKE_3:
                    indexer.set(270);
                    indexIntake = IndexIntake.INTAKE_1;
                    break;
            }
        } else if (gamepad1.dpadLeftWasPressed()) {
            switch (indexIntake) {
                case INTAKE_4:
                    indexIntake = IndexIntake.INTAKE_5;
                    indexer.set(270);
                    break;
                case INTAKE_5:
                    indexIntake = IndexIntake.INTAKE_6;
                    indexer.set(130);
                    break;
                case INTAKE_6:
                    indexer.set(0);
                    indexIntake = IndexIntake.INTAKE_4;
                    break;
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            switch (indexShoot) {
                case SHOOT_1:
                    indexShoot = IndexShoot.SHOOT_2;
                    indexer.set(170);
                    break;
                case SHOOT_2:
                    indexShoot = IndexShoot.SHOOT_3;
                    indexer.set(240);
                    break;
                case SHOOT_3:
                    indexer.set(300);
                    indexShoot = IndexShoot.SHOOT_1;
                    break;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            switch (indexShoot) {
                case SHOOT_4:
                    indexShoot = IndexShoot.SHOOT_5;
                    indexer.set(300);
                    break;
                case SHOOT_5:
                    indexShoot = IndexShoot.SHOOT_6;
                    indexer.set(240);
                    break;
                case SHOOT_6:
                    indexShoot = IndexShoot.SHOOT_4;
                    indexer.set(170);
                    break;
            }

             */


        //return 0;
    }


    private void initAprilTag() {

        // Create the AprilTag processor.
        // The following default settings are available to un-comment and edit as needed.
        //.setDrawAxes(false)
        //.setDrawCubeProjection(false)
        //.setDrawTagOutline(true)
        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

}


