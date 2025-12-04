package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
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
public class test_red extends OpMode {
    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //private AprilTagProcessor aprilTag;
    //private VisionPortal visionPortal;
    int intake1 = 3;
    int intake2 = 3;
    int intake3 = 4;
    //double hue;
    //private AprilTagDetection detection;
    View relativeLayout;
    //NormalizedColorSensor colorSensor;
    private ServoEx pitch = null;
    private CRServoEx yaw = null;
    private ServoEx agigtator = null;
    private ServoEx indexer = null;

    private DcMotorEx intake = null;
    private MotorEx launcher = null;

    private Follower follower;
    public Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private enum IndexIntake {
        INTAKE_1,
        INTAKE_2,
        INTAKE_3
    }

    private enum IndexShoot {
        SHOOT_1,
        SHOOT_2,
        SHOOT_3
    }

    private IndexIntake indexIntake = IndexIntake.INTAKE_1;
    private IndexShoot indexShoot = IndexShoot.SHOOT_1;

    double goalX = 137;
    double goalY = 142;

    @Override
    public void init() {

        //initializes hardware
        launcher = new MotorEx(hardwareMap, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //AbsoluteAnalogEncoder yawEncoder = new AbsoluteAnalogEncoder(hardwareMap, "yaw_encoder");
        //yaw = new CRServoEx(hardwareMap, "yaw", yawEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        yaw = new CRServoEx(hardwareMap, "yaw");
        agigtator = new ServoEx(hardwareMap, "agigtator");
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 300);
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);

        //TODO: tune ts
        //yaw.setPIDF(new PIDFCoefficients(0.001, 0.0, 0.1, 0.0001));
        //yaw.set(Math.toRadians(90)); // move to 90 degrees (in radians)

        //int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //initializes follower for pedropathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        //initializes telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        agigtator.setInverted(true);

        //initAprilTag();

        //sets run mode for motors


        launcher.setRunMode(MotorEx.RunMode.RawPower);


        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        //ensures that all servos start at the correct position
        agigtator.set(0);
        indexer.set(90);
        //sets pidf values for the launcher
        launcher.setVeloCoefficients(.05, .5, .05);

        //adds status of initialization to telemetry
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        //updates follower and telemetry at the start of each loop
        follower.update();
        telemetryM.update();

        //slow mode turns on when a is pressed
        //turns back off when a is released
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

        /*
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                yaw.set(Math.toRadians(detection.ftcPose.yaw));
                pitch.set(detection.ftcPose.pitch);
                launcher.set(gamepad1.left_trigger);
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop



         */


        //makes gamepad vibrate when the launcher is at the minimum velocity
        if (launcher.getVelocity() >= 2300) {
            gamepad1.rumble(1000);
        }


        //sets the position of the agigtator when the right trigger is pressed
        //basically, when right trigger is pressed, the ball shoots out
        if (gamepad1.right_trigger != 0) {
            agigtator.set(.3);
        } else {
            agigtator.set(0);
        }

        if (gamepad1.dpadRightWasPressed()) {
            switch (indexIntake) {
                case INTAKE_1:
                    indexer.set(0);
                    indexIntake = IndexIntake.INTAKE_2;
                    /*
                    if (hue > 225 && hue < 350){
                        intake2 = 0;
                    } else if (hue > 90 && hue < 150) {
                        intake2 = 1;
                        *
                    }
                    */
                    break;
                case INTAKE_2:
                    indexIntake = IndexIntake.INTAKE_3;
                    /*
                    if (hue > 225 && hue < 350){
                        intake3 = 0;
                    } else if (hue > 90 && hue < 150) {
                        intake3 = 1;
                    }
                    */
                    indexer.set(130);
                    break;
                case INTAKE_3:
                    indexer.set(270);
                    /*
                    if (hue > 225 && hue < 350){
                        intake1 = 0;
                    } else if (hue > 90 && hue < 150) {
                        intake1 = 1;
                    }

                     */
                    indexIntake = IndexIntake.INTAKE_1;
                    break;
            }
        } else if (gamepad1.dpadLeftWasPressed()) {
            switch (indexIntake) {
                case INTAKE_1:
                    indexIntake = IndexIntake.INTAKE_2;
                    /*
                    if (hue > 225 && hue < 350){
                        intake1 = 0;
                    } else if (hue > 90 && hue < 150) {
                        intake1 = 1;
                    }
                    */
                    indexer.set(270);
                    break;
                case INTAKE_2:
                    indexIntake = IndexIntake.INTAKE_3;
                    /*
                    if (hue > 225 && hue < 350){
                        intake3 = 0;
                    } else if (hue > 90 && hue < 150) {
                        intake3 = 1;
                    }
                     */
                    indexer.set(130);
                    break;
                case INTAKE_3:
                    indexer.set(0);
                    /*
                    if (hue > 225 && hue < 350){
                        intake2 = 0;
                    } else if (hue > 90 && hue < 150) {
                        intake2 = 1;
                    }

                     */
                    indexIntake = IndexIntake.INTAKE_1;
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
                    /*
                    if (intake1 == 0) {
                        relativeLayout.setBackgroundColor(Color.rgb(180, 53, 189));
                    } else if (intake1 == 1) {
                        relativeLayout.setBackgroundColor(Color.rgb(54, 201, 76));
                    }

                     */
                    indexShoot = IndexShoot.SHOOT_1;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            switch (indexShoot) {
                case SHOOT_1:
                    indexShoot = IndexShoot.SHOOT_2;
                    indexer.set(300);
                    /*
                    if (intake1 == 0) {
                        relativeLayout.setBackgroundColor(Color.rgb(180, 53, 189));
                    } else if (intake1 == 1) {
                        relativeLayout.setBackgroundColor(Color.rgb(54, 201, 76));
                    }

                     */
                    break;
                case SHOOT_2:
                    indexShoot = IndexShoot.SHOOT_3;
                    indexer.set(240);
                    break;
                case SHOOT_3:
                    indexShoot = IndexShoot.SHOOT_1;
                    indexer.set(170);
                    break;
            }
        }
    }
/*
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
 */
}


