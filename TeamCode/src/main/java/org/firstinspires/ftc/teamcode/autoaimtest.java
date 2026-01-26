package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Thread.sleep;

import android.app.Activity;
import android.view.View;

import com.bylazar.configurables.annotations.Configurable;
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
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "Turret Aim Test", group = "Testing")
public class autoaimtest extends OpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static double servo_speed = 0;
    public static double kp = 1;
    public static double ki = 1;
    public static double kd = 0;
    public static double kf = 0;
    private ServoEx pitch = null;
    private CRServoEx yaw1 = null;
    private CRServoEx yaw2 = null;
    private MotorEx launcher = null;
    private double pitchAngleDegrees;
    @Override
    public void init() {

        //initializes hardware
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        yaw2 = new CRServoEx(hardwareMap, "yaw2");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        launcher = new MotorEx(hardwareMap, "launcher");
        //yaw1.setPIDF(new PIDFCoefficients(kp, ki, kd, kf));
        //yaw2.setPIDF(new PIDFCoefficients(kp, ki, kd, kf));

        launcher.setInverted(true);

        initAprilTag();

        //sets run mode for motors

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        pitch.set(1462.5);


        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        //sets pidf values for the launcher
        launcher.setVeloCoefficients(.03, .8, .05);

        //adds status of initialization to telemetry
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
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

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        telemetryAprilTag();

        //for red goal
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 && detection.metadata !=null) {
                yaw1.set(.02*detection.ftcPose.bearing);
                yaw2.set(.02*detection.ftcPose.bearing);
                if (detection.ftcPose.pitch > 110 && detection.ftcPose.pitch < 140){
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
                double velocity = (Math.sqrt(numerator / denominator))*142.239908137;
                if (gamepad1.left_trigger != 0) {
                    launcher.set(velocity);
                } else {
                    launcher.set(0);
                }
                telemetry.addData("target velocity", velocity);
                telemetry.addData("current velocity", launcher.getVelocity());
                telemetry.addData("pitch angle", pitchAngleDegrees);
            } else if (currentDetections.isEmpty()) {
                yaw1.set(0);
                yaw2.set(0);
                launcher.set(0);
            }
        }   // end for() loop

        //makes gamepad vibrate when the launcher is at the minimum velocity
        if (launcher.getVelocity() >= 2300) {
            gamepad1.rumble(1000);
        }
        return 0;
    }

                // The following default settings are available to un-comment and edit as needed.

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

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


}


