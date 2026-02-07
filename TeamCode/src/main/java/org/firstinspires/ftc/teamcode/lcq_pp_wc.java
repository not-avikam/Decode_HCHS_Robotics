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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "lcq_pp_wc", group = "lcq")
public class lcq_pp_wc extends OpMode {
    private MotorEx launcher;
    private Motor turretEncoder;
    private Motor intake;
    private ServoImplEx light;
    private ServoEx pitch;
    private ServoEx trigger;
    private CRServoEx linearServo1;
    private CRServoEx linearServo2;
    int currentAlliance = 0;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    IMU imu;
    private CRServoEx yaw1 = null;
    double theta;
    public static double kS = 0;
    public static double kV = 0;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean visionAvailable = false;
    boolean visionActive = false;
    InterpLUT lut;
    MecanumDrive mecanum;
    Follower follower;
    private final int ticks_per_degree = 12;
    public static Pose startingPose = new Pose(0, 0, Math.toRadians(0));
    Pose scorePose = new Pose(132.12651646447142, 136.18370883882147);

    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        linearServo1 = new CRServoEx(hardwareMap, "linearServo1");
        linearServo2 = new CRServoEx(hardwareMap, "linearServo2");
        light = hardwareMap.get(ServoImplEx.class, "light");
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        turretEncoder = new Motor(hardwareMap, "turretEncoder");
        trigger = new ServoEx(hardwareMap, "trigger");

        turretEncoder.stopAndResetEncoder();

        initAprilTag();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(11);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        launcher.setVeloCoefficients(0, .4, 0);

        intake.setInverted(true);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);

        lut = new InterpLUT();

        lut.add(0.0, 30.0);
        lut.add(60.0, 30.0);
        lut.add(85.0, 40.0);
        lut.add(110.0, 55.0);
        lut.add(135.0, 65.0);
        lut.add(160.0, 70.0);

        lut.createLUT();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

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
            light.setPosition(.277);
        } else if (gamepad1.dpadLeftWasPressed()) {
            scorePose = new Pose(11.585788561525153, 136.4332755632582);
            currentAlliance = 1;
            light.setPosition(.611);
        }

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

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (visionActive) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        // Only use tags that don't have Obelisk in them
                        if (!detection.metadata.name.contains("Obelisk")) {
                            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                                    detection.robotPose.getPosition().x,
                                    detection.robotPose.getPosition().y,
                                    detection.robotPose.getPosition().z));
                            startingPose = new Pose(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                            follower.setStartingPose(startingPose);
                            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                        }
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }
            }
        }

        telemetry.update();
        telemetry.addData("start pose", startingPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

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

        double distanceToTarget;

        if (tag24 != null && currentAlliance == 0 && visionActive) {
            yaw1.set(tag24.ftcPose.bearing*.01);

            distanceToTarget = tag24.ftcPose.x;

            if (gamepad1.left_trigger == 0) {
                pitch.set(tag24.ftcPose.elevation*5.9);
            } else {
                pitch.set(lut.get(distanceToTarget));
            }

            Pose currentPose = new Pose(tag24.robotPose.getPosition().x, tag24.robotPose.getPosition().y, orientation.getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(currentPose);

            telemetry.addLine("tag detected");
            telemetry.addData("distance to target (in)", distanceToTarget);
        } else if (tag20 != null && currentAlliance == 1 && visionActive) {
            yaw1.set(tag20.ftcPose.bearing*.01);

            distanceToTarget = tag20.ftcPose.range;

            if (gamepad1.left_trigger == 0) {
                pitch.set(tag20.ftcPose.elevation*5.9);
            } else {
                pitch.set(lut.get(distanceToTarget));
            }

            Pose currentPose = new Pose(tag20.robotPose.getPosition().x, tag20.robotPose.getPosition().y, orientation.getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(currentPose);

            telemetry.addLine("tag detected");
            telemetry.addData("distance to target (in)", distanceToTarget);
        } else if (tag24 == null && tag20 == null) {
            double currentangle = turretEncoder.getCurrentPosition() / ticks_per_degree;
            double error = currentangle - Math.toDegrees(follower.getHeading());
            double deltaX = follower.getPose().getX() - scorePose.getX();
            double deltaY = follower.getPose().getY() - scorePose.getY();
            double targetAngle = Math.tan(deltaY / deltaX);

            if (currentangle == 350 || currentangle == 0) {
                yaw1.set(0);
            } else {
                yaw1.set((error - targetAngle) * .01);
            }

            distanceToTarget = Math.hypot(deltaX, deltaY);

            pitch.set(lut.get(distanceToTarget));

            telemetry.addData("distance to target (in)", distanceToTarget);
        }



        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV);

        if (gamepad1.left_trigger != 0) {
            launcher.setVelocity(feedforward.calculate(1500));
            if (launcher.getVelocity() <= 1520 || launcher.getVelocity() >= 1480) {
                light.setPosition(.46);
            } else {
                light.setPosition(.666);
            }
        } else {
            launcher.set(0);
            light.setPosition(0);
        }

        if (gamepad1.x) {
            intake.set(1);
        } else if (gamepad1.a) {
            intake.set(-1);
        } else {
            intake.set(0);
        }

        if (gamepad1.start) {
            linearServo1.set(1);
            linearServo2.set(1);
        } else {
            linearServo1.set(-.3);
            linearServo2.set(-.3);
        }

        if (gamepad1.dpadDownWasPressed()) {
            imu.resetYaw();
        }

        if (gamepad1.right_trigger != 0) {
            trigger.set(1);
            intake.set(1);
        } else if (gamepad1.rightBumperWasPressed()) {
            semiAuto();
        } else {
            intake.set(0);
            trigger.set(0);
        }

        telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        telemetry.addData("LUT angle", theta);

        telemetry.setAutoClear(true);

        // Telemetry
        panelsTelemetry.addData("kS", kS);
        panelsTelemetry.addData("kV", kV);

        telemetry.update();

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

    private void semiAuto() {
        if (launcher.getVelocity() != 1500) {
            trigger.set(0);
            intake.set(0);
        } else {
            trigger.set(1);
            intake.set(1);
        }
    }
}