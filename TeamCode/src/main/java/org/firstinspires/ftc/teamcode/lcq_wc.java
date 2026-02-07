package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "lcq_wc", group = "lcq")
public class lcq_wc extends OpMode {
    private MotorEx launcher;
    private Motor intake;
    private ServoImplEx light;
    private ServoEx pitch;
    private ServoEx trigger;
    private CRServoEx linearServo1;
    private CRServoEx linearServo2;
    private Limelight3A limelight;
    int currentAlliance = 0;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    IMU imu;
    private CRServoEx yaw1 = null;
    double theta;
    public static double kS = 0;
    public static double kV = 0;
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    InterpLUT lut;
    MecanumDrive mecanum;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean visionAvailable = false;
    boolean visionActive = false;
    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        linearServo1 = new CRServoEx(hardwareMap, "linearServo1");
        linearServo2 = new CRServoEx(hardwareMap, "linearServo2");
        light = hardwareMap.get(ServoImplEx.class, "light");
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        trigger = new ServoEx(hardwareMap, "trigger");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

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
        lut.add(160.0, 70.0);

        lut.createLUT();

        mecanum = new MecanumDrive(false, frontLeft, frontRight,
                backLeft, backRight);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

        telemetry.addLine("Press right for red alliance");
        telemetry.addLine("Press left for blue alliance");
        telemetry.addData("Selected Alliance", currentAlliance == 0 ? "Red" : "Blue");

        if (gamepad1.dpadRightWasPressed()) {
            currentAlliance = 0;
            limelight.pipelineSwitch(0);
            light.setPosition(0);
        } else if (gamepad1.dpadLeftWasPressed()) {
            currentAlliance = 1;
            limelight.pipelineSwitch(1);
            light.setPosition(1);
        }

    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double distanceToTarget;

        mecanum.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, orientation.getYaw());

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

        if (tag24 != null && currentAlliance == 0 && visionActive) {
            yaw1.set(tag24.ftcPose.bearing*.01);

            distanceToTarget = tag24.ftcPose.x;

            if (gamepad1.left_trigger == 0) {
                pitch.set(tag24.ftcPose.elevation*5.9);
            } else {
                pitch.set(lut.get(distanceToTarget));
            }

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

            telemetry.addLine("tag detected");
            telemetry.addData("distance to target (in)", distanceToTarget);
        } else if (tag24 == null && tag20 == null) {
            yaw1.set(gamepad1.right_stick_y);
        }

        if (gamepad1.dpadDownWasPressed()) {
            imu.resetYaw();
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

        if (gamepad1.left_stick_button) {
            linearServo1.set(1);
            linearServo2.set(1);
        } else {
            linearServo1.set(-.3);
            linearServo2.set(-.3);
        }

        if (gamepad1.right_trigger != 0) {
            trigger.set(1);
            intake.set(1);
        } else if (gamepad1.rightBumperWasPressed()){
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
        }  else {
            trigger.set(1);
            intake.set(1);
        }
    }
}