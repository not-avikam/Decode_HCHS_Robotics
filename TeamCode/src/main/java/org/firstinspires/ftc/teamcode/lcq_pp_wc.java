package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "lcq_pp_wc", group = "lcq")
public class lcq_pp_wc extends OpMode {
    public static double kP = .2;
    public static double kI = .4;
    public static double kD = 0;
    double adder = 300;
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
        mecanum = new MecanumDrive(false, frontLeft, frontRight,
                backLeft, backRight);

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

        yaw1.setInverted(false);

        initAprilTag();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(11);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        frontLeft.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);
        frontRight.setInverted(true);

        //launcher.setVeloCoefficients(kP, kI, kD);

        intake.setInverted(true);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);

        lut = new InterpLUT();

        lut.add(-50, 0);
        lut.add(0, 0);
        lut.add(40, 0);
        lut.add(57, 220);
        lut.add(69, 220);
        lut.add(112, 250);
        lut.add(135.0, 65.0);
        lut.add(137, 775);
        lut.add(175, 120);
        lut.add(204, 70.0);

        lut.createLUT();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

        telemetry.addLine("Press right for red alliance");
        telemetry.addLine("Press left for blue alliance");
        telemetry.addData("Selected Alliance", currentAlliance == 0 ? "Red" : "Blue");


        if (gamepad1.dpadRightWasPressed()) {
            currentAlliance = 0;
            light.setPosition(.277);
        } else if (gamepad1.dpadLeftWasPressed()) {
            currentAlliance = 1;
            light.setPosition(.611);
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
                exposureControl.setExposure(10, TimeUnit.MILLISECONDS);
            }
        }
    }

    @Override
    public void loop() {

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
        
        double bearing = 0;

        if (tag24 != null && currentAlliance == 0) {
            distanceToTarget = tag24.ftcPose.range;
            
            bearing = tag24.ftcPose.bearing - 15;

            yaw1.set(bearing * .02);

            telemetry.addLine("tag detected");
        } else if (tag20 != null && currentAlliance == 1) {

            bearing = tag20.ftcPose.bearing - 15;

            yaw1.set(bearing * .02);

            distanceToTarget = tag20.ftcPose.range;

            telemetry.addLine("tag detected");
        } else {
            yaw1.set(gamepad1.right_stick_y * .5);
        }


        theta = lut.get(distanceToTarget);
        pitch.set(theta);

        mecanum.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true);

        if (gamepad1.dpadDownWasPressed()) {
            imu.resetYaw();
        }

        if (gamepad1.dpadRightWasPressed()) {
            adder += 50;
        } else if (gamepad1.dpadLeftWasPressed()) {
            adder -= 50;
        }

        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV);

        double targetspeed;

        if (distanceToTarget >= 120) {
            targetspeed = 2000;
        } else {
            targetspeed = 1500;
        }

        if (gamepad1.left_trigger != 0) {
            launcher.setVelocity(targetspeed + adder);
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
        } else if (gamepad1.right_trigger != 0) {
            trigger.set(0);
            intake.set(1);
        } else {
            intake.set(0);
            trigger.set(.2);
        }

        if (gamepad1.leftBumperWasPressed()) {
            agitator.set(1);
        } else {
            agitator.set(0);
        }

            telemetry.addData("distance to target (in)", distanceToTarget);
            telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
            telemetry.addData("LUT angle", theta);
            telemetry.addData("adder", adder);
            telemetry.addData("target speed", targetspeed);
            telemetry.setAutoClear(true);

            // Telemetry
            panelsTelemetry.addData("distance to target (in)", distanceToTarget);
            panelsTelemetry.addData("kS", kS);
            panelsTelemetry.addData("kV", kV);
            telemetry.update();

        }

        // Helper to check which side of a line a point is on
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
    }
