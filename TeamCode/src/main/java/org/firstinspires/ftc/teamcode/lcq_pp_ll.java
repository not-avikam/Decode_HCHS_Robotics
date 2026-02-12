package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.PoseStorage.currentPose;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.video.KalmanFilter;

import java.util.List;

@Configurable
@TeleOp(name = "lcq_pp_ll", group = "lcq")
public class lcq_pp_ll extends OpMode {
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
    private MotorEx yaw1 = null;
    double theta;
    public static double kS = 0;
    public static double kV = 0;
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    InterpLUT lut;
    Follower follower;
    public static Pose startingPose = new Pose(0, 0, Math.toRadians(0));
    Pose scorePose = new Pose(132.12651646447142, 136.18370883882147);
    double distanceToTarget;
    private Motor turretEncoder;
    private final int ticks_per_rev = 10000;

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
        yaw1 = new MotorEx(hardwareMap, "yaw1");
        trigger = new ServoEx(hardwareMap, "trigger");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretEncoder = new Motor(hardwareMap, "turretEncoder");

        turretEncoder.stopAndResetEncoder();

        limelight.pipelineSwitch(0);

        yaw1.encoder = turretEncoder.encoder;
        yaw1.setRunMode(Motor.RunMode.PositionControl);
        yaw1.setPositionCoefficient(.03);
        turretEncoder.encoder.setDistancePerPulse(360/ticks_per_rev);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(11);

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
        lut.add(204, 70.0);

        lut.createLUT();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        limelight.start();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

        telemetry.addLine("Press right for red alliance");
        telemetry.addLine("Press left for blue alliance");
        telemetry.addData("Selected Alliance", currentAlliance == 0 ? "Red" : "Blue");

        if (currentPose != null) {
            telemetry.addData("Current Pose", currentPose);
        } else if (startingPose == null) {
            telemetry.addLine("pose not passed from auto");
        }


        if (gamepad1.dpadRightWasPressed()) {
            scorePose = new Pose(132.12651646447142, 136.18370883882147);
            currentAlliance = 0;
            limelight.pipelineSwitch(0);
            light.setPosition(.277);
        } else if (gamepad1.dpadLeftWasPressed()) {
            scorePose = new Pose(11.585788561525153, 136.4332755632582);
            currentAlliance = 1;
            limelight.pipelineSwitch(1);
            light.setPosition(.611);
        }

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            telemetry.addData("Botpose", botpose.toString());

            // Access barcode results
            List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
            for (LLResultTypes.BarcodeResult br : barcodeResults) {
                telemetry.addData("Barcode", "Data: %s", br.getData());
            }

            startingPose = new Pose(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setStartingPose(startingPose);
            telemetry.addData("Starting Pose Read From Limelight", startingPose);

        } else {
            telemetry.addData("Limelight", "No data available");
        }

        follower.update();

    }

    @Override
    public void start() {follower.startTeleopDrive();}

    @Override
    public void loop() {
        double deltaX = scorePose.getX() - follower.getPose().getX();
        double deltaY = scorePose.getY() - follower.getPose().getY();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Only update pose if the robot's velocity is low
            boolean isRobotStopped;

            if (follower.getVelocity().getMagnitude() >= .2) {
                isRobotStopped = false;
            } else {
                isRobotStopped = true;
            }

            if (isRobotStopped) {
                Pose3D botpose3d = result.getBotpose();
                // Convert limelight pose to Pedro Pathing coordinate system
                Pose limelightPose = new Pose(botpose3d.getPosition().x, botpose3d.getPosition().y, orientation.getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                Pose currentPose = follower.getPose();

                // Ignore vision data if it's too far from the current pose (noise)
                if (Math.hypot(limelightPose.getX() - currentPose.getX(), limelightPose.getY() - currentPose.getY()) < 6.0) {
                    double alpha = 0.1; // Trust factor

                    // Weighted average for position
                    double newX = alpha * limelightPose.getX() + (1.0 - alpha) * currentPose.getX();
                    double newY = alpha * limelightPose.getY() + (1.0 - alpha) * currentPose.getY();

                    // Interpolate heading, handling wraparound
                    double headingError = AngleUnit.normalizeRadians(limelightPose.getHeading() - currentPose.getHeading());
                    double newHeading = currentPose.getHeading() + alpha * headingError;

                    follower.setPose(new Pose(newX, newY, newHeading));
                    follower.update();
                }
            }
        }

        distanceToTarget = Math.hypot(deltaX, deltaY);
        double turretTarget = AngleUnit.normalizeDegrees(
                Math.toDegrees(Math.atan2(deltaY, deltaX)) - Math.toDegrees(follower.getHeading()));
        if (result != null && result.isValid()) {
            double visionCorrectedTarget = AngleUnit.normalizeDegrees(turretTarget - result.getTx());
            yaw1.setTargetDistance(visionCorrectedTarget);
        } else {
            yaw1.setTargetDistance(turretTarget);
        }

        yaw1.set(.5);

        theta = lut.get(distanceToTarget);
        pitch.set(theta);

        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );

        if (gamepad1.dpadDownWasPressed()) {
            imu.resetYaw();
        }

        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV);

        if (gamepad1.left_trigger != 0) {
            //launcher.setVelocity(feedforward.calculate(1500));
            launcher.setVelocity(1500);
            if (launcher.getVelocity() <= 1520 && launcher.getVelocity() >= 1480) {
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

        if (gamepad1.dpadDownWasPressed()) {
            imu.resetYaw();
        }

        if (gamepad1.right_trigger != 0) {
            trigger.set(1);
            intake.set(1);
        } else if (isRobotInTriangle(follower.getPose().getX(), follower.getPose().getY(), 48, 0, 72, 24, 96, 0) && (launcher.getVelocity() <= 1520) && (launcher.getVelocity() >= 1480)) {
            trigger.set(1);
        } else if (isRobotInTriangle(follower.getPose().getX(), follower.getPose().getY(), 0, 144, 144, 144, 72, 72) && (launcher.getVelocity() <= 1520) && (launcher.getVelocity() >= 1480)){
            trigger.set(1);
        } else {
            intake.set(0);
            trigger.set(0);
        }

        telemetry.addData("distance to target (in)", distanceToTarget);
        telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        telemetry.addData("LUT angle", theta);

        telemetry.setAutoClear(true);

        // Telemetry
        panelsTelemetry.addData("distance to target (in)", distanceToTarget);
        panelsTelemetry.addData("kS", kS);
        panelsTelemetry.addData("kV", kV);

        telemetry.update();

    }

    // Helper to check which side of a line a point is on
    private double crossProduct(double x1, double y1, double x2, double y2, double x3, double y3) {
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    }

    public boolean isRobotInTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
        double d1, d2, d3;
        boolean has_neg, has_pos;

        d1 = crossProduct(px, py, x1, y1, x2, y2);
        d2 = crossProduct(px, py, x2, y2, x3, y3);
        d3 = crossProduct(px, py, x3, y3, x1, y1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }
}
