package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static java.lang.Math.cos;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "new bot axon", group = "lm2 2025")
public class NEW_BOT_axon extends OpMode {
    // ---- Turret Constants ----
    public static double TURRET_KP = 0.01;
    public static double TURRET_KD = 0.002;
    public static double TURRET_DEADBAND_DEG = 1.0;
    public static double TICKS_PER_REV = 4314; // turret output ticks

    // ---- Turret State ----
    private double lastTurretError = 0;
    private double turretZeroOffsetDeg = 0;
    private boolean turretZeroed = false;

    double hueIntake;
    private double adder = 200;
    private int indexerColor1 = 3;
    private int indexerColor2 = 3;
    private int indexerColor3 = 3;
    private DistanceSensor distanceSensor;
    NormalizedColorSensor colorSensorIntake;
    private MotorEx launcher;
    private Motor intake;
    private Servo agigtator;
    private ServoImplEx light;
    private ServoEx indexer;
    private ServoEx pitch;
    private CRServoEx linearServo1;
    private CRServoEx linearServo2;
    double distance;
    double[] INTAKE_POS = {0, 130, 270};
    double[] SHOOT_POS = {162, 228, 295.5};
    int i = 0;
    int s = 0;
    int currentAlliance = 0;
    double velocityIPS = 0;
    double velocityTPS = 0;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    IMU imu;
    private Motor turretEncoder;
    private CRServoEx yaw1 = null;
    public static double height = 35;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    GamepadEx driverOp;
    boolean visionAvailable = false;
    boolean visionActive = false;
    double theta = 30*5.9;
    private Follower follower;
    private Pose scorePose = new Pose(132.12651646447142, 136.18370883882147);
    private Pose startingPose = new Pose(87.562, 9.127, Math.toRadians(90));
    public static double kS = 0;
    public static double kV = 0;
    SquIDFController squidfController;
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    InterpLUT lut;
    double getTurretAngleDegRaw() {
        return (turretEncoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public static double kF = .2;

    private boolean slowMode = false;
    MecanumDrive mecanum;
    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        linearServo1 = new CRServoEx(hardwareMap, "linearServo1");
        linearServo2 = new CRServoEx(hardwareMap, "linearServo2");
        light = hardwareMap.get(ServoImplEx.class, "light");
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        turretEncoder = new Motor(hardwareMap, "turretEncoder");
        driverOp = new GamepadEx(gamepad1);

        frontLeft.setInverted(false);
        backLeft.setInverted(false);
        backRight.setInverted(true);
        frontRight.setInverted(true);



        squidfController = new SquIDFController(
                .6,  // kP
                0,  // kI
                0, // kD
                kF   // kF
        );

        turretEncoder.setRunMode(Motor.RunMode.RawPower);
        turretEncoder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretEncoder.setInverted(false); // change to true if angle moves backwards
        turretEncoder.stopAndResetEncoder();


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // set the run mode

        intake.setInverted(true);
        //agigtator.setDirection(Servo.Direction.REVERSE);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        lut = new InterpLUT();

        lut.add(0.0,   30.0);
        lut.add(60.0,  30.0);
        lut.add(85.0,  40.0);
        lut.add(110.0, 55.0);
        lut.add(135.0, 65.0);
        lut.add(160.0, 70.0);


        mecanum = new MecanumDrive(false, frontLeft, frontRight,
                backLeft, backRight);
        lut.createLUT();

        indexer.set(0);
        pitch.set(theta);

        follower = Constants.createFollower(hardwareMap);

        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
            telemetry.addLine("Localization: PASSED FROM AUTON");
        } else {
            follower.setStartingPose(startingPose);
            telemetry.addLine("Localization: DEFAULT USED");
        }
        follower.update();

        initAprilTag();

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
            light.setPosition(0);
        } else if (gamepad1.dpadLeftWasPressed()) {
            scorePose = new Pose(11.585788561525153, 136.4332755632582);
            currentAlliance = 1;
            light.setPosition(1);
        }

        if (visionActive) {

        }

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        mecanum.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false);

        distance = distanceSensor.getDistance(DistanceUnit.MM);
        hueIntake = JavaUtil.colorToHue(colorSensorIntake.getNormalizedColors().toColor());

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

        double deltaX = follower.getPose().getX() - scorePose.getX();
        double deltaY = follower.getPose().getY() - scorePose.getY();
        double distanceToTarget = Math.hypot(deltaX, deltaY);

        theta = lut.get(distanceToTarget);
        double clampedTheta = clamp(theta,
                30,
                70);
        pitch.set(clampedTheta*5.9);


        double denom =
                2 * Math.pow(cos(theta), 2) *
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

        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV);

        if (gamepad1.left_trigger !=0 && velocityTPS > 0) {
            launcher.setVelocity(squidfController.calculate(velocityTPS*gamepad1.left_trigger));
        } else if (gamepad1.left_trigger != 0 && velocityTPS == 0 || gamepad1.right_stick_button) {
            launcher.set(gamepad1.left_trigger*2000);
            light.setPosition(0.275);
        } else if (gamepad1.left_bumper) {
            launcher.set(-1000);
        } else {
            launcher.setVelocity(0);
        }

        // ---------------- TURRET AIMING ----------------

// 1) Auto-zero turret once (robot must start with turret facing forward)
        if (!turretZeroed) {
            turretZeroOffsetDeg = getTurretAngleDegRaw();
            turretZeroed = true;
        }

// 2) Vector from robot → goal (FIELD frame)
        double dx = scorePose.getX() - follower.getPose().getX();
        double dy = scorePose.getY() - follower.getPose().getY();

// 3) Field-centric angle to goal
        double targetFieldAngleDeg =
                Math.toDegrees(Math.atan2(dy, dx));

// 4) Robot heading (field-centric)
        double robotHeadingDeg =
                Math.toDegrees(follower.getHeading()) * -1;

// 5) Desired turret angle RELATIVE TO ROBOT
        double targetTurretAngleDeg =
                targetFieldAngleDeg - robotHeadingDeg;

        targetTurretAngleDeg = normalizeAngle(targetTurretAngleDeg);

// 6) Current turret angle RELATIVE TO ROBOT
        double currentTurretAngleDeg =
                getTurretAngleDegRaw() - turretZeroOffsetDeg;

        currentTurretAngleDeg = normalizeAngle(currentTurretAngleDeg);

// 7) Error (what the turret must correct)
        double errorAngle =
                targetTurretAngleDeg - currentTurretAngleDeg;

        errorAngle = normalizeAngle(errorAngle);

// 8) PD controller
        double errorDerivative = clamp(
                errorAngle - lastTurretError,
                -10,
                10
        );

        double turretPower =
                errorAngle * TURRET_KP +
                        errorDerivative * TURRET_KD;

// 9) Deadband
        if (Math.abs(errorAngle) < TURRET_DEADBAND_DEG) {
            turretPower = 0;
        }

// 10) Clamp + apply
        turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
        yaw1.set(-turretPower);

// 11) Save state
        lastTurretError = errorAngle;

// ---------------- END TURRET ----------------



        // Agitator
        if (gamepad1.right_trigger != 0) {
            agigtator.setPosition(0);
        } else {
            agigtator.setPosition(0.3);
        }

        if (gamepad1.aWasPressed()) {
            if (indexerColor1 == 1) indexer.set(295.5);      // Slot 1 is Green
            else if (indexerColor2 == 1) indexer.set(228);  // Slot 2 is Green
            else if (indexerColor3 == 1) indexer.set(162);  // Slot 3 is Green
        } else if (gamepad1.yWasPressed()) {
            if (indexerColor1 == 2) indexer.set(295.5);      // Slot 1 is Purple
            else if (indexerColor2 == 2) indexer.set(228);  // Slot 2 is Purple
            else if (indexerColor3 == 2) indexer.set(162);  // Slot 3 is Purple
        }

        if (gamepad1.bWasPressed() && gamepad1.xWasPressed()) {
            follower.setPose(startingPose);
            follower.setStartingPose(startingPose);
            follower.update();
        }

        if (i == 0 && hueIntake <= 225 && hueIntake >= 90) {
            //green
            indexerColor1 = 1;
        } else if (i == 0 && hueIntake <= 350 && hueIntake >= 225) {
            //purple
            indexerColor1 = 2;
        } else if (i == 1 && hueIntake <= 225 && hueIntake >= 90) {
            //green
            indexerColor2 = 1;
        } else if (i == 1 && hueIntake <= 350 && hueIntake >= 225) {
            //purple
            indexerColor2 = 2;
        } else if (i == 2 && hueIntake <= 225 && hueIntake >= 90) {
            //green
            indexerColor3 = 1;
        } else if (i == 2 && hueIntake <= 350 && hueIntake >= 225) {
            //purple
            indexerColor3 = 2;
        } else if (i == 0 && distance > 30) {
            indexerColor1 = 3;
        } else if (i == 1 && distance > 30) {
            indexerColor2 = 3;
        } else if (i == 2) {
            indexerColor3 = 3;
        }



        if (s == 2 && indexerColor1 == 1 || s == 1 && indexerColor2 == 1 || s == 0 && indexerColor3 == 1) {
            light.setPwmEnable();
            light.setPosition(0.455);
            telemetry.speak("Purple");
        } else if (s == 2 && indexerColor1 == 2 || s == 1 && indexerColor2 == 2 || s == 0 && indexerColor3 == 2) {
            light.setPwmEnable();
            light.setPosition(0.725);
            telemetry.speak("Green");
        }   else if (s == 2 && indexerColor1 == 3 || s == 1 && indexerColor2 == 3 || s == 0 && indexerColor3 == 3) {
            light.setPwmDisable();
        } else if (launcher.getVelocity() <= velocityTPS+100 && launcher.getVelocity() >= velocityTPS-100) {
            light.setPwmEnable();
            light.setPosition(0.633);
            telemetry.speak("Target velocity achieved");
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
        } else if (gamepad1.back) {
            intake.set(-1);
        } else {
            intake.set(0);
        }

        if (gamepad1.rightBumperWasPressed()) {
            follower.holdPoint(follower.getPose());
        } else if (gamepad1.rightBumperWasReleased()) {
            follower.breakFollowing();
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            linearServo1.set(1);
            linearServo2.set(1);
        } else if (gamepad1.leftStickButtonWasReleased()){
            linearServo1.set(-.3);
            linearServo2.set(-.3);
        }

        telemetry.addData("distance to target (in)", distanceToTarget);
        telemetry.addData("target velocity IPS", velocityIPS);
        telemetry.addData("target velocity TPS", velocityTPS);
        telemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        telemetry.addData("additive", adder);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("hue", hueIntake);
        telemetry.addData("indexer color 1", indexerColor1);
        telemetry.addData("indexer color 2", indexerColor2);
        telemetry.addData("indexer color 3", indexerColor3);
        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);
        telemetry.addData("Target Turret Deg", targetTurretAngleDeg);
        telemetry.addData("Current Turret Deg", currentTurretAngleDeg);
        telemetry.addData("Turret Error Deg", errorAngle);
        telemetry.addData("Turret Power", turretPower);

        telemetry.setAutoClear(true);

        // Telemetry
        panelsTelemetry.addData("distance to target (in)", distanceToTarget);
        panelsTelemetry.addData("target velocity IPS", velocityIPS);
        panelsTelemetry.addData("target velocity TPS", velocityTPS);
        panelsTelemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        panelsTelemetry.addData("additive", adder);
        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("hue", hueIntake);
        panelsTelemetry.addData("indexer color 1", indexerColor1);
        panelsTelemetry.addData("indexer color 2", indexerColor2);
        panelsTelemetry.addData("indexer color 3", indexerColor3);
        panelsTelemetry.addData("kS", kS);
        panelsTelemetry.addData("kV", kV);

        follower.update();

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
}