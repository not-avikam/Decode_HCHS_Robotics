package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "lm2 teleop drive", group = "lm2 2025")
public class launcher_velocity_calculator extends OpMode {

    public static double velocity_servo = 0;
    public static double velocity_intake = 0;
    public static double pitch_position = 0;

    // Adjustable shot parameters
    public static double distanceToTarget = 60; // inches
    public static double launchAngleDeg = 45;   // degrees

    private MotorEx launcher;
    private CRServoEx crservo1;
    private CRServoEx crservo2;
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
    public static Pose startingPose;
    private TelemetryManager telemetryM;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        crservo1 = new CRServoEx(hardwareMap, "yaw1");
        crservo2 = new CRServoEx(hardwareMap, "yaw2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(0.6, 0, 0);
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

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
            launcher.setVelocity(velocityTPS);
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
        if (gamepad1.y) {
            distanceToTarget += 5;
        } else if (gamepad1.a) {
            distanceToTarget -= 5;
        }

        // Other mechanisms
        crservo1.set(velocity_servo);
        crservo2.set(-velocity_servo);
        intake.setPower(velocity_intake);
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

        // Telemetry
        panelsTelemetry.addData("Distance (in)", distanceToTarget);
        panelsTelemetry.addData("Velocity IPS", velocityIPS);
        panelsTelemetry.addData("Velocity TPS", velocityTPS);
        panelsTelemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        panelsTelemetry.update(telemetry);

        telemetry.update();
    }
}
