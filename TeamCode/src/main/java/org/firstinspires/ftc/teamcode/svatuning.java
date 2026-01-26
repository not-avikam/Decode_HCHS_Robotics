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
@TeleOp(name = "feedforward tuner", group = "tun")
public class svatuning extends OpMode {
    SimpleMotorFeedforward launcherfeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double velocity;
    private MotorEx launcher;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        launcher.setVeloCoefficients(0.6, 0, 0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        launcher.setVelocity(velocity);

        panelsTelemetry.addData("Velocity Target", velocity);
        panelsTelemetry.addData("Velocity actual", launcher.getVelocity());

        panelsTelemetry.update();
        //return 0;
    }

}
