package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous(name = "new bot auton", group = "lm2 2025")
public class lm3_decode extends OpMode {

    private Motor frontLeftDrive = null;
    private Motor backLeftDrive = null;
    private Motor frontRightDrive = null;
    private Motor backRightDrive = null;
    private ServoEx indexer;
    private ServoEx pitch;

    private MotorEx launcher;
    private Timer actionTimer;
    private Servo agigtator;
    int shootBall;
    double targetVelocity = 2000;
    double[] SHOOT_POS = {162, 228, 295.5};

    @Override
    public void init() {
        frontLeftDrive = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        backLeftDrive = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);
        frontRightDrive = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        backRightDrive = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        launcher = new MotorEx(hardwareMap, "launcher");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);

        frontLeftDrive.setRunMode(Motor.RunMode.RawPower);
        frontRightDrive.setRunMode(Motor.RunMode.RawPower);
        backLeftDrive.setRunMode(Motor.RunMode.RawPower);
        backRightDrive.setRunMode(Motor.RunMode.RawPower);

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(0.6, 0, 0);

        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);

        actionTimer = new Timer();

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
        pitch.set(155);


        launcher.set(targetVelocity);

        switch (shootBall) {

            case 0:
                indexer.set(162);
                if ((launcher.getVelocity()) >= (2000 - 150) && launcher.getVelocity() <= (2000 + 250)) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    agigtator.setPosition(0.3);
                    actionTimer.resetTimer();
                    setShootBall(2);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    indexer.set(228);
                    actionTimer.resetTimer();
                    setShootBall(3);
                }
                break;
            case 3:
                if ((launcher.getVelocity()) >= (2000 - 150) && launcher.getVelocity() <= (2000 + 250) && actionTimer.getElapsedTimeSeconds() >= .5) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(4);
                }
                break;
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    agigtator.setPosition(0.3);
                    actionTimer.resetTimer();
                    setShootBall(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    indexer.set(295.5);
                    actionTimer.resetTimer();
                    setShootBall(6);
                }
                break;
            case 6:
                if ((launcher.getVelocity()) >= (2000 - 150) && launcher.getVelocity() <= (2000 + 250) && actionTimer.getElapsedTimeSeconds() > .5) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    agigtator.setPosition(0.3);
                    actionTimer.resetTimer();
                    setShootBall(8);
                }
                break;
            case 8:
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    indexer.set(0);
                    actionTimer.resetTimer();
                    setShootBall(9);
                    frontLeftDrive.set(1);
                    backRightDrive.set(-1);
                    backLeftDrive.set(-1);
                    frontRightDrive.set(1);
                }
                break;
        }


        telemetry.addData("tuurret velocity", launcher.getVelocity());
        telemetry.addData("sball", shootBall);
        telemetry.update();


    }

    public void setShootBall(int sBall) {
        shootBall = sBall;
    }
}