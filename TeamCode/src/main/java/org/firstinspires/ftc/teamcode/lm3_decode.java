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
    boolean visionAvailable = false;
    boolean visionActive = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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

    //ID | Pattern
    //21 | GPP
    //22 | PGP
    //23 | PPG
    int currentPattern = 23;
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

        AprilTagDetection tag21 = null;
        AprilTagDetection tag22 = null;
        AprilTagDetection tag23 = null;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == 21 && detection.metadata != null) {
                tag21 = detection;
                break; // lock onto ONLY tag 24
            }

            if (detection.id == 22 && detection.metadata != null) {
                tag22 = detection;
                break;
            }

            if (detection.id == 23 && detection.metadata != null) {
                tag23 = detection;
                break;
            }

        }
        /*

        if (tag21 != null && ) {
            currentPattern = 21;
        } else if (tag22 != null && ) {
            currentPattern = 22;
        } else if (tag23 != null && ) {
            currentPattern = 23;
        }

         */

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

        public void setShootBall ( int sBall){
            shootBall = sBall;
        }

        public void shootBallSorted () {

        }

        private void initAprilTag () {
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
                telemetry.addData("Error", e.getMessage());
            }
        }
}