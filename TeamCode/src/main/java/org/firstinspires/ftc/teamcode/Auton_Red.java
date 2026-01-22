package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeUnit;
@Autonomous(name="Auto Red HSI", group="lm2 2025")
public class Auton_Red extends OpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;

    //ID | Pattern
    //21 | GPP
    //22 | PGP
    //23 | PPG
    private int detected_obelisk = 23;
    private MotorEx launcher = null;
    private Motor intake = null;
    private Servo agigtator = null;
    private ServoEx pitch = null;
    private ServoEx yaw1 = null;
    private IMU imu;
    private ServoEx indexer = null;
    NormalizedColorSensor colorSensor;
    private final Pose startPose = new Pose(87,9, Math.toRadians(0));
    private double kP = .01;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;

    private int pathState;
    private int shootBall;
    private int ballsShot = 0;
    private int intakeBall1, intakeBall2, intakeBall3;
    private PathChain score1, pickup1, score2, pickup2, score3, pickup3;
    double velocityIPS = 0;
    double velocityTPS = 1300;
    public static double launchAngleDeg = 45;   // degrees
    double atX;
    double atY;
    double imuHeading;
    YawPitchRollAngles orientation;
    private boolean pitchShoot = false;
    double pitchAngle = .12;
    private double adder = 200;
    boolean visionAvailable = false;
    boolean visionActive = false;
    public void buildPaths() {

        pickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.562, 9.127),

                                new Pose(115.742, 34.873)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(16))

                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(115.742, 34.873),

                                new Pose(85.098, 20.667)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(45))

                .build();

        pickup2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.098, 20.667),
                                new Pose(100.224, 63.386),
                                new Pose(116.587, 58.564)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(116.587, 58.564),
                                new Pose(154.000, 80.000),
                                new Pose(94.103, 52.793),
                                new Pose(93.008, 84.110)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();

        pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(93.008, 84.110),

                                new Pose(125.820, 82.996)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.820, 82.996),

                                new Pose(109.620, 94.685)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shootPreload();
                if (ballsShot >= 3) {
                    follower.followPath(pickup1, false);
                    setPathState(1);
                }
                break;
            case 1:
                setIntakeBall1(0);
                intakePGP();
                /*
                if (detected_obelisk == PPG_TAG_ID) {
                    intakePPG();
                } else if (detected_obelisk == PGP_TAG_ID) {
                    intakePGP();
                } else if (detected_obelisk == GPP_TAG_ID) {
                    intakeGPP();
                }
                */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (shootBall >= 6){
                        follower.followPath(pickup2,false);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                setIntakeBall2(0);
                if (detected_obelisk == PPG_TAG_ID) {
                    intakePPG();
                } else if (detected_obelisk == PGP_TAG_ID) {
                    intakePGP();
                } else if (detected_obelisk == GPP_TAG_ID) {
                    intakeGPP();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Score Sample */
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (shootBall >= 6) {
                        follower.followPath(pickup3,false);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                setIntakeBall3(0);
                if (detected_obelisk == PPG_TAG_ID) {
                    intakePPG();
                } else if (detected_obelisk == PGP_TAG_ID) {
                    intakePGP();
                } else if (detected_obelisk == GPP_TAG_ID) {
                    intakeGPP();
                }
                if(!follower.isBusy()) {
                    telemetry.addLine("Autonomous Routine Completed");
                    telemetry.addLine("Good Luck!");
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score3, true);
                    setPathState(8);
                }
                break;
            case 8:
                setShootBall(0);
                if(!follower.isBusy()) {
                    shoot();
                    if (shootBall >= 6) {
                        telemetry.addLine("Autonomous Routine Completed");
                        telemetry.addLine("Good Luck!");
                        //follower.followPath(goToHuman, true);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (detected_obelisk == PPG_TAG_ID) {
                    intakePPG();
                } else if (detected_obelisk == PGP_TAG_ID) {
                    intakePGP();
                } else if (detected_obelisk == GPP_TAG_ID) {
                    intakeGPP();
                }
                if (intakeBall3 == 3) {
                    setPathState(-1);
                    telemetry.addLine("Autonomous Routine Completed");
                    telemetry.addLine("Good Luck!");
                }
                break;

        }
    }


    public void intakePPG() {

        double hue;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        switch (intakeBall1) {
            case 0:
                if (hue > 225 && hue < 350){
                    indexer.set(130);
                    setIntakeBall1(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    setIntakeBall1(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall1(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall1(2);
                }
                break;
            case 2:
                if (hue > 225 && hue < 350){
                    setIntakeBall1(3);
                } else if (hue > 90 && hue < 150) {
                    setIntakeBall1(3);
                }
                break;
        }

        switch (intakeBall2) {
            case 0:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall2(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall2(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(130);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                if (hue > 225 && hue < 350){
                    setIntakeBall2(3);
                } else if (hue > 90 && hue < 150) {
                    setIntakeBall2(3);
                }
                break;
        }

        switch (intakeBall3) {
            case 0:
                indexer.set(270);
                if (hue > 225 && hue < 350){
                    indexer.set(0);
                    setIntakeBall3(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(0);
                    setIntakeBall3(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(130);
                    setIntakeBall3(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    setIntakeBall3(2);
                }
                break;
            case 2:
                if (hue > 225 && hue < 350){
                    setIntakeBall3(3);
                } else if (hue > 90 && hue < 150) {
                    setIntakeBall3(3);
                }
                break;
        }

    }

    public void intakeGPP() {

        double hue;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        switch (intakeBall1) {
            case 0:
                indexer.set(130);
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall1(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall1(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(0);
                    setIntakeBall1(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(0);
                    setIntakeBall1(2);
                }
                break;
            case 2:
                if (hue > 225 && hue < 350){
                    setIntakeBall1(3);
                } else if (hue > 90 && hue < 150) {
                    setIntakeBall1(3);
                }
                break;
        }

        switch (intakeBall2) {
            case 0:
                indexer.set(130);
                if (hue > 225 && hue < 350){
                    indexer.set(0);
                    setIntakeBall2(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(0);
                    setIntakeBall2(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                if (hue > 225 && hue < 350){
                    setIntakeBall2(3);
                } else if (hue > 90 && hue < 150) {
                    setIntakeBall2(3);
                }
                break;
        }

        switch (intakeBall3) {
            case 0:
                if (hue > 225 && hue < 350){
                    indexer.set(130);
                    setIntakeBall3(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    setIntakeBall3(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall3(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall3(2);
                }
                break;
            case 2:
                if (hue > 225 && hue < 350){
                    setIntakeBall3(3);
                } else if (hue > 90 && hue < 150) {
                    setIntakeBall3(3);
                }
                break;
        }

    }

    public void intakePGP() {

        double hue;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        switch (intakeBall1) {
            case 0:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall1(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall1(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(130);
                    setIntakeBall1(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    setIntakeBall1(2);
                }
                break;
            case 2:
                setIntakeBall1(3);
                break;
        }

        switch (intakeBall2) {
            case 0:
                if (hue > 225 && hue < 350){
                    indexer.set(130);
                    setIntakeBall2(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    setIntakeBall2(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                setIntakeBall2(3);
                break;
        }

        switch (intakeBall3) {
            case 0:
                indexer.set(130);
                if (hue > 225 && hue < 350){
                    indexer.set(0);
                    setIntakeBall3(1);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(0);
                    setIntakeBall3(1);
                }
                break;
            case 1:
                if (hue > 225 && hue < 350){
                    indexer.set(270);
                    setIntakeBall3(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    setIntakeBall3(2);
                }
                break;
            case 2:
                setIntakeBall3(3);
                break;
        }

    }

    public void setIntakeBall1(int iBall) {
        intakeBall1 = iBall;
    }

    public void setIntakeBall2(int iiBall) {
        intakeBall2 = iiBall;
    }

    public void setIntakeBall3(int iiiBall) {
        intakeBall3 = iiiBall;
    }

    public void shoot() {

        switch (shootBall) {
            case 0:
                follower.holdPoint(follower.getPose());
                indexer.set(300);
                pitchShoot = true;
                if ((launcher.getVelocity()) >= (velocityTPS-150) && launcher.getVelocity() <= (velocityTPS+150)) {
                    agigtator.setPosition(0);
                    ballsShot ++;
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0.3);
                    actionTimer.resetTimer();
                    setShootBall(2);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.set(228);
                    actionTimer.resetTimer();
                    setShootBall(3);
                }
                break;
            case 3:
                if ((launcher.getVelocity()) >= (velocityTPS-150) && launcher.getVelocity() <= (velocityTPS+150) && actionTimer.getElapsedTimeSeconds() >= .2) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(4);
                }
                break;
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0.3);
                    actionTimer.resetTimer();
                    setShootBall(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.set(162);
                    actionTimer.resetTimer();
                    setShootBall(6);
                }
                break;
            case 6:
                if ((launcher.getVelocity()) >= (velocityTPS-150) && launcher.getVelocity() <= (velocityTPS+150) && actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0.3);
                    actionTimer.resetTimer();
                    setShootBall(8);
                }
                break;
            case 8:
                follower.breakFollowing();
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    pitchShoot = false;
                    indexer.set(0);
                    actionTimer.resetTimer();
                    setShootBall(9);
                }
                break;
        }

    }

    public void setShootBall(int sBall) {
        shootBall = sBall;
        shootTimer.resetTimer();
    }

    public void shootPreload() {

        if (detected_obelisk == GPP_TAG_ID) {
            indexer.set(295.5);
        } else {
            indexer.set(228);
        }

        switch (shootBall) {
            case 0:
                follower.holdPoint(follower.getPose());
                pitchShoot = true;
                if ((launcher.getVelocity()) >= (velocityTPS-100) && launcher.getVelocity() <= (velocityTPS+100)) {
                    agigtator.setPosition(0);
                    ballsShot ++;
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
                    if (detected_obelisk == PGP_TAG_ID) {
                        indexer.set(295.5);
                    } else if (detected_obelisk == PPG_TAG_ID){
                        indexer.set(162);
                    } else {
                        indexer.set(228);
                    }
                    actionTimer.resetTimer();
                    setShootBall(3);
                }
                break;
            case 3:
                if ((launcher.getVelocity()) >= (velocityTPS-100) && launcher.getVelocity() <= (velocityTPS+100) && actionTimer.getElapsedTimeSeconds() >= .5) {
                    agigtator.setPosition(0);
                    ballsShot ++;
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
                    if (detected_obelisk == PPG_TAG_ID) {
                        indexer.set(295.5);
                    } else {
                        indexer.set(162);
                    }
                    actionTimer.resetTimer();
                    setShootBall(6);
                }
                break;
            case 6:
                if ((launcher.getVelocity()) >= (velocityTPS-100) && launcher.getVelocity() <= (velocityTPS+100) && actionTimer.getElapsedTimeSeconds() > .5) {
                    agigtator.setPosition(0);
                    ballsShot ++;
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
                follower.breakFollowing();
                if (actionTimer.getElapsedTimeSeconds() > .5) {
                    pitchShoot = false;
                    indexer.set(0);
                    actionTimer.resetTimer();
                    setShootBall(9);
                }
                break;
        }

    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        setShootBall(0);
        ballsShot = 0;
        pathState = pState;
        pathTimer.resetTimer();

    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        double height = 30;

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

        if (tag21 != null && follower.getCurrentPathChain() == pickup1) {
            detected_obelisk = 21;
        } else if (tag22 != null && follower.getCurrentPathChain() == pickup1) {
            detected_obelisk = 22;
        } else if (tag23 != null && follower.getCurrentPathChain() == pickup1) {
            detected_obelisk = 23;
        }


        telemetry.addData("tuurret velocity", launcher.getVelocity());
        telemetry.addData("sball", shootBall);
        telemetry.update();


        pitch.set(pitchAngle);
        double theta = Math.toRadians(45);
        double deltaX = follower.getPose().getX() - 132.33092417423273;
        double deltaY = follower.getPose().getY() - 136.3625744877852;
        double distanceToTarget = Math.sqrt(Math.pow(deltaX, 2) - Math.pow(deltaY, 2));

        // Physics denominator
        double denom =
                2 * Math.pow(Math.cos(theta), 2) *
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

        if (follower.getCurrentPathChain() == score1 || follower.getCurrentPathChain() == score2 || follower.getCurrentPathChain() == score3) {
            launcher.setVelocity(velocityTPS);
        }

        if (follower.getCurrentPathChain() == pickup1 || follower.getCurrentPathChain() == pickup2 || follower.getCurrentPathChain() == pickup3) {
            intake.set(1);
        }

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        if (follower.isBusy()) {
            telemetry.addLine("follower is busy");
        } else {
            telemetry.addLine("follower is finished");
        }
        telemetry.addData("launcher velocity target", velocityTPS);
        telemetry.addData("actual launcher velocity", launcher.getVelocity());
        telemetry.addData("distance to target", distanceToTarget);
        telemetry.addData("motif", detected_obelisk);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("shoot ball", shootBall);
        telemetry.addData("intake ball 1", intakeBall1);
        telemetry.addData("intake ball 2", intakeBall2);
        telemetry.addData("intake ball 3", intakeBall3);

        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new ServoEx(hardwareMap, "yaw1");

         orientation = imu.getRobotYawPitchRollAngles();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        shootBall = 0;
        ballsShot = 0;

        initAprilTag();

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setInverted(true);

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        //pitch.set(1462.5);

        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        //ensures that all servos start at the correct position
        agigtator.setPosition(0);
        indexer.set(0);
        //sets pid values for the launcher
        launcher.setVeloCoefficients(0.6, 0, 0);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        shootTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        indexer.set(0);
        pitch.set(70*5.9);
        agigtator.setPosition(0.3);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setShootBall(0);
        setIntakeBall1(0);
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

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