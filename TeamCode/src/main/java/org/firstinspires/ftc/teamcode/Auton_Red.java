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
    public static int detected_obelisk = 23;
    private MotorEx launcher = null;
    private CRServoEx intake = null;
    private Servo agigtator = null;
    private ServoEx pitch = null;
    private MotorEx yaw1 = null;
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
    private PathChain preload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, goToHuman;
    double velocityIPS = 0;
    double velocityTPS = 1300;
    public static double launchAngleDeg = 45;   // degrees
    public static double distanceToTarget = 60; // inches
    double atX;
    double atY;
    double imuHeading;
    YawPitchRollAngles orientation;
    private boolean pitchShoot = false;
    double pitchAngle = .12;
    private double adder = 200;
    public void buildPaths() {

        preload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.000, 9.000),

                                new Pose(102.241, 35.334)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        grabPickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.241, 35.334),

                                new Pose(134.471, 35.329)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        scorePickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.471, 35.329),

                                new Pose(87.002, 9.116)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        grabPickup2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(87.002, 9.116),
                                new Pose(69.177, 69.328),
                                new Pose(133.710, 57.353)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        scorePickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.710, 57.353),

                                new Pose(81.000, 67.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        grabPickup3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(81.000, 67.000),
                                new Pose(90.136, 82.616),
                                new Pose(126.974, 84.855)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        scorePickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.974, 84.855),

                                new Pose(111.196, 98.607)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        goToHuman = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(111.196, 98.607),

                                new Pose(119.525, 70.201)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shootPreload();
                if (ballsShot >= 3) {
                    follower.followPath(preload, false);
                    setPathState(1);
                }
                break;
            case 1:

                break;
            case 2:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,false);
                    setPathState(3);
                }
                break;
            case 3:
                setIntakeBall1(0);
                if (detected_obelisk == PPG_TAG_ID) {
                    intakePPG();
                } else if (detected_obelisk == PGP_TAG_ID) {
                    intakePGP();
                } else if (detected_obelisk == GPP_TAG_ID) {
                    intakeGPP();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (shootBall >= 6){
                        follower.followPath(grabPickup2,false);
                        setPathState(5);
                    }
                }
                break;
            case 5:
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
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    /* Score Sample */
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (shootBall >= 6) {
                        follower.followPath(grabPickup3,false);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                setIntakeBall3(0);
                if (detected_obelisk == PPG_TAG_ID) {
                    intakePPG();
                } else if (detected_obelisk == PGP_TAG_ID) {
                    intakePGP();
                } else if (detected_obelisk == GPP_TAG_ID) {
                    intakeGPP();
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(8);
                }
                break;
            case 8:
                setShootBall(0);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    shoot();
                    if (shootBall >= 6) {
                        follower.followPath(goToHuman, true);
                        /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
            indexer.set(300);
        } else {
            indexer.set(228);
        }

        switch (shootBall) {

            case 0:
                follower.holdPoint(follower.getPose());
                pitchShoot = true;
                if ((launcher.getVelocity()) >= (1300-150) && launcher.getVelocity() <= (1300+250)) {
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
                if ((launcher.getVelocity()) >= (1300-150) && launcher.getVelocity() <= (1300+250) && actionTimer.getElapsedTimeSeconds() >= .5) {
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
                if ((launcher.getVelocity()) >= (velocityTPS-150) && launcher.getVelocity() <= (velocityTPS+250) && actionTimer.getElapsedTimeSeconds() > .5) {
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

        double height = 50;

        AprilTagDetection tag24 = null;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == 24 && detection.metadata != null) {
                tag24 = detection;
                break; // lock onto ONLY tag 24
            }
        }

        if (tag24 != null) {
            atX = tag24.ftcPose.x;
            atY = tag24.ftcPose.y;
            imuHeading = orientation.getYaw(AngleUnit.DEGREES);

            yaw1.set(tag24.ftcPose.bearing*.03);

            distanceToTarget = tag24.ftcPose.range;
            if (gamepad1.left_trigger !=0) {
                pitchAngle = 45*5.9;
            } else if (tag24.ftcPose.elevation < 35) {
                pitchAngle = 35 * 5.9;
            } else if (tag24.ftcPose.elevation > 70) {
                pitchAngle = 70 * 5.9;
            } else {
                pitchAngle = ((tag24.ftcPose.elevation*5.9)+30);
            }

            telemetry.addLine("tag detected - forcing pose reset");
            telemetry.addLine("tag detected");
        }

        pitch.set(pitchAngle);
        double theta = Math.toRadians(pitchAngle);

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

        // IPS -> TPS
        if (velocityIPS > 0) {
            velocityTPS = 1300;
        } else {
            velocityTPS = 1300;
        }

        pitch.set(35*5.9);

        launcher.setVelocity((1500));

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        if (follower.isBusy()) {
            telemetry.addLine("follower is busy");
        } else {
            telemetry.addLine("follower is finished");
        }

        // Feedback to Driver Hub
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
        intake = new CRServoEx(hardwareMap, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        imu = hardwareMap.get(IMU.class, "imu");
        yaw1 = new MotorEx(hardwareMap, "yaw1");

         orientation = imu.getRobotYawPitchRollAngles();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // set the run mode
        yaw1.setRunMode(MotorEx.RunMode.RawPower);
        yaw1.stopAndResetEncoder();
        yaw1.setPositionCoefficient(0.05);

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

        for (AprilTagDetection detection : currentDetections) {

            if (detection != null) {
                if (detection.id == PPG_TAG_ID && ((detection.ftcPose.yaw > 15 && detection.ftcPose.yaw < 25) || (detection.ftcPose.yaw > -25 && detection.ftcPose.yaw < -15))) {
                    detected_obelisk = PPG_TAG_ID;
                    telemetry.addLine("Purple Purple Green detected");
                } else if (detection.id == PGP_TAG_ID && ((detection.ftcPose.yaw > 15 && detection.ftcPose.yaw < 25) || (detection.ftcPose.yaw > -25 && detection.ftcPose.yaw < -15))) {
                    detected_obelisk = PGP_TAG_ID;
                    telemetry.addLine("Purple Green Purple detected");
                } else if (detection.id == GPP_TAG_ID && ((detection.ftcPose.yaw > 15 && detection.ftcPose.yaw < 25) || (detection.ftcPose.yaw > -25 && detection.ftcPose.yaw < -15))) {
                    detected_obelisk = GPP_TAG_ID;
                    telemetry.addLine("Green Purple Purple detected");
                } else {
                    telemetry.addLine("No matching Tag Detected in range");
                }
            } else {
                telemetry.addLine("No Tag Detected");
                telemetry.addLine("Defaulting to Purple Purple Green");
                detected_obelisk = PPG_TAG_ID;
            }

            telemetry.update();
            }
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
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

    }   // end method initAprilTag()

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(atX, atY, imuHeading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}