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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeUnit;

//TODO: Use a color sensor to verify that hue values are correct

@Autonomous(name="Auto Red HSI", group="HSI LM2")
public class Auton_Red extends OpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    public static int detected_obelisk;
    private MotorEx launcher = null;
    private DcMotorEx intake = null;
    private ServoEx agigtator = null;
    private ServoEx pitch = null;
    private CRServoEx yaw1, yaw2 = null;
    private ServoEx indexer = null;
    NormalizedColorSensor colorSensor;
    private final Pose startPose = new Pose(117,128, Math.toRadians(45));
    private final Pose scorePreload = new Pose(97, 84, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scoreFace = new Pose(137,142);
    private final Pose pickup1pose = new Pose(127, 83, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(88, 76, Math.toRadians(45));
    private final Pose scorePose3 = new Pose(81, 67, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(133, 57, Math.toRadians(0));
    private final Pose pickup2Posectrl = new Pose(88, 59);
    private final Pose pickup3Pose = new Pose(132, 35, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Posectrl = new Pose(90, 32);
    private final Pose scorePose4 = new Pose(80, 16, Math.toRadians(67));
    private final Pose human = new Pose (134,42, Math.toRadians(90));
    double velocity;
    private double kP = .01;
    private double pitchAngleDegrees;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;

    private int pathState;
    private int shootBall;
    private int intakeBall1, intakeBall2, intakeBall3;
    private PathChain preload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, goToHuman;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        preload = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,  scorePreload))
                .setHeadingInterpolation(HeadingInterpolator.linear(45, 0))
                .build();
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePreload,  pickup1pose))
                .setConstantHeadingInterpolation(0)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1pose, scorePose2))
                .setConstantHeadingInterpolation(0)
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, pickup2Posectrl, pickup2Pose))
                .setConstantHeadingInterpolation(0)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose3))
                .setConstantHeadingInterpolation(0)
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose3, pickup3Posectrl, pickup3Pose))
                .setConstantHeadingInterpolation(0)
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose4))
                .setConstantHeadingInterpolation(0)
                .build();

        goToHuman = follower.pathBuilder()
                .addPath(new BezierLine(scorePose4, human))
                .setLinearHeadingInterpolation(0, 90)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload, false);
                setPathState(1);
                break;
            case 1:
                setShootBall(0);
                shoot();
                if (shootBall >= 6) {
                    setPathState(2);
                }
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
                setShootBall(0);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
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
                setShootBall(0);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
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
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
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
                indexer.set(300);
                if ((launcher.getVelocity()) >= (velocity-100) & launcher.getVelocity() <= (velocity+100)) {
                    agigtator.set(.3);
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.set(0);
                    actionTimer.resetTimer();
                    setShootBall(2);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.set(240);
                    actionTimer.resetTimer();
                    setShootBall(3);
                }
                break;
            case 3:
                if ((launcher.getVelocity()) >= (velocity-100) && launcher.getVelocity() <= (velocity+100) && actionTimer.getElapsedTimeSeconds() >= .2) {
                    agigtator.set(.3);
                    actionTimer.resetTimer();
                    setShootBall(4);
                }
                break;
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.set(0);
                    actionTimer.resetTimer();
                    setShootBall(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.set(170);
                    actionTimer.resetTimer();
                    setShootBall(6);
                }
                break;
            case 6:
                if ((launcher.getVelocity()) >= (velocity-100) & launcher.getVelocity() <= (velocity+100) && actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.set(.3);
                    actionTimer.resetTimer();
                    setShootBall(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.set(0);
                    actionTimer.resetTimer();
                    setShootBall(8);
                }
                break;
            case 8:
                launcher.setVelocity(0);
                if (actionTimer.getElapsedTimeSeconds() > .2) {
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

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        setIntakeBall1(0);
        pathState = pState;
        pathTimer.resetTimer();

    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 && detection.metadata !=null) {
                yaw1.set(.02*detection.ftcPose.bearing);
                yaw2.set(.02*detection.ftcPose.bearing);
                if (detection.ftcPose.pitch > 110 && detection.ftcPose.pitch < 140){
                    pitchAngleDegrees = detection.ftcPose.pitch*11.25;
                } else if (detection.ftcPose.pitch > 140){
                    pitchAngleDegrees = 140;
                } else if (detection.ftcPose.pitch < 110) {
                    pitchAngleDegrees = 110;
                } else {
                    pitchAngleDegrees = 125;
                }
                pitch.set(pitchAngleDegrees);
                double theta = Math.toRadians(pitchAngleDegrees);
                double R = detection.ftcPose.range;
                double h = detection.ftcPose.elevation;
                double g = 9.8;
                double numerator = g * R * R;
                double denominator = 2 * Math.pow(Math.cos(theta), 2) * (R * Math.tan(theta) - h);
                velocity = (Math.sqrt(numerator / denominator))*142.239908137;
                if (follower.getCurrentPathChain() == preload || follower.getCurrentPathChain() == scorePickup1 || follower.getCurrentPathChain() == scorePickup2 || follower.getCurrentPathChain() == scorePickup3) {
                    launcher.set(velocity);
                } else {
                    launcher.set(0);
                }
                telemetry.addData("target velocity", velocity);
                telemetry.addData("current velocity", launcher.getVelocity());
                telemetry.addData("pitch angle", pitchAngleDegrees);
            } else if (currentDetections.isEmpty()) {
                yaw1.set(0);
                yaw2.set(0);
                launcher.set(0);
            }
        }   // end for() loop

        if (follower.getCurrentPathChain() == grabPickup1 || follower.getCurrentPathChain() == grabPickup2 || follower.getCurrentPathChain() == grabPickup3) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = new ServoEx(hardwareMap, "agigtator");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        yaw2 = new CRServoEx(hardwareMap, "yaw2");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        launcher = new MotorEx(hardwareMap, "launcher");

        initAprilTag();

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        agigtator.setInverted(true);

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        pitch.set(1462.5);

        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        //ensures that all servos start at the correct position
        agigtator.set(0);
        indexer.set(0);
        //sets pidf values for the launcher
        launcher.setVeloCoefficients(.03, .8, .05);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        indexer.set(0);
        agigtator.set(0);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
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
}