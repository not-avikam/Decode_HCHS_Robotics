package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//TODO: Use a color sensor to verify that hue values are correct

@Autonomous(name="Auto", group="HSI LM2")
public class Auton_Red extends OpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection detection;
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    public static int detected_obelisk;
    private MotorEx launcher = null;
    private DcMotorEx intake = null;
    private ServoEx agigtator = null;
    private Servo pitch = null;
    private CRServoEx yaw1, yaw2 = null;
    private ServoEx indexer = null;
    NormalizedColorSensor colorSensor;
    private final Pose startPose = new Pose(117,128, Math.toRadians(45));
    private final Pose scorePreload = new Pose(84, 84, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scoreFace = new Pose(137,142);
    private final Pose pickup1pose = new Pose(131, 83, Math.toRadians(0));
    private final Pose pickup1posectrl = new Pose(109,82);
    private final Pose scorePose2 = new Pose(83, 82, Math.toRadians(45));
    private final Pose scorePose3 = new Pose(75, 75, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(132, 61, Math.toRadians(0));
    private final Pose pickup2Posectrl = new Pose(85, 50);
    private final Pose pickup3Pose = new Pose(130, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Posectrl = new Pose(66, 30);
    private final Pose scorePose4 = new Pose(80, 16, Math.toRadians(67));
    private final Pose human = new Pose (10,10, Math.toRadians(180));
    private double kP = .01;

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
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePreload))
                .build();
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreload, pickup1posectrl,  pickup1pose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pickup1pose))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1pose, scorePose2))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(scoreFace))
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, pickup2Posectrl, pickup2Pose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pickup2Pose))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose3))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(scoreFace))
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose3, pickup3Posectrl, pickup3Pose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pickup3Pose))
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose4))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(scoreFace))
                .build();

        goToHuman = follower.pathBuilder()
                .addPath(new BezierLine(scorePose4, human))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), human.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload, false);
            case 1:
                setShootBall(0);
                shoot();
                if (shootBall >= 6) {
                    setPathState(1);
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
                    setPathState(2);
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
                    setPathState(3);
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
                        setPathState(4);
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
                    setPathState(5);
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
                        setPathState(6);
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
                    setPathState(7);
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
                        setPathState(-1);
                    }
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
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall1(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    intake.setPower(0);
                    setIntakeBall1(2);
                }
                break;
            case 2:
                setIntakeBall1(3);
                break;
        }

        switch (intakeBall2) {
            case 0:
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    intake.setPower(0);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                setIntakeBall2(3);
                break;
        }

        switch (intakeBall3) {
            case 0:
                indexer.set(270);
                intake.setPower(1);
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
                    indexer.set(130);
                    intake.setPower(0);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    intake.setPower(0);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                setIntakeBall2(3);
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
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall1(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(0);
                    intake.setPower(0);
                    setIntakeBall1(2);
                }
                break;
            case 2:
                setIntakeBall1(3);
                break;
        }

        switch (intakeBall2) {
            case 0:
                indexer.set(130);
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    intake.setPower(0);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                setIntakeBall2(3);
                break;
        }

        switch (intakeBall3) {
            case 0:
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    intake.setPower(0);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                setIntakeBall2(3);
                break;
        }

    }

    public void intakePGP() {

        double hue;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        switch (intakeBall1) {
            case 0:
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall1(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(130);
                    intake.setPower(0);
                    setIntakeBall1(2);
                }
                break;
            case 2:
                setIntakeBall1(3);
                break;
        }

        switch (intakeBall2) {
            case 0:
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    intake.setPower(0);
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
                intake.setPower(1);
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
                    intake.setPower(0);
                    setIntakeBall2(2);
                } else if (hue > 90 && hue < 150) {
                    indexer.set(270);
                    intake.setPower(0);
                    setIntakeBall2(2);
                }
                break;
            case 2:
                setIntakeBall2(3);
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
                launcher.set(1);
                indexer.set(300);
                if ((launcher.getVelocity()) >= (2300)) {
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
                if ((launcher.getVelocity()) >= (2300) && actionTimer.getElapsedTimeSeconds() >= .2) {
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
                if ((launcher.getVelocity()) >= (2300) && actionTimer.getElapsedTimeSeconds() > .2) {
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
                launcher.setVelocity(.3);
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
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        double distanceToTarget = Math.sqrt(
                Math.pow(follower.getPose().getX() - 137, 2) +
                        Math.pow(follower.getPose().getY() - 142, 2)
        );

        double launchAngle = Math.toDegrees(Math.atan2(54, distanceToTarget));

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        if (detection.id == 24) {
            yaw1.set(detection.ftcPose.bearing * kP);
            yaw2.set(detection.ftcPose.bearing * kP);
        } else {
            yaw1.set(1);
            yaw2.set(1);
        }


        telemetry.addData("Launch Angle", launchAngle);

        pitch.setPosition(launchAngle);

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
        launcher = new MotorEx(hardwareMap, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = new ServoEx(hardwareMap, "agigtator");
        pitch = hardwareMap.get(Servo.class, "pitch");
        indexer = new ServoEx(hardwareMap, "indexer", 1, AngleUnit.DEGREES);
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        yaw2 = new CRServoEx(hardwareMap, "yaw2");

        yaw2.setInverted(true);

        yaw1.setPIDF(new PIDFCoefficients(kP, 0.0, 0.1, 0.0001));
        yaw2.setPIDF(new PIDFCoefficients(kP, 0, 0.1, .0001));

        initAprilTag();

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        agigtator.setInverted(true);


        launcher.setRunMode(MotorEx.RunMode.RawPower);


        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        //ensures that all servos start at the correct position
        agigtator.set(0);
        indexer.set(90);
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
        if (detection.id==PPG_TAG_ID) {
            detected_obelisk = PPG_TAG_ID;
            telemetry.addLine("Purple Purple Green detected");
        } else if (detection.id==PGP_TAG_ID) {
            detected_obelisk = PGP_TAG_ID;
            telemetry.addLine("Purple Green Purple detected");
        } else if (detection.id==GPP_TAG_ID) {
            detected_obelisk = GPP_TAG_ID;
            telemetry.addLine("Green Purple Purple detected");
        } else {
            telemetry.addLine("No Tag Detected");
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
        yaw1.set(Math.toRadians(0));
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
}