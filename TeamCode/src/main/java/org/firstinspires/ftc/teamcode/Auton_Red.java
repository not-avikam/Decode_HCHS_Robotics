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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

@Autonomous(name="Auto", group="StarterBot")

public class Auton_Red extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private CRServo agigtator = null;
    private Servo pitch = null;


    RobotHardware robot = new RobotHardware(this);

    private final Pose startPose = new Pose(87, 88, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(132, 134);
    private final Pose scoreFace = new Pose(137,142);
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1pose = new Pose(131, 83, Math.toRadians(0));
    private final Pose pickup1posectrl = new Pose(109,82);
    private final Pose scorePose2 = new Pose(83, 82, Math.toRadians(45));
    private final Pose scorePose3 = new Pose(75, 75, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(132, 61, Math.toRadians(0));
    private final Pose pickup2Posectrl = new Pose(859, 50);
    private final Pose pickup3Pose = new Pose(130, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Posectrl = new Pose(66, 30);
    private final Pose scorePose4 = new Pose(80, 16, Math.toRadians(67));
    private final Pose human = new Pose (10,10, Math.toRadians(180));



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shot;

    private int pathState;

    private double launchAngle;

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, goToHuman;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        //scorePreload = new Path(new BezierLine(startPose, scorePose));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, pickup1posectrl,  pickup1Pose))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pickup1Pose))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose2))
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
                robot.init();
                shoot();
                if(pathTimer.getElapsedTimeSeconds() > 5){
                    setPathState(1);
                }
                break;
            case 1:
                intake.setPower(1);
                agigtator.setPower(.4);


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
            case 2:
                if (follower.isBusy()) {
                    intake.setPower(1);
                    agigtator.setPower(.2);
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.isBusy()) {
                    intake.setPower(.5);
                    agigtator.setPower(.2);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (pathTimer.getElapsedTimeSeconds() > 5){
                        follower.followPath(grabPickup2,true);
                        setPathState(4);
                    }
                }
                break;
            case 4:

                if(follower.isBusy()) {
                    intake.setPower(1);
                    agigtator.setPower(.2);
                }
                //TODO

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        follower.followPath(grabPickup3,true);
                        setPathState(6);
                    }
                }
                break;
            case 6:

                if(follower.isBusy()) {
                    intake.setPower(1);
                    agigtator.setPower(.2);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    shoot();
                    if ((pathTimer.getElapsedTimeSeconds()) > 5) {
                        follower.followPath(goToHuman, true);
                        /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                        setPathState(-1);
                    }
                }
                break;
        }
    }

    public void shoot() {
        // TODO: tune these
        double distanceToTarget = Math.sqrt(
                Math.pow(follower.getPose().getX() - 132, 2) +
                        Math.pow(follower.getPose().getY() - 134, 2)
        );
        double launchAngle = Math.toRadians(45.0);
        double goalHeightDifference = 1.111; // Example: 10 inches = 0.254 meters
        double g = 9.81; // gravity m/s^2

        double cosTheta = Math.cos(launchAngle);
        double tanTheta = Math.tan(launchAngle);

// Denominator = 2*cos^2(theta)*(x*tan(theta)-y)
        double denominator = 2 * cosTheta * cosTheta * (distanceToTarget * tanTheta - goalHeightDifference);

        double v0Squared = (g * Math.pow(distanceToTarget, 2)) / denominator;

        double launcherTarget = Math.sqrt(v0Squared); // final launch speed m/s

        shot = new Timer();

        if ((launcher.getVelocity()) == (launcherTarget*1425.1)) {
            agigtator.setPower(1);
        }
        launcher.setVelocity(launcherTarget*1425.1);

        if (launcher.getVelocity() == launcherTarget) {
            shot.resetTimer();
            agigtator.setPower(1);
        }

        if ((shot.getElapsedTimeSeconds()) > 2) {
            launcher.setVelocity(0);
            agigtator.setPower(0);
        }

    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void
    loop() {

        double distanceToTarget = Math.sqrt(
                Math.pow(follower.getPose().getX() - 137, 2) +
                        Math.pow(follower.getPose().getY() - 142, 2)
        );
        double goalHeightDifference = 54;

        double launchAngle = Math.acos(goalHeightDifference/distanceToTarget);

        telemetry.addData("Launch Angle", launchAngle);

        pitch.setPosition(launchAngle/300);

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

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)// use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(CRServo.class, "agigtator");
        pitch = hardwareMap.get(Servo.class, "pitch");



        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}