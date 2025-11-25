package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//TODO: Use a color sensor to verify that hue values are correct

@Autonomous(name="Auto", group="HSI LM2")

public class Auton_Blue extends OpMode {

    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private Servo agigtator = null;
    private Servo pitch = null;
    private Servo indexer = null;
    NormalizedColorSensor colorSensor;

    private final Pose startPose = new Pose(60, 84, Math.toRadians(140)); // Start Pose of our robot.
    private final Pose scoreFace = new Pose(7,142);
    private final Pose pickup1pose = new Pose(16, 84, Math.toRadians(180));
    private final Pose pickup1posectrl = new Pose(51,83.5);
    private final Pose scorePose2 = new Pose(67, 76, Math.toRadians(140));
    private final Pose scorePose3 = new Pose(70, 70, Math.toRadians(140));
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180));
    private final Pose pickup2Posectrl = new Pose(59, 60);
    private final Pose pickup3Pose = new Pose(14, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Posectrl = new Pose(66, 32);
    private final Pose scorePose4 = new Pose(67, 20, Math.toRadians(120));
    private final Pose human = new Pose (127,10, Math.toRadians(0));

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;

    private int pathState;
    private int shootBall;
    private int intakeBall;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, goToHuman;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        //scorePreload = new Path(new BezierLine(startPose, scorePose));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, pickup1posectrl,  pickup1pose))
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
                setShootBall(0);
                shoot();
                if (shootBall >= 6) {
                    setPathState(1);
                }
                break;
            case 1:
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
                setIntakeBall(0);
                intake();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
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
            case 4:
                setIntakeBall(0);
                intake();

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
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
            case 6:
                setIntakeBall(0);
                intake();

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
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


    public void intake() {

        double hue;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        switch (intakeBall) {

            case 0:
                intake.setPower(1);
                if (hue > 200 && hue < 320){
                    indexer.setPosition(.333);
                    setIntakeBall(1);
                } else if (hue > 80 && hue < 160) {
                    indexer.setPosition(.333);
                    setIntakeBall(1);
                }
                break;
            case 1:
                if (hue > 200 && hue < 320){
                    indexer.setPosition(.666);
                    intake.setPower(0);
                    setIntakeBall(2);
                } else if (hue > 80 && hue < 160) {
                    indexer.setPosition(.666);
                    intake.setPower(0);
                    setIntakeBall(2);
                }
                break;
            case 2:
                setIntakeBall(3);
                break;
        }

    }

    public void setIntakeBall(int iBall) {
        intakeBall = iBall;
    }

    public void shoot() {



        switch (shootBall) {
            case 0:
                launcher.setPower(1);
                if ((launcher.getVelocity()) >= (2300)) {
                    agigtator.setPosition(.3);
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(2);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.setPosition(.333);
                    actionTimer.resetTimer();
                    setShootBall(3);
                }
                break;
            case 3:
                if ((launcher.getVelocity()) >= (2300) && actionTimer.getElapsedTimeSeconds() >= .2) {
                    agigtator.setPosition(.3);
                    actionTimer.resetTimer();
                    setShootBall(4);
                }
                break;
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.setPosition(.667);
                    actionTimer.resetTimer();
                    setShootBall(6);
                }
                break;
            case 6:
                if ((launcher.getVelocity()) >= (2300) && actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(.3);
                    actionTimer.resetTimer();
                    setShootBall(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    agigtator.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(8);
                }
                break;
            case 8:
                launcher.setVelocity(.3);
                if (actionTimer.getElapsedTimeSeconds() > .2) {
                    indexer.setPosition(0);
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
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        pitch = hardwareMap.get(Servo.class, "pitch");
        indexer = hardwareMap.get(Servo.class, "indexer");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        agigtator.setDirection(Servo.Direction.REVERSE);


        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);


        colorSensor.setGain(2);



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

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(.05, .5, 0.05, .6));


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        indexer.setPosition(0);
        agigtator.setPosition(0);

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
        setShootBall(0);
        setIntakeBall(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}