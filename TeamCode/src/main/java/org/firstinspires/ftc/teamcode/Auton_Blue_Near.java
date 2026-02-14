package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;
@Autonomous(name="Near Blue", group="lcq")
public class Auton_Blue_Near extends OpMode {
    ColorBlobLocatorProcessor colorLocator;
    private Servo light;
    private MotorEx launcher = null;
    private Motor intake = null;
    private Servo trigger = null;
    private Servo agitator = null;
    private ServoEx pitch = null;
    NormalizedColorSensor colorSensor;
    private final Pose startPose = new Pose(17.9325277177655, 121.18266083765477, Math.toRadians(54));
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;
    private boolean visionPathActive = false;
    private int pathState;
    private int shootBall;
    private PathChain score1, pickup1, score2, pickup2, score3, pickup3;
    private final Pose scorePose = new Pose(11.826689774696712, 136.3625744877852);
    public void buildPaths() {

        pickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                (startPose),
                                new Pose(72.828, 65.043),
                                new Pose(25.648, 84.037)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .45,
                                        HeadingInterpolator.facingPoint(scorePose)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .3,
                                        .6,
                                        HeadingInterpolator.linear(Math.toRadians(125), Math.toRadians(72))
                                )))
                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.648, 84.037),

                                new Pose(48.412, 83.808)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose))

                .build();

        pickup2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.412, 83.808),
                                new Pose(61.791, 55.650),
                                new Pose(18.742, 56.581),
                                new Pose(17.260, 65.801)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(180))

                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.260, 65.801),

                                new Pose(57.070, 72.879)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose))

                .build();

        pickup3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.070, 72.879),
                                new Pose(68.611, 34.633),
                                new Pose(26.243, 37.076)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(186))

                .build();

        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.243, 37.076),

                                new Pose(55.964, 13.576)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                    follower.followPath(pickup1, false);
                    if (Math.hypot(follower.getPose().getX() - 53, follower.getPose().getY() - 83) < 1.0) {
                        follower.pausePathFollowing();
                        shoot();
                        if (shootBall >= 2) {
                            follower.resumePathFollowing();
                        }
                    }
                    if (!follower.isBusy() && shootBall >= 2) {
                        setPathState(1);
                    }
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    shoot();
                    if (shootBall >= 2){
                        follower.followPath(pickup2, false);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Score Sample */
                    shoot();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (shootBall >= 2) {
                        follower.followPath(pickup3, false);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(score3, true);
                    setPathState(8);
                }
                break;
            case 8:
                setShootBall(0);
                if(!follower.isBusy()) {
                    shoot();
                    if (shootBall >= 2) {
                        telemetry.addLine("Autonomous Routine Completed");
                        telemetry.addLine("Good Luck!");
                        setPathState(9);
                    }
                }
                break;
                /*
            case 9:
                Pose tunnel = new Pose(22.46100519930676, 44.97920277296362);
                Pose toScore = new Pose(55.963506496589744, 13.57574606490825);

                Path toTunnel = new Path(new BezierLine(follower.getPose(), tunnel));
                Path toScorePath = new Path(new BezierLine(follower.getPose(), toScore));

                toTunnel.setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose));

                if (!follower.isBusy()) {
                    driveToClosestArtifact();
                    break;
                }

                // Step 3: score
                if (!visionPathActive) {
                    follower.followPath(toScorePath, true);
                    shoot();
                }
                break;

                 */

        }
    }

    public void shoot() {

        switch (shootBall) {
            case 0:
                follower.pausePathFollowing();
                if (Math.abs(launcher.getVelocity() - 1500) < 20) {
                    trigger.setPosition(1);
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                follower.resumePathFollowing();
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    trigger.setPosition(0);
                    agitator.setPosition(1);
                    actionTimer.resetTimer();
                    setShootBall(2);
                }
                break;
            case 2:
                agitator.setPosition(0);
                setShootBall(3);
                break;
        }

    }

    public void setShootBall(int sBall) {
        shootBall = sBall;
        shootTimer.resetTimer();
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        setShootBall(0);
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        if (follower.getCurrentPathChain() == score1 || follower.getCurrentPathChain() == score2 || follower.getCurrentPathChain() == score3 || shootBall == 0) {
            launcher.setVelocity(1500);
        } else {
            launcher.set(0);
        }
        intake.set(1);

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        if (follower.isBusy()) {
            telemetry.addLine("follower is busy");
        } else {
            telemetry.addLine("follower is finished");
        }

        telemetry.addData("actual launcher velocity", launcher.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("shoot ball", shootBall);

        telemetry.setAutoClear(true);

        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        light = hardwareMap.get(Servo.class, "light");
        agitator = hardwareMap.get(Servo.class, "agitator");

        shootBall = 0;

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setInverted(true);

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        //ensures that all servos start at the correct position
        trigger.setPosition(0);
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

        pitch.set(0);
        trigger.setPosition(0);
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
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}