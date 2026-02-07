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
@Autonomous(name="Far Red", group="lcq")
public class Auton_Red_Far extends OpMode {
    private Motor turretEncoder;
    private VisionPortal visionPortal;
    private Servo light;
    ColorBlobLocatorProcessor colorLocator;
    private MotorEx launcher = null;
    private Motor intake = null;
    private Servo trigger = null;
    private ServoEx pitch = null;
    private CRServoEx yaw1 = null;
    NormalizedColorSensor colorSensor;
    private final Pose startPose = new Pose(87.562, 9.127, Math.toRadians(90));
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;
    private boolean visionPathActive = false;
    private int pathState;
    private int shootBall;
    private PathChain score1, pickup1, score2, pickup2, score3, pickup3;
    boolean visionAvailable = false;
    boolean visionActive = false;
    boolean tagDetected = false;
    private final Pose scorePose = new Pose(11.826689774696712, 136.3625744877852);
    public void buildPaths() {

        pickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(87.562, 9.127),
                                new Pose(88.172, 36.468),
                                new Pose(115.742, 34.124)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(115.742, 34.124),

                                new Pose(85.098, 20.667)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose))

                .build();

        pickup2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.098, 20.667),
                                new Pose(83.004, 65.383),
                                new Pose(116.338, 59.811)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(116.338, 59.811),
                                new Pose(154.000, 80.000),
                                new Pose(94.103, 52.793),
                                new Pose(95.503, 83.361)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose))

                .build();

        pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.503, 83.361),

                                new Pose(118.583, 83.994)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.583, 83.994),

                                new Pose(109.620, 94.685)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shoot();
                if (shootBall >= 6) {
                    follower.followPath(pickup1, false);
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
                    if (shootBall >= 6){
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
                    if (shootBall >= 6) {
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
                    if (shootBall >= 6) {
                        telemetry.addLine("Autonomous Routine Completed");
                        telemetry.addLine("Good Luck!");
                        setPathState(9);
                    }
                }
                break;
                /*
            case 9:
                Pose tunnel = new Pose(117.39341421143848, 45.483535528596185);
                Pose toScore = new Pose(91.4367417677643, 15.246100519930673);

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

    public void finalIntake() {
        if (!follower.isBusy()) {
            driveToClosestArtifact();
        }
    }

    public void shoot() {

        switch (shootBall) {
            case 0:
                follower.pausePathFollowing();
                if ((launcher.getVelocity()) >= 1500+20 || launcher.getVelocity() <= 1500-20) {
                    trigger.setPosition(1);
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                follower.resumePathFollowing();
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    trigger.setPosition(0);
                    actionTimer.resetTimer();
                    setShootBall(2);
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
        setShootBall(0);
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

        if (follower.getCurrentPathChain() == score1 || follower.getCurrentPathChain() == score2 || follower.getCurrentPathChain() == score3) {
            launcher.setVelocity(1500);
        } else {
            launcher.set(0);
        }

        intake.set(1);


        if (!follower.isBusy()) {
            visionPathActive = false;
        }

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
        light = hardwareMap.get(Servo.class, "light");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        yaw1 = new CRServoEx(hardwareMap, "yaw1");
        turretEncoder = new Motor(hardwareMap, "turretEncoder");

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
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);

        telemetry.update();
    }

    private ColorBlobLocatorProcessor.Blob getClosestArtifact() {
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Blob bestBlob = null;
        double maxRadius = 0;

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            double r = b.getCircle().getRadius();
            if (r > maxRadius) {
                maxRadius = r;
                bestBlob = b;
            }
        }
        return bestBlob;
    }

    private Pose getArtifactTargetPose(ColorBlobLocatorProcessor.Blob blob) {
        Circle c = blob.getCircle();

        double imageWidth = 640;
        double centerX = imageWidth / 2.0;

        double xErrorPixels = c.getX() - centerX;

        // Tunables
        double degreesPerPixel = 0.045;   // tune this
        double inchesPerRadius = 0.14;   // tune this

        double rawHeadingDeg = xErrorPixels * degreesPerPixel;
        rawHeadingDeg = Math.max(-15, Math.min(15, rawHeadingDeg));
        double headingOffset = Math.toRadians(rawHeadingDeg);

        double distance = Math.min(c.getRadius() * inchesPerRadius, 12);

        Pose robotPose = follower.getPose();

        double targetX = robotPose.getX() + distance * Math.cos(robotPose.getHeading() + headingOffset);
        double targetY = robotPose.getY() + distance * Math.sin(robotPose.getHeading() + headingOffset);

        return new Pose(
                targetX,
                targetY,
                robotPose.getHeading() + headingOffset
        );
    }

    public void driveToClosestArtifact() {
        if (visionPathActive) return;

        ColorBlobLocatorProcessor.Blob blob = getClosestArtifact();
        if (blob == null) {
            light.setPosition(.277);
            return;
        }

        Pose target = getArtifactTargetPose(blob);

        Path toArtifact = new Path(
                new BezierLine(follower.getPose(), target)
        );

        if (blob.getCircle().getRadius() < 10) return;

        follower.followPath(toArtifact, true);
        visionPathActive = true;
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}