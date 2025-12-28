package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


@Configurable
@TeleOp(name = "launcher_velocity_calculator", group = "Testing")
public class launcher_velocity_calculator extends OpMode {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    public static double velocity = 0;
    public static double velocity_servo = 0;
    public static double velocity_intake = 0;
    public static double indexer_position = 0;
    public static double pitch_position = 0;
    private ServoEx indexer = null;

    private MotorEx launcher = null;
    //private Servo servo = null;
    private CRServoEx crservo1 = null;
    private CRServoEx crservo2 = null;
    private DcMotor intake = null;
    private Servo agigtator = null;
    private ServoEx pitch = null;
    double[] INTAKE_POS = {0, 130, 270};
    double[] SHOOT_POS = {170, 240, 300};
    double velocityIPS;
    public static double distanceToTarget = 0;
    int i = 0;
    int s = 0;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        //servo = hardwareMap.get(Servo.class, "servo");
        crservo1 = new CRServoEx(hardwareMap, "yaw1");
        crservo2 = new CRServoEx(hardwareMap, "yaw2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        //keep it between 400 and 0 for pitch
        //400 corresponds to 145 degrees, and zero corresponds to 105

        launcher.setInverted(true);
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        launcher.setVeloCoefficients(.03, .8, .05);

        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {

        velocityIPS = Math.sqrt(
                (386.4 * distanceToTarget * distanceToTarget) /
                        (2 * Math.pow(Math.cos(50), 2)
                                * (distanceToTarget * Math.tan(50) - (39)))
        );

        double velocityTPS = Math.log(velocityIPS / 69.9) / 0.000821;

        if (gamepad1.left_trigger !=0) {
            launcher.setVelocity(velocityIPS);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_trigger !=0) {
            agigtator.setPosition(0);
        } else {
            agigtator.setPosition(.3);
        }

        crservo1.set(velocity_servo);
        crservo2.set(-velocity_servo);
        intake.setPower(velocity_intake);
        pitch.set(pitch_position);

        if (gamepad1.dpadRightWasPressed()) {
            if (i < INTAKE_POS.length - 1) {
                i++;
            }
            indexer.set(INTAKE_POS[i]);
        } else if (gamepad1.dpadLeftWasPressed()) {
            if (i > 0) {
                i--;
            }
            indexer.set(INTAKE_POS[i]);
        } else if (gamepad1.dpadUpWasPressed()) {
            if (s < SHOOT_POS.length - 1) {
                s++;
            }
            indexer.set(SHOOT_POS[s]);
        } else if (gamepad1.dpadDownWasPressed()) {
            if (s > 0) {
                s--;
            }
            indexer.set(SHOOT_POS[s]);
        }

        /*
        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(0);
        } else if (gamepad1.dpadDownWasPressed()){
            servo.setPosition(1);
        }

         */
        
        panelsTelemetry.addData("launcher velocity", launcher.getVelocity());
        panelsTelemetry.addData("target velocity TPS", velocityTPS);
        panelsTelemetry.addData("target velocity IPS", velocityIPS);
        panelsTelemetry.update(telemetry);

        telemetry.addData("launcher velocity", launcher.getVelocity());
        telemetry.addData("target velocity TPS", velocityTPS);
        telemetry.addData("target velocity IPS", velocityIPS);
        telemetry.update();

    }
}
