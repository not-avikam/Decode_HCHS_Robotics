package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;


@Configurable
@TeleOp(name = "pid_tuning", group = "Testing")
public class PID_Tuners extends OpMode {
    public static double kp = 1;
    public static double ki = 1;
    public static double kd = 0;
    public static double kf = 0;
    public static double velocity = 0;
    public static double velocity_servo = 0;
    public static double distanceToTarget = 60; // inches
    public static double launchAngleDeg = 45;   // degrees
    double velocityIPS = 0;
    double velocityTPS = 0;
    private MotorEx launcher;
    //private Servo servo = null;

    private Servo agigtator;

    private CRServoEx crservo1 = null;
    private CRServoEx crservo2 = null;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {

        launcher = new MotorEx(hardwareMap, "launcher");
        //servo = hardwareMap.get(Servo.class, "servo");
        crservo1 = new CRServoEx(hardwareMap, "yaw1");
        crservo2 = new CRServoEx(hardwareMap, "yaw2");
        agigtator = hardwareMap.get(Servo.class, "agigtator");


        crservo1.setPIDF(new PIDFCoefficients(kp, ki, kd, kf));
        crservo2.setPIDF(new PIDFCoefficients(kp, ki, kd, kf));

        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        launcher.setVeloCoefficients(kp, ki, kd);
        launcher.setVelocity(velocity);
        crservo1.set(velocity_servo);
        crservo2.set(-velocity_servo);


        /*
        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(0);
        } else if (gamepad1.dpadDownWasPressed()){
            servo.setPosition(1);
        }

         */

        double theta = Math.toRadians(launchAngleDeg);

        // Physics denominator
        double denom =
                2 * Math.pow(Math.cos(theta), 2) *
                        (distanceToTarget * Math.tan(theta) - 39);

        // Compute required IPS safely
        if (denom > 0 && distanceToTarget > 0) {
            velocityIPS = Math.sqrt(
                    (386.4 * distanceToTarget * distanceToTarget) / denom
            );
        } else {
            velocityIPS = 0;
        }

        // IPS -> TPS (your regression, inverted)
        if (velocityIPS > 0) {
            velocityTPS = Math.log(velocityIPS / 69.9) / 0.000821;
        } else {
            velocityTPS = 0;
        }

        // Agitator
        if (gamepad1.right_trigger != 0) {
            agigtator.setPosition(0);
        } else {
            agigtator.setPosition(0.3);
        }


        // Telemetry
        panelsTelemetry.addData("Distance (in)", distanceToTarget);
        panelsTelemetry.addData("Velocity IPS", velocityIPS);
        panelsTelemetry.addData("Velocity TPS", velocityTPS);
        panelsTelemetry.addData("Launcher Actual TPS", launcher.getVelocity());
        panelsTelemetry.update(telemetry);

        telemetry.update();
        panelsTelemetry.update(telemetry);

        //return theta;
    }
}