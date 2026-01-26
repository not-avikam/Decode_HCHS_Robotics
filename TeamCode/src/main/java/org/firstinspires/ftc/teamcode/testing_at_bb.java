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
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


@Configurable
@TeleOp(name = "bb test", group = "Testing")
public class testing_at_bb extends OpMode {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    public static double velocity = 0;
    public static double velocity_servo = 0;
    public static double velocity_intake = 0;
    public static double agigtator_position = 0;
    public static double indexer_position = 0;
    public static double pitch_position = 0;
    private ServoEx indexer = null;

    private DcMotorEx motor = null;
    //private Servo servo = null;
    private CRServoEx crservo1 = null;
    private CRServoEx crservo2 = null;
    private DcMotor intake = null;
    private Servo agigtator = null;
    private ServoEx pitch = null;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        //servo = hardwareMap.get(Servo.class, "servo");
        crservo1 = new CRServoEx(hardwareMap, "yaw1");
        crservo2 = new CRServoEx(hardwareMap, "yaw2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        //keep it between 400 and 0 for pitch
        //400 corresponds to 145 degrees, and zero corresponds to 105

        crservo1.setPIDF(new PIDFCoefficients(kp, ki, kd, kf));
        crservo2.setPIDF(new PIDFCoefficients(kp, ki, kd, kf));

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));
        motor.setVelocity(velocity);
        crservo1.set(velocity_servo);
        crservo2.set(-velocity_servo);
        intake.setPower(velocity_intake);
        agigtator.setPosition(agigtator_position);
        indexer.set(indexer_position);
        pitch.set(pitch_position);


        /*
        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(0);
        } else if (gamepad1.dpadDownWasPressed()){
            servo.setPosition(1);
        }

         */

        panelsTelemetry.addData("motor velocity", motor.getVelocity());
        panelsTelemetry.update(telemetry);

        //return 0;
    }
}