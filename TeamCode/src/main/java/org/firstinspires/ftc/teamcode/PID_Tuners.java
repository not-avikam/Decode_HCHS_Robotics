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


@Configurable
@TeleOp(name = "test", group = "Testing")
public class PID_Tuners extends OpMode {
    public static double kp = 1;
    public static double ki = 1;
    public static double kd = 0;
    public static double kf = 0;
    public static double velocity = 0;
    private DcMotorEx motor = null;
    private Servo servo = null;
    private CRServo crservo = null;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        crservo = hardwareMap.get(CRServo.class, "crservo");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));
        motor.setVelocity(velocity);
        crservo.setPower(gamepad1.left_stick_x);

        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(0);
        } else if (gamepad1.dpadDownWasPressed()){
            servo.setPosition(1);
        }

        panelsTelemetry.addData("motor velocity", motor.getVelocity());

        panelsTelemetry.update(telemetry);

    }
}