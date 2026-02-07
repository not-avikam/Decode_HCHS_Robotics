package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
@Configurable
@TeleOp(name = "cont spindex test", group = "Testing")
public class ContinousIndexer extends OpMode {
    private CRServoEx indexer = null;
    private NormalizedColorSensor colorSensorIntake = null;
    private NormalizedColorSensor colorSensorShoot = null;

    // States for the color sequence for each sensor
    private enum IndexIntake {
        INTAKE_1, INTAKE_2, INTAKE_3
    }
    private enum IndexShoot {
        SHOOT_1, SHOOT_2, SHOOT_3
    }

    // State machine for motor control
    private enum MotionState {
        IDLE,
        SEARCHING_SHOOT_FORWARD,
        SEARCHING_SHOOT_BACKWARD,
        SEARCHING_INTAKE_FORWARD,
        SEARCHING_INTAKE_BACKWARD
    }

    private IndexIntake indexIntake = IndexIntake.INTAKE_1;
    private IndexShoot indexShoot = IndexShoot.SHOOT_1;
    private MotionState motionState = MotionState.IDLE;

    double hueShoot, hueIntake;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // TODO: Tune these hue value ranges for better accuracy
    private boolean isYellow(double hue) { return (hue > 60 && hue < 150); }
    private boolean isOrange(double hue) { return (hue > 30 && hue < 90); }
    private boolean isBlue(double hue) { return (hue > 150 && hue < 350); }


    @Override
    public void init() {
        indexer = new CRServoEx(hardwareMap, "indexer");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        colorSensorShoot = hardwareMap.get(NormalizedColorSensor.class, "sensor_shoot");
    }

    @Override
    public void loop() {
        // 1. Read sensor values at the start of every loop
        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA colorsShoot = colorSensorShoot.getNormalizedColors();
        hueIntake = JavaUtil.colorToHue(colorsIntake.toColor());
        hueShoot = JavaUtil.colorToHue(colorsShoot.toColor());

        // 2. Handle gamepad inputs to change the motion state
        // This section decides WHAT to do
        if (gamepad1.dpadUpWasPressed()) {
            motionState = MotionState.SEARCHING_SHOOT_FORWARD;
        } else if (gamepad1.dpadDownWasPressed()) {
            motionState = MotionState.SEARCHING_SHOOT_BACKWARD;
        } else if (gamepad1.dpadRightWasPressed()) {
            motionState = MotionState.SEARCHING_INTAKE_FORWARD;
        } else if (gamepad1.dpadLeftWasPressed()) {
            motionState = MotionState.SEARCHING_INTAKE_BACKWARD;
        } else if (gamepad1.bWasPressed()) { // Manual stop
            motionState = MotionState.IDLE;
        }

        // 3. Execute the logic based on the current motion state
        // This section figures out HOW to do it, and runs every loop cycle
        boolean colorFound = false;
        switch (motionState) {
            case IDLE:
                indexer.set(0);
                break;

            case SEARCHING_INTAKE_FORWARD:
                switch (indexIntake) {
                    case INTAKE_1: if (isYellow(hueShoot)) colorFound = true; break;
                    case INTAKE_2: if (isOrange(hueShoot)) colorFound = true; break;
                    case INTAKE_3: if (isBlue(hueShoot)) colorFound = true; break;
                }

                if (colorFound) {
                    indexer.set(0); // Stop motor immediately
                    motionState = MotionState.IDLE;
                    indexIntake = (indexIntake == IndexIntake.INTAKE_1) ? IndexIntake.INTAKE_2 :
                                  (indexIntake == IndexIntake.INTAKE_2) ? IndexIntake.INTAKE_3 :
                                  IndexIntake.INTAKE_1;
                } else {
                    indexer.set(1); // Run motor forward
                }
                break;

            case SEARCHING_INTAKE_BACKWARD:
                switch (indexIntake) {
                    case INTAKE_1: if (isBlue(hueShoot)) colorFound = true; break;
                    case INTAKE_2: if (isOrange(hueShoot)) colorFound = true; break;
                    case INTAKE_3: if (isYellow(hueShoot)) colorFound = true; break;
                }

                if (colorFound) {
                    indexer.set(0); // Stop motor immediately
                    motionState = MotionState.IDLE;
                    indexIntake = (indexIntake == IndexIntake.INTAKE_1) ? IndexIntake.INTAKE_2 :
                                  (indexIntake == IndexIntake.INTAKE_2) ? IndexIntake.INTAKE_3 :
                                  IndexIntake.INTAKE_1;
                } else {
                    indexer.set(-1); // Run motor backward
                }
                break;

            case SEARCHING_SHOOT_FORWARD:
                switch (indexShoot) {
                    case SHOOT_1: if (isYellow(hueIntake)) colorFound = true; break;
                    case SHOOT_2: if (isOrange(hueIntake)) colorFound = true; break;
                    case SHOOT_3: if (isBlue(hueIntake)) colorFound = true; break;
                }

                if (colorFound) {
                    indexer.set(0); // Stop motor immediately
                    motionState = MotionState.IDLE;
                    indexShoot = (indexShoot == IndexShoot.SHOOT_1) ? IndexShoot.SHOOT_2 :
                                 (indexShoot == IndexShoot.SHOOT_2) ? IndexShoot.SHOOT_3 :
                                 IndexShoot.SHOOT_1;
                } else {
                    indexer.set(1); // Run motor forward
                }
                break;

            case SEARCHING_SHOOT_BACKWARD:
                switch (indexShoot) {
                    case SHOOT_1: if (isBlue(hueIntake)) colorFound = true; break;
                    case SHOOT_2: if (isOrange(hueIntake)) colorFound = true; break;
                    case SHOOT_3: if (isYellow(hueIntake)) colorFound = true; break;
                }

                if (colorFound) {
                    indexer.set(0); // Stop motor immediately
                    motionState = MotionState.IDLE;
                    indexShoot = (indexShoot == IndexShoot.SHOOT_1) ? IndexShoot.SHOOT_2 :
                                 (indexShoot == IndexShoot.SHOOT_2) ? IndexShoot.SHOOT_3 :
                                 IndexShoot.SHOOT_1;
                } else {
                    indexer.set(-1); // Run motor backward
                }
                break;
        }

        // 4. Update telemetry
        telemetry.addData("Intake Hue", "%.2f", hueIntake);
        telemetry.addData("Shoot Hue", "%.2f", hueShoot);
        panelsTelemetry.addData("Motion State", motionState.name());
        panelsTelemetry.addData("Next Intake Color", indexIntake.name());
        panelsTelemetry.addData("Next Shoot Color", indexShoot.name());
        panelsTelemetry.update(telemetry);
        //return 0;
    }
}
