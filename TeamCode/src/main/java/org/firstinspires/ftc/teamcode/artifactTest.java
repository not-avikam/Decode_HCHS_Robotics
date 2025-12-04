package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
@TeleOp(name="artifact test", group="Testing")
public class artifactTest extends LinearOpMode {


    // CORRECTED: Typo "agigtator" is now "agitator"
    private ServoEx agitator = null;
    private ServoEx indexer = null;
    private DcMotor intake = null;
    public static double pos;

    // RENAMED: Enum name changed to follow Java conventions

    /*
    private enum IndexIntake {
        INTAKE_1,
        INTAKE_2,
        INTAKE_3
    }

    private enum IndexShoot {
        SHOOT_1,
        SHOOT_2,
        SHOOT_3
    }

    private IndexIntake indexIntake = IndexIntake.INTAKE_1;
    private IndexShoot indexShoot = IndexShoot.SHOOT_1;

    public static double shootpos1 = 240;
    public static double shootpos2 = 170;
    public static double shootpos3 = 300;

     */

    @Override
    public void runOpMode() {
        // CORRECTED: Hardware map name to match variable
        agitator = new ServoEx(hardwareMap, "agigtator");
        indexer = new ServoEx(hardwareMap, "indexer", 0, 300);
        intake = hardwareMap.get(DcMotor.class, "intake");

        agitator.setInverted(true);
        intake.setDirection(DcMotor.Direction.REVERSE);

        indexer.set(0);
        agitator.set(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger != 0) {
                agitator.set(.2);
            } else {
                agitator.set(0);
            }

            // --- RESTRUCTURED: Indexer Control Logic ---
            // Using an if/else if structure prevents different buttons from overriding each other.

            // The left trigger now has priority for shooting.
            // The D-pad will only be checked if the left trigger is not being pressed.

            //TODO
            //170 corresponds to 260
            //300 corresponds to 0
            //240 corresponds to 130

            /*
            if (gamepad1.dpadRightWasPressed()) {
                switch (indexIntake) {
                    case INTAKE_1:
                        indexIntake = IndexIntake.INTAKE_2;
                        indexer.set(0);
                        break;
                    case INTAKE_2:
                        indexIntake = IndexIntake.INTAKE_3;
                        indexer.set(130);
                        break;
                    case INTAKE_3:
                        indexer.set(260);
                        indexIntake = IndexIntake.INTAKE_1;
                        break;
                }
            } else if (gamepad1.dpadLeftWasPressed()) {
                switch (indexIntake) {
                    case INTAKE_1:
                        indexIntake = IndexIntake.INTAKE_2;
                        indexer.set(260);
                        break;
                    case INTAKE_2:
                        indexIntake = IndexIntake.INTAKE_3;
                        indexer.set(130);
                        break;
                    case INTAKE_3:
                        indexer.set(0);
                        indexIntake = IndexIntake.INTAKE_1;
                        break;
                }
            }

            if (gamepad1.dpadUpWasPressed()) {
                switch (indexShoot) {
                    case SHOOT_1:
                        indexShoot = IndexShoot.SHOOT_2;
                        indexer.set(shootpos1);
                        break;
                    case SHOOT_2:
                        indexShoot = IndexShoot.SHOOT_3;
                        indexer.set(shootpos2);
                        break;
                    case SHOOT_3:
                        indexer.set(shootpos3);
                        indexShoot = IndexShoot.SHOOT_1;
                }
            } else if (gamepad1.dpadDownWasPressed()) {
                switch (indexShoot) {
                    case SHOOT_1:
                        indexShoot = IndexShoot.SHOOT_2;
                        indexer.set(shootpos3);
                        break;
                    case SHOOT_2:
                        indexShoot = IndexShoot.SHOOT_3;
                        indexer.set(shootpos2);
                        break;
                    case SHOOT_3:
                        indexShoot = IndexShoot.SHOOT_1;
                        indexer.set(shootpos1);
                        break;
                }
            }

             */

            indexer.set(pos);


            if (gamepad1.xWasPressed()) {
                intake.setPower(1);
            } else if (gamepad1.xWasReleased()) {
                intake.setPower(0);
            }

        }
    }

}
