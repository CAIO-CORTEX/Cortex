package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CortexCaioAuto", group="Linear opMode")

public class CortexCaioAuto extends LinearOpMode {

    private DcMotor FL0 = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;

    static final double DRIVE_GEAR_REDUCTION = 20.0;

    static final double WHEEL_DIAMETER_INCHES = 3.78;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {
        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        FL0.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.REVERSE);

        FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "FL:%7d FR:%7d BL:%7d BR:%7d",
                FL0.getCurrentPosition(),
                FR1.getCurrentPosition(),
                BL2.getCurrentPosition(),
                BR3.getCurrentPosition());
        telemetry.update();

        waitForStart();
        //COMEÇAR AQUI
        encoderDrive(DRIVE_SPEED, 20,20,20,20,5.0);
        encoderDrive(DRIVE_SPEED,-20,-20,-20,-20,5.0);
        encoderDrive(TURN_SPEED,20,-20,20,-20,5.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    public void encoderDrive(double speed,
                             double FL0Inches,
                             double FR1Inches,
                             double BL2Inches,
                             double BR3Inches,
                             double timeoutS) {
        int newFL0Target;
        int newFR1Target;
        int newBL2Target;
        int newBR3Target;

        if (opModeIsActive()) {

            newFL0Target = FL0.getCurrentPosition() + (int)(FL0Inches * COUNTS_PER_INCH);
            newFR1Target = FR1.getCurrentPosition() + (int)(FR1Inches * COUNTS_PER_INCH);
            newBL2Target = BL2.getCurrentPosition() + (int)(BL2Inches * COUNTS_PER_INCH);
            newBR3Target = BR3.getCurrentPosition() + (int)(BR3Inches * COUNTS_PER_INCH);
            FL0.setTargetPosition(newFL0Target);
            FR1.setTargetPosition(newFR1Target);
            BL2.setTargetPosition(newBL2Target);
            BR3.setTargetPosition(newBR3Target);

            FL0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FL0.setPower(Math.abs(speed));
            FR1.setPower(Math.abs(speed));
            BL2.setPower(Math.abs(speed));
            BR3.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL0.isBusy() && FR1.isBusy() && BL2.isBusy() && BR3.isBusy())) {

                telemetry.addData("STATUS", "Run time" + runtime.toString());
                telemetry.addData("Running to", "FL:%7d FR:%7d BL:%7d BR:%7d", newFL0Target, newFR1Target, newBL2Target, newBR3Target);
                telemetry.addData("Currently at", " FL:%7d FR:%7d BL:%7d BR:%7d",
                                            FL0.getCurrentPosition(), FR1.getCurrentPosition(), BL2.getCurrentPosition(), BR3.getCurrentPosition());
                telemetry.update();

                FL0.setPower(0);
                FR1.setPower(0);
                BL2.setPower(0);
                BR3.setPower(0);

                FL0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                sleep(250);
            }

        }
    }
}