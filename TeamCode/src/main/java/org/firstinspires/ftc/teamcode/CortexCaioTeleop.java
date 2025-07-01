package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CortexCaioTeleop", group="Linear opMode")

public class CortexCaioTeleop extends LinearOpMode  {

    //MOVEMENT
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLO = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;


    //ODOMETRY
    private DcMotor LO1 = null;
    private DcMotor CO2 = null;
    private DcMotor RO3 = null;


    @Override
    public void runOpMode() {
        FLO = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");
        LO1 = hardwareMap.get(DcMotor.class, "LO1");
        CO2 = hardwareMap.get(DcMotor.class, "CO2");
        RO3 = hardwareMap.get(DcMotor.class, "RO3");

        FLO.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.REVERSE);

        LO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CO2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RO3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LO1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CO2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RO3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   =  -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double guinada =  gamepad1.right_stick_x;

            double leftFrontPower   = axial + lateral + guinada;
            double rightFrontPower  = axial - lateral - guinada;
            double leftBackPower    = axial - lateral + guinada;
            double rightBackPower   = axial + lateral - guinada;


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower   /= max;
                rightFrontPower  /= max;
                leftBackPower    /= max;
                rightBackPower   /= max;
            }

            FLO.setPower(leftFrontPower);
            FR1.setPower(rightFrontPower);
            BL2.setPower(leftBackPower);
            BR3.setPower(rightBackPower);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }

}
