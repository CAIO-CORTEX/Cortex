package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CortexCaioTeleop", group="Linear opMode")

public class CortexCaioTeleop extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLO = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;
    private DcMotor AF0 = null;
    private DcMotor AU1 = null;

    @Override
    public void runOpMode() {
        FLO = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");
        AF0 = hardwareMap.get(DcMotor.class, "AF0");
        AU1 = hardwareMap.get(DcMotor.class, "AU1");

        FLO.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.REVERSE);
        BR3.setDirection(DcMotor.Direction.FORWARD);
        AF0.setDirection(DcMotor.Direction.FORWARD);
        AU1.setDirection(DcMotor.Direction.FORWARD);

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

            if (gamepad1.right_bumper) {
                AF0.setPower(1);
                sleep(100);
                AF0.setPower(0);
            }
            if (gamepad1.left_bumper) {
                AF0.setPower(-1);
                sleep(100);
                AF0.setPower(0);
            }
            if (gamepad1.x) {
                AU1.setPower(1);
                sleep(100);
                AU1.setPower(0);
            }
            if (gamepad1.a){
                AU1.setPower(-1);
                sleep(100);
                AU1.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }

}
