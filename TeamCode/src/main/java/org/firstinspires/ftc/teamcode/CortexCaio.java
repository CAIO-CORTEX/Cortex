package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import javax.microedition.khronos.opengles.GL10;

@TeleOp(name="CortexCaio", group="Linear opMode")

public class CortexCaio extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLO = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;

    @Override
    public void runOpMode() {
        FLO = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        FLO.setDirection(DcMotor.Direction.REVERSE);
        FR1.setDirection(DcMotor.Direction.FORWARD);
        BL2.setDirection(DcMotor.Direction.REVERSE);
        BR3.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;
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
