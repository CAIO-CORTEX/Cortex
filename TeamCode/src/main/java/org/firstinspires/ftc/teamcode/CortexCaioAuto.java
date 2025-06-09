package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CortexCaioAuto", group="Linear opMode")

public class CortexCaioAuto extends LinearOpMode {

    private DcMotor FL0 = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;

    @Override
    public void runOpMode() {
        FL0 = hardwareMap.get(DcMotor.class,"FL0");
        FR1 = hardwareMap.get(DcMotor.class,"FR1");
        BL2 = hardwareMap.get(DcMotor.class,"BL2");
        BR3 = hardwareMap.get(DcMotor.class,"BR3");

        FL0.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            FL0.setPower(1);
            FR1.setPower(1);
            BL2.setPower(1);
            BR3.setPower(1);
            sleep(400);
            FL0.setPower(1);
            FR1.setPower(-1);
            BL2.setPower(1);
            BR3.setPower(-1);
            sleep(350);
            FL0.setPower(1);
            FR1.setPower(1);
            BL2.setPower(1);
            BR3.setPower(1);
            sleep(350);
            FL0.setPower(-1);
            FR1.setPower(1);
            BL2.setPower(-1);
            BR3.setPower(1);
            sleep(350);
            FL0.setPower(1);
            FR1.setPower(1);
            BL2.setPower(1);
            BR3.setPower(1);
            sleep(900);
            FL0.setPower(0);
            FR1.setPower(0);
            BL2.setPower(0);
            BR3.setPower(0);
            sleep(100000);
        }
    }
}
