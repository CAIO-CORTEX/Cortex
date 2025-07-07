package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="CortexCaioOdometria")
public class CortexCaioOdometria extends LinearOpMode {
    //encoders de odometria
    DcMotor LO1, CO2, RO3;
    double x = 0, y = 0, angulo = 0;
    int prevLo = 0, prevRo = 0, prevCo = 0;
    final double COUNTS_PER_CM = 543.17;
    final double ROBOT_WIDTH_CM = 42.20;
    final double HORIZONTAL_OFFSET_CM = 18.0;

    @Override
    public void runOpMode() {
        LO1 = hardwareMap.dcMotor.get("LO1");
        CO2 = hardwareMap.dcMotor.get("CO2");
        RO3 = hardwareMap.dcMotor.get("RO3");

        resetEncoders();

        telemetry.addLine("Encoders prontos.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateOdometry();

            telemetry.addData("X (cm)", "%.2f", x);
            telemetry.addData("Y (cm)", "%.2f", y);
            telemetry.addData("Angulo (deg)", "%.2f", Math.toDegrees(angulo));
            telemetry.update();

            idle();  // cede ciclos para o sistema, ajudando o telemetry :contentReference[oaicite:1]{index=1}
        }

        // Opcional: exibe posição final por alguns segundos
        telemetry.addData("Final X", "%.2f", x);
        telemetry.addData("Final Y", "%.2f", y);
        telemetry.addData("Final ang", "%.2f", Math.toDegrees(angulo));
        telemetry.update();
        sleep(3000);
    }

    public void resetEncoders() {
        LO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CO2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RO3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        LO1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CO2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RO3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateOdometry() {
        int Lo = LO1.getCurrentPosition();
        int Ro = RO3.getCurrentPosition();
        int Co = CO2.getCurrentPosition();

        int Lvo = Lo - prevLo;
        int Rvo = Ro - prevRo;
        int Cho = Co - prevCo;

        prevLo = Lo; prevRo = Ro; prevCo = Co;

        double deltaAng = (Rvo - Lvo) / (COUNTS_PER_CM * ROBOT_WIDTH_CM);
        double forward = (Lvo + Rvo) / 2.0 / COUNTS_PER_CM;
        double strafe = (Cho - deltaAng * HORIZONTAL_OFFSET_CM) / COUNTS_PER_CM;

        double oldAng = angulo;
        angulo += deltaAng;

        x += forward * Math.cos(oldAng) - strafe * Math.sin(oldAng);
        y += forward * Math.sin(oldAng) + strafe * Math.cos(oldAng);
    }
}