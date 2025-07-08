package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="CortexCaioOdometria")
public class CortexCaioOdometria extends LinearOpMode {
    //encoders de odometria
    DcMotor RO1, CO2, LO3;
    // POSIÇÃO 0
    double x = 0, y = 0, angulo = 0;
    int prevLo = 0, prevRo = 0, prevCo = 0;
    final double COUNTS_PER_CM = 543.17;
    final double ROBOT_WIDTH_CM = 42.20;
    final double HORIZONTAL_OFFSET_CM = 18.0;

    @Override
    public void runOpMode() {
        RO1 = hardwareMap.dcMotor.get("LO1");
        CO2 = hardwareMap.dcMotor.get("CO2");
        LO3 = hardwareMap.dcMotor.get("RO3");

        resetEncoders();

        telemetry.addLine("Encoders prontos.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateOdometry();

            //POSIÇÃO ATUAL
            telemetry.addData("X (cm)", "%.2f", x);
            telemetry.addData("Y (cm)", "%.2f", y);
            telemetry.addData("Angulo (deg)", "%.2f", Math.toDegrees(angulo));
            telemetry.update();

            telemetry.addData("LO3 TICKS","%.2f",LO3.getCurrentPosition());
            telemetry.addData("CO2 TICKS","%.2f",CO2.getCurrentPosition());
            telemetry.addData("RO1 TICKS","%.2f",RO1.getCurrentPosition());

            idle();  // cede ciclos para o sistema, ajudando o telemetry :contentReference[oaicite:1]{index=1}
        }
    }

    public void resetEncoders() {
        RO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CO2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LO3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        RO1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CO2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LO3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void updateOdometry() {
        int Ro = RO1.getCurrentPosition();
        int Lo = LO3.getCurrentPosition();
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