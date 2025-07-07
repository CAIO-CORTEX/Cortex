package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="CortexCaioOdometriaComMotores")
public class CortexCaioOdometriaComMotores extends LinearOpMode {
    //motores de movimento
    DcMotor FL0, FR1, BL2, BR3;
    //encoders de odometria
    DcMotor LO1, CO2, RO3;
    double x = 0, y = 0, angulo = 0;
    int prevLo = 0, prevRo = 0, prevCo = 0;
    final double COUNTS_PER_CM = 543.17;
    final double ROBOT_WIDTH_CM = 42.20;
    final double HORIZONTAL_OFFSET_CM = 18.0;

    @Override
    public void runOpMode() {
        FL0 = hardwareMap.dcMotor.get("FL0");
        FR1 = hardwareMap.dcMotor.get("FR1");
        BL2 = hardwareMap.dcMotor.get("BL2");
        BR3 = hardwareMap.dcMotor.get("BR3");
        LO1 = hardwareMap.dcMotor.get("LO1");
        CO2 = hardwareMap.dcMotor.get("CO2");
        RO3 = hardwareMap.dcMotor.get("RO3");

        FL0.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.REVERSE);

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

            //eixo X: 5 = direita, -5 = esquerda. eixo Y: 5 = para frente, -5 = para trás
            movemotors(5,0,1);
            sleep(200);
            sleep(400);
            movemotors(0,5,1);
            sleep(200);
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
        LO1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CO2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RO3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void movemotors(double targetX, double targetY, double power) {
        while (opModeIsActive()) {
            updateOdometry();

            double errorX = targetX - x;
            double errorY = targetY - y;
            double distance = Math.hypot(errorX, errorY);

            // Mostra posição continuamente
            telemetry.addData("Posição Atual", "X: %.2f Y: %.2f Ângulo: %.2f", x, y, Math.toDegrees(angulo));
            telemetry.addData("Distância até alvo", "%.2f", distance);
            telemetry.update();

            if (distance < 2) break;

            double angle = Math.atan2(errorY, errorX);
            double driveX = Math.cos(angle) * power;
            double driveY = Math.sin(angle) * power;

            drive(driveY, driveX, 0);
        }

        stopmotors();
    }

    private void drive(double forward, double strafe, double rotate) {

        double FL = forward + strafe + rotate;
        double FR = forward - strafe - rotate;
        double BL = forward - strafe + rotate;
        double BR = forward + strafe - rotate;

        // Normalizar as potências
        double max = Math.max(Math.abs(FL), Math.max(Math.abs(FR), Math.max(Math.abs(BL), Math.abs(BR))));
        if (max > 1.0) {
            FL /= max;
            FR /= max;
            BL /= max;
            BR /= max;
        }

        FL0.setPower(FL);
        FR1.setPower(FR);
        BL2.setPower(BL);
        BR3.setPower(BR);
    }

    private void stopmotors() {
        FL0.setPower(0);
        FR1.setPower(0);
        BL2.setPower(0);
        BR3.setPower(0);
    }

}
