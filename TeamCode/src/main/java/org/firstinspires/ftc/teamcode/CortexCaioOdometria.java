package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="CortexCaioOdometria")

public class CortexCaioOdometria extends LinearOpMode {

    //MOTORES DE MOVIMENTO
    DcMotor FL0, FR1, BL2, BR3;

    //ENCODERS DA ODOMETRIA
    DcMotor LO1, CO2, RO3;

    //POSIÇÃO ATUAL DO ROBÔ
    double x = 0, y = 0, angulo = 0;

    //leituras anteriores dos encoders
    int prevLo = 0, prevRo = 0, prevCo = 0;

    //constantes do robô em cm
    final double COUNTS_PER_CM = 17.14;
    final double ROBOT_WIDTH_CM = 14.80;
    final double HORIZONTAL_OFFSET_CM = 17.50;

    @Override
    public void runOpMode() {
        //mapeamento de hardware(como aperece na drive)
        FL0 = hardwareMap.dcMotor.get("FL0");
        FR1 = hardwareMap.dcMotor.get("FR1");
        BL2 = hardwareMap.dcMotor.get("BL2");
        BR3 = hardwareMap.dcMotor.get("BR3");
        LO1 = hardwareMap.dcMotor.get("LO1");
        CO2 = hardwareMap.dcMotor.get("CO2");
        RO3 = hardwareMap.dcMotor.get("RO3");

        //definição da direção dos motores
        FL0.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.REVERSE);

        //resetdos encoders
        resetEncoders();

        waitForStart();

        while (opModeIsActive()) {
            updateOdometry();

            drive(10,10,10);
            moveToPosition(10, 0, 1);


        }

        telemetry.addData("final position", "X: %.2f Y: %.2f Ângulo: %.2f", x, y, Math.toDegrees(angulo));
        telemetry.update();
    }

    private void resetEncoders() {
        LO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CO2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RO3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LO1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CO2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RO3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateOdometry() {
        int Lo = LO1.getCurrentPosition();
        int Ro = RO3.getCurrentPosition();
        int Co = CO2.getCurrentPosition();

        int Lvo = Lo - prevLo;
        int Rvo = Ro - prevRo;
        int Ho = Co - prevCo;

        prevLo = Lo;
        prevRo = Ro;
        prevCo = Ho;

        double deltaAngulo = (Rvo - Lvo) / (COUNTS_PER_CM * ROBOT_WIDTH_CM);

        double forward = (Lvo + Rvo) / 2.0 / COUNTS_PER_CM;
        double strafe = (Co - (deltaAngulo * HORIZONTAL_OFFSET_CM)) / COUNTS_PER_CM;

        double oldAngle = angulo;
        angulo += deltaAngulo;

        x += forward * Math.cos(oldAngle) - strafe * Math.sin(oldAngle);
        y += forward * Math.cos(oldAngle) + strafe * Math.sin(oldAngle);
    }

    private void moveToPosition(double targetX, double targetY, double power) {
        while (opModeIsActive()) {

            double errorX = targetX - x;
            double errorY = targetY - y;
            double distance = Math.hypot(errorX, errorY);

            //mostra a posição continuamente
            telemetry.addData("Posição atual", "X: %.2f Y: %.2f Ângulo: %.2f", x, y, Math.toDegrees(angulo));
            telemetry.addData("Distancia até alvo", "%.2rf", distance);
            telemetry.update();

            if (distance < 2) break;

            double angle = Math.atan2(errorY, errorX);

            double driveX = Math.cos(angle) * power;
            double driveY = Math.sin(angle) * power;

            drive(driveY, driveX, 0);
        }

        Stop();

    }

    private void drive(double forward, double strafe, double rotate) {

        double FL = forward + strafe + rotate;
        double FR = forward - strafe - rotate;
        double BL = forward - strafe + rotate;
        double BR = forward + strafe - rotate;

        //normalizar as potencias
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

    private void Stop() {
        FL0.setPower(0);
        FR1.setPower(0);
        BL2.setPower(0);
        BR3.setPower(0);
    }
}
