package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.ejml.simple.SimpleMatrix;


@TeleOp(name = "EJML", group = "test")

public class EJMLVelo extends OpMode {

    DcMotor leftTop = null;
    DcMotor leftBottom = null;
    DcMotor rightTop = null;
    DcMotor rightBottom = null;

    DcMotor rightEncoder = null;

    double gearRatio = 1024;




    @Override
    public void init() {
        leftTop = hardwareMap.get(DcMotor.class, "2-0");
        leftTop.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBottom = hardwareMap.get(DcMotor.class, "2-1");
        leftBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        rightTop = hardwareMap.get(DcMotor.class, "2-2");
        rightTop.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBottom = hardwareMap.get(DcMotor.class, "2-3");
        rightBottom.setDirection(DcMotorSimple.Direction.FORWARD);


        rightEncoder = hardwareMap.get(DcMotor.class, "1-0");


    }

    @Override
    public void loop() {
        double forward = -gamepad1.right_stick_y;
        double moduleRot = gamepad1.right_stick_x;

        SimpleMatrix wheelMatrix = new SimpleMatrix(new double[][] { { forward }, { moduleRot } });
        SimpleMatrix diffMatrix = new SimpleMatrix(new double[][] { { 0.5 , -0.5 }, { 0.5, 0.5 } });

        SimpleMatrix ringsMatrix = diffMatrix.solve(wheelMatrix);

        double top = ringsMatrix.get(0, 0);
        double bottom = ringsMatrix.get(1, 0);

        runMotorsAtVelo(top, bottom, top, bottom);



        telemetry.addData( "forward", forward);
        telemetry.addData( "moduleRot", moduleRot);
        telemetry.addData( "RMotorAngle", determineAngle(rightTop.getCurrentPosition(), rightBottom.getCurrentPosition()));
        telemetry.addData( "LMotorAngle", determineAngle(leftTop.getCurrentPosition(), leftBottom.getCurrentPosition()));
        telemetry.update();
    }

    private double determineAngle(double motorTop, double motorBottom){
        double avgMotor = ((motorTop + motorBottom)/2.0);
        return (avgMotor/gearRatio) * 360.0;
    }

    private void runMotorsAtVelo(double LT, double LB, double RT, double RB){
        leftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTop.setPower(LT);
        leftBottom.setPower(LB);
        rightTop.setPower(RT);
        rightBottom.setPower(RB);
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }



}
