package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SwerveTest2", group = "test")

public class SwerveTest2 extends OpMode {

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
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;
        double forward = -gamepad1.left_stick_y;

        double angle = Math.atan2(y, x);

        if(Math.sqrt(x*x + y*y) > 0.9){
            rotateModule(Math.toDegrees(angle));
        }else{
            runMotorsForward(forward);
        }

        telemetry.addData( "angle", angle);
        telemetry.addData( "RMotorAngle", determineAngle(rightTop.getCurrentPosition(), rightBottom.getCurrentPosition()));
        telemetry.addData( "LMotorAngle", determineAngle(leftTop.getCurrentPosition(), leftBottom.getCurrentPosition()));
        telemetry.update();
    }

    double determineAngle(double motorTop, double motorBottom){
        double avgMotor = ((motorTop + motorBottom)/2.0);
        double angleValue = (avgMotor/gearRatio) * 360.0;
        return angleValue;
    }

    void rotateModule(double angle){
        int encoderValue = (int) ((angle / 360.0) * gearRatio);
        runMotorsToPosition(encoderValue, encoderValue, encoderValue, encoderValue);
    }

    void runMotorsToPosition(int LT, int LB, int RT, int RB){
        leftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftTop.setTargetPosition(LT);
        leftBottom.setTargetPosition(LB);
        rightTop.setTargetPosition(RT);
        rightBottom.setTargetPosition(RB);

        leftTop.setPower(0.5);
        leftBottom.setPower(0.5);
        rightTop.setPower(0.5);
        rightBottom.setPower(0.5);
    }

    void runMotorsForward(double power){
        leftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftTop.setPower(power);
        leftBottom.setPower(-power);
        rightTop.setPower(power);
        rightBottom.setPower(-power);
    }



}
