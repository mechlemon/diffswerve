package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "HoldAngle", group = "test")

public class HoldAngle extends OpMode {

    private DcMotor leftTop = null;
    private DcMotor leftBottom = null;
    private DcMotor rightTop = null;
    private DcMotor rightBottom = null;

    private DcMotor rightOdometryEncoder = null;

    private double gearRatio = 1024;

    private double targetAngle;

    private double leftModuleAngle, rightModuleAngle = 0;
    private Calculate.PIDF leftModuleAnglePID =  new Calculate.PIDF( 0.01, 0, 0,  0.1, 0, 0);
    private Calculate.PIDF rightModuleAnglePID =  new Calculate.PIDF( 0.01, 0, 0,  0.1, 0, 0);

    private String[] titles = {"kP", "kI", "kD", "kF", "tolerance", "velTolerance"};
    private double[] values = {0.01, 0, 0, 0, 0, 0};
    private Tuner tuner;

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

        rightOdometryEncoder = hardwareMap.get(DcMotor.class, "1-0");

        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    @Override
    public void loop() {

        tuner.tune();
        leftModuleAnglePID.setConstants(tuner.get("kP"), tuner.get("kI"), tuner.get("kD"), tuner.get("kF"), tuner.get("tolerance"), tuner.get("velTolerance"));
        rightModuleAnglePID.setConstants(tuner.get("kP"), tuner.get("kI"), tuner.get("kD"), tuner.get("kF"), tuner.get("tolerance"), tuner.get("velTolerance"));

        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;

        double forward = -gamepad1.left_stick_y;

        if(Math.sqrt(x*x + y*y) > 0.9){
            targetAngle = Math.toDegrees(Math.atan2(y, x)) - 90;
            rotateModules(targetAngle);
        }else{
            runMotorsAtVelo(forward, -forward, forward, -forward);
        }

        telemetry.addData( "targetAngle", targetAngle);
        telemetry.addData( "RMotorAngle", determineAngle(rightTop.getCurrentPosition(), rightBottom.getCurrentPosition()));
        telemetry.addData( "LMotorAngle", determineAngle(leftTop.getCurrentPosition(), leftBottom.getCurrentPosition()));
        telemetry.update();
    }

    private double determineAngle(double motorTop, double motorBottom){
        double avgMotor = ((motorTop + motorBottom)/2.0);
        double angleValue = (avgMotor/gearRatio) * 360.0;
        return angleValue;
    }

    private void rotateModules(double angle){
        leftModuleAngle = determineAngle(leftTop.getCurrentPosition(), leftBottom.getCurrentPosition());
        rightModuleAngle = determineAngle(rightTop.getCurrentPosition(), rightBottom.getCurrentPosition());
        leftModuleAnglePID.loop(leftModuleAngle, angle);
        rightModuleAnglePID.loop(rightModuleAngle, angle);

        double leftPower = leftModuleAnglePID.getPower();
        double rightPower = rightModuleAnglePID.getPower();
        runMotorsAtPower(leftPower, leftPower, rightPower, rightPower);
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

    private void runMotorsAtPower(double LT, double LB, double RT, double RB){
        leftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftTop.setPower(LT);
        leftBottom.setPower(LB);
        rightTop.setPower(RT);
        rightBottom.setPower(RB);
    }

    private void runMotorsForward(double power){
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
