package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D.Type;


@TeleOp(name = "Teleop2", group = "teleop")

public class Teleop2 extends OpMode {

    DcMotorEx leftTop = null;
    DcMotorEx leftBottom = null;
    DcMotorEx rightTop = null;
    DcMotorEx rightBottom = null;

    Vector2D joystick;

    ModuleController leftController = new ModuleController(new ModuleController.ModuleState());
    ModuleController rightController = new ModuleController(new ModuleController.ModuleState());

    IMU imu;

    @Override
    public void init() {
        leftTop = hardwareMap.get(DcMotorEx.class, "1-0");
        leftTop.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBottom = hardwareMap.get(DcMotorEx.class, "1-1");
        leftBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        rightTop = hardwareMap.get(DcMotorEx.class, "1-2");
        rightTop.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBottom = hardwareMap.get(DcMotorEx.class, "1-3");
        rightBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();
    }

    @Override
    public void loop() {
        joystick = new Vector2D(gamepad1.right_stick_x, -gamepad1.right_stick_y, Type.CARTESIAN);

        leftController.updateState(
                leftTop.getCurrentPosition(),
                leftBottom.getCurrentPosition(),
                leftTop.getVelocity(),
                leftBottom.getVelocity()
        );

        rightController.updateState(
                rightTop.getCurrentPosition(),
                rightBottom.getCurrentPosition(),
                rightTop.getVelocity(),
                rightBottom.getVelocity()
        );

        double leftRot = leftController.rotateModule(gamepad1.left_stick_x * Math.PI);
        double rightRot = rightController.rotateModule(gamepad1.left_stick_x * Math.PI);

        leftTop.setPower(leftRot);
        leftBottom.setPower(leftRot);
        rightTop.setPower(rightRot);
        rightBottom.setPower(rightRot);

        telemetry.addData( "RMotorAngle", rightController.state.moduleAngle);
        telemetry.addData( "LMotorAngle", leftController.state.moduleAngle);
        telemetry.update();
    }

    private void setDrivePowersAndFeed(double LT, double LB, double RT, double RB, double feedforward){
        leftTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftTop.setPower(LT + Math.copySign(feedforward, LT));
        leftBottom.setPower(LB + Math.copySign(feedforward, LB));
        rightTop.setPower(RT + Math.copySign(feedforward, RT));
        rightBottom.setPower(RB + Math.copySign(feedforward, RB));
    }

    private void runMotorsAtVelo(double LT, double LB, double RT, double RB){
        leftTop.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightTop.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
