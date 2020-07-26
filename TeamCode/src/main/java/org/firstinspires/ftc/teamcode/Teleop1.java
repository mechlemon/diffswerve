package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D.Type;



@TeleOp(name = "Teleop1", group = "teleop")

public class Teleop1 extends OpMode {

    DcMotorEx leftTop = null;
    DcMotorEx leftBottom = null;
    DcMotorEx rightTop = null;
    DcMotorEx rightBottom = null;

    Vector2D joystick;

    RobotController controller = new RobotController(new RobotController.RobotState());
    RobotController.RobotState targetRobotState;

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

        controller.updateState(
                Math.toRadians(imu.getHeading()),
                leftTop.getCurrentPosition(),
                leftBottom.getCurrentPosition(),
                leftTop.getVelocity(),
                leftBottom.getVelocity(),
                rightTop.getCurrentPosition(),
                rightBottom.getCurrentPosition(),
                rightTop.getVelocity(),
                rightBottom.getVelocity()
        );


        joystick = joystick.scalarMult(5).rotate(-Math.toRadians(imu.getHeading()));

        targetRobotState = new RobotController.RobotState(new Vector2D(joystick.x, joystick.y, Type.CARTESIAN), -gamepad1.left_stick_x * Math.PI);

        controller.move(targetRobotState);

        if(joystick.getMagnitude() > 0.1){
            leftTop.setPower(controller.leftController.modulePowers.topPower);
            leftBottom.setPower(controller.leftController.modulePowers.bottomPower);
            rightTop.setPower(controller.rightController.modulePowers.topPower);
            rightBottom.setPower(controller.rightController.modulePowers.bottomPower);
        }else{
            leftTop.setPower(0);
            leftBottom.setPower(0);
            rightTop.setPower(0);
            rightBottom.setPower(0);
        }


        telemetry.addData( "heading", controller.robotState.heading);

        telemetry.addData( "RMotorAngle", controller.rightController.state.moduleAngle);
        telemetry.addData( "LMotorAngle", controller.leftController.state.moduleAngle);
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
