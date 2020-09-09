package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.controllers.RobotController.RobotState;
import org.firstinspires.ftc.teamcode.controllers.RobotController;
import org.firstinspires.ftc.teamcode.controllers.RobotController.RobotPowers;
import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.Tuner;


@TeleOp(name = "UserCode", group = "teleop")

public class UserCode extends OpMode {

    DcMotorEx leftTop, leftBottom, rightTop, rightBottom;
    IMU imu;

    RobotController controller = new RobotController(new RobotState());
    RobotState targetRobotState;

    Vector2D joystick;

    Vector2D targetLinVelo = new Vector2D();
    double targetAngVelo = 0;

    Tuner tuner;
    String[] titles = new String[] {"scale"};
    double[] values = new double[] {0.5};

    @Override
    public void init(){
        leftTop = hardwareMap.get(DcMotorEx.class, "1-0");
        leftTop.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBottom = hardwareMap.get(DcMotorEx.class, "1-1");
        leftBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTop = hardwareMap.get(DcMotorEx.class, "1-2");
        rightTop.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBottom = hardwareMap.get(DcMotorEx.class, "1-3");
        rightBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();

        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }



    @Override
    public void loop(){
        joystick = new Vector2D(-gamepad1.left_stick_y, gamepad1.left_stick_x, Vector2D.Type.CARTESIAN);

        controller.updateState(
            imu.getAngVelo(),
            leftTop.getCurrentPosition(),
            leftBottom.getCurrentPosition(),
            leftTop.getVelocity(),
            leftBottom.getVelocity(),
            rightTop.getCurrentPosition(),
            rightBottom.getCurrentPosition(),
            rightTop.getVelocity(),
            rightBottom.getVelocity()
        );

        tuner.tune();

        joystick = joystick.scalarMult(tuner.get("scale")).rotate(Math.toRadians(imu.getHeading()));
//        joystick = joystick.scalarMult(tuner.get("scale"));

//        targetLinVelo = joystick.add(controller.robotState.linVelo);
        targetLinVelo = joystick;
        // targetAngVelo = -Controls.rawZ * 3;
        targetAngVelo = 0;

        targetRobotState = new RobotState(targetLinVelo, targetAngVelo);

        telemetry.addData("targetLinVelo", targetLinVelo);


        RobotPowers robotPowers = controller.move(targetRobotState);

        setDrivePowersAndFeed(
            robotPowers.leftTopPower,
            robotPowers.leftBottomPower,
            robotPowers.rightTopPower,
            robotPowers.rightBottomPower,
            0.0
        );

//        setDrivePowersAndFeed(
//                gamepad1.left_stick_x,
//                gamepad1.left_stick_y,
//                gamepad1.right_stick_x,
//                gamepad1.right_stick_y,
//                0.0
//        );

//        telemetry.addData("heading", imu.getHeading());

        telemetry.addData("leftAngle", controller.leftController.state.moduleAngle);
        telemetry.addData("rightAngle", controller.rightController.state.moduleAngle);
        telemetry.addData("left inv", controller.leftController.reversed);
        telemetry.addData("right inv", controller.rightController.reversed);
//
//        telemetry.addData("LT power", robotPowers.leftTopPower);
//        telemetry.addData("LB power", robotPowers.leftBottomPower);
//        telemetry.addData("RT power", robotPowers.rightTopPower);
//        telemetry.addData("RB power", robotPowers.rightBottomPower);

    }

    private void setDrivePowersAndFeed(double LT, double LB, double RT, double RB, double feedforward){
        leftTop.setPower(LT + Math.copySign(feedforward, LT));
        leftBottom.setPower(LB + Math.copySign(feedforward, LB));
        rightTop.setPower(RT + Math.copySign(feedforward, RT));
        rightBottom.setPower(RB + Math.copySign(feedforward, RB));

    }
}
