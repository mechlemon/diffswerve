package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.*;




@TeleOp(name = "IMUtest", group = "IMUtest")

public class IMUtest extends OpMode {

    IMU imu;

    @Override
    public void init(){
        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();
    }

    @Override
    public void loop(){
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        telemetry.addData("yaw", imu.getHeading());
        imu.setHeadingAxis(IMU.HeadingAxis.PITCH);
        telemetry.addData("pitch", imu.getHeading());
        imu.setHeadingAxis(IMU.HeadingAxis.ROLL);
        telemetry.addData("roll", imu.getHeading());
        telemetry.update();


    }



}