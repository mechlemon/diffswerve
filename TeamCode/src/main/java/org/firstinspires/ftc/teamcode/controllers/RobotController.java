package org.firstinspires.ftc.teamcode.controllers;


import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D;
import org.firstinspires.ftc.teamcode.controllers.ModuleController.ModuleState;
import org.firstinspires.ftc.teamcode.controllers.ModuleController.ModulePowers;


public class RobotController {

    public RobotState robotState;

    public ModuleController leftController = new ModuleController(new ModuleState(0, 0, 0, 0));
    public ModuleController rightController = new ModuleController(new ModuleState(0, 0, 0, 0));

    public ModuleState targetLeftModuleState = new ModuleState(0, 0, 0, 0);
    public ModuleState targetRightModuleState = new ModuleState(0, 0, 0, 0);

    public Vector2D turnCenter = new Vector2D();
    double targetLeftWheelSpeed, targetRightWheelSpeed;

    public RobotController(RobotState robotState){
        this.robotState = robotState;
    }

    public double deadband = 0.05;

    public RobotPowers move(RobotState targetRobotState){

        double targetLeftModuleAngle = 0;
        double targetRightModuleAngle = 0;

        double targetTangentialSpeed = targetRobotState.linVelo.getMagnitude(); //of the center of the robot
        if(Math.abs(targetRobotState.angVelo) < deadband && targetRobotState.linVelo.getMagnitude() < deadband){
            targetLeftModuleAngle = leftController.state.moduleAngle;
            targetRightModuleAngle = rightController.state.moduleAngle;
            targetLeftWheelSpeed = 0;
            targetRightWheelSpeed = 0;
            stop(true);
            robotState.mode = "stop";
        }else if(Math.abs(targetRobotState.angVelo) < deadband){  //going straight deadband
            stop(false);
            targetLeftModuleAngle = targetRobotState.linVelo.getAngle();
            targetRightModuleAngle = targetRobotState.linVelo.getAngle();

            targetLeftWheelSpeed = targetRobotState.linVelo.getMagnitude();
            targetRightWheelSpeed = targetRobotState.linVelo.getMagnitude();
            robotState.mode = "straight";
        }else if(targetRobotState.linVelo.getMagnitude() < deadband){
            stop(false);
            targetLeftModuleAngle = 0;
            targetRightModuleAngle = 0;

            targetLeftWheelSpeed = -targetRobotState.angVelo * Constants.HALF_DIST_BETWEEN_WHEELS;
            targetRightWheelSpeed = targetRobotState.angVelo * Constants.HALF_DIST_BETWEEN_WHEELS;
            robotState.mode = "spin";

        }else{
            stop(false);
            //Coords of center of rotation
            // turnCenter.x = targetRobotState.linVelo.y / Math.abs(targetRobotState.angVelo);
            // turnCenter.y = targetRobotState.linVelo.x / Math.abs(targetRobotState.angVelo);

            turnCenter = targetRobotState.linVelo.rotate(-Math.PI/2.0).scalarMult(1/-targetRobotState.angVelo);

            // Vector2D origin2Robot = Main.robot.position.subtract(new Vector2D(5, 5, Type.CARTESIAN));
            // Vector2D robot2Origin = origin2Robot.scalarMult(-1).rotate(-Main.robot.heading);

            // turnCenter.x = robot2Origin.x;
            // turnCenter.y = robot2Origin.y;

            // turnCenter.x += Main.robot.position.x * 0.01;
            // turnCenter.y += targetRobotState.linVelo.y * 0.01;

            // targetLeftModuleAngle = Math.atan2(turnCenter.y - Constants.HALF_DIST_BETWEEN_WHEELS, turnCenter.x) - Math.copySign(Math.PI/2.0, turnCenter.x);
            // targetRightModuleAngle = Math.atan2(turnCenter.y + Constants.HALF_DIST_BETWEEN_WHEELS, turnCenter.x) - Math.copySign(Math.PI/2.0, turnCenter.x);

            Vector2D centerToLeftWheel = new Vector2D(Constants.HALF_DIST_BETWEEN_WHEELS, Math.PI/2.0, Vector2D.Type.POLAR);
            Vector2D centerToRightWheel = new Vector2D(Constants.HALF_DIST_BETWEEN_WHEELS, -Math.PI/2.0, Vector2D.Type.POLAR);

            Vector2D leftWheelToTurnCenter = turnCenter.subtract(centerToLeftWheel);
            Vector2D rightWheelToTurnCenter = turnCenter.subtract(centerToRightWheel);

            targetLeftModuleAngle = leftWheelToTurnCenter.getAngle() - Math.copySign(Math.PI/2.0, targetRobotState.angVelo);
            targetRightModuleAngle = rightWheelToTurnCenter.getAngle() - Math.copySign(Math.PI/2.0, targetRobotState.angVelo);

            // System.out.println("left: " + Math.toDegrees(targetLeftModuleAngle));
            // System.out.println("right: " + Math.toDegrees(targetRightModuleAngle));


            targetLeftWheelSpeed = targetTangentialSpeed * leftWheelToTurnCenter.getMagnitude() / turnCenter.getMagnitude();
            targetRightWheelSpeed = targetTangentialSpeed * rightWheelToTurnCenter.getMagnitude() / turnCenter.getMagnitude();
            // targetLeftModuleAngle = 0;
            // targetRightModuleAngle = 0;
            robotState.mode = "combo";

        }




        targetLeftModuleState = new ModuleState(targetLeftModuleAngle, 0, 0, targetLeftWheelSpeed / Constants.WHEEL_RADIUS);
        targetRightModuleState = new ModuleState(targetRightModuleAngle, 0, 0, targetRightWheelSpeed / Constants.WHEEL_RADIUS);

        ModulePowers leftPowers = leftController.move(targetLeftModuleState);
        ModulePowers rightPowers = rightController.move(targetRightModuleState);

        return new RobotPowers(leftPowers, rightPowers);
    }

    public void stop(boolean stopOrNot){
        leftController.stop(stopOrNot);
        rightController.stop(stopOrNot);
    }


    public double calcClosestHeading(double currentHeading, double targetHeading){
        double difference2Pi = (currentHeading - targetHeading) % (2 * Math.PI); //angle error from (-180, 180)
        double closestHeading;
        if(Math.abs(difference2Pi) < (Math.PI)){ //chooses closer of the two acceptable angles closest to currentAngle
            closestHeading = currentHeading - difference2Pi;
        }else{
            closestHeading = currentHeading - difference2Pi + Math.copySign(2 * Math.PI, difference2Pi);
        }
        return closestHeading;
    }

    public void updateState(
            double angVelo,
            double leftTopEncoderPosition,
            double leftBottomEncoderPosition,
            double leftTopEncoderVelocity,
            double leftBottomEncoderVelocity,
            double rightTopEncoderPosition,
            double rightBottomEncoderPosition,
            double rightTopEncoderVelocity,
            double rightBottomEncoderVelocity
    ){
        robotState.angVelo = angVelo;
        leftController.updateState(
                leftTopEncoderPosition,
                leftBottomEncoderPosition,
                leftTopEncoderVelocity,
                leftBottomEncoderVelocity);
        rightController.updateState(
                rightTopEncoderPosition,
                rightBottomEncoderPosition,
                rightTopEncoderVelocity,
                rightBottomEncoderVelocity);
    }



    public static class RobotState{
        public Vector2D linVelo;
        public double angVelo;
        public String mode = "stop";
        public RobotState(Vector2D linVelo, double angVelo){
            this.linVelo = linVelo;
            this.angVelo = angVelo;
        }
        public RobotState(){
            this.linVelo = new Vector2D();
            this.angVelo = 0;
        }
        public RobotState copy(){
            return new RobotState(linVelo, angVelo);
        }
    }

    public static class RobotPowers{
        public double leftTopPower, leftBottomPower, rightTopPower, rightBottomPower;
        public RobotPowers(double leftTopPower, double leftBottomPower, double rightTopPower, double rightBottomPower){
            this.leftTopPower = leftTopPower;
            this.leftBottomPower = leftBottomPower;
            this.rightTopPower = rightTopPower;
            this.rightBottomPower = rightBottomPower;
        }
        public RobotPowers(ModulePowers leftPowers, ModulePowers rightPowers){
            this.leftTopPower = leftPowers.topPower;
            this.leftBottomPower = leftPowers.bottomPower;
            this.rightTopPower = rightPowers.topPower;
            this.rightBottomPower = rightPowers.bottomPower;
        }
    }











}