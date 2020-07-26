package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.lib.Calculate.Vector2D;
import org.firstinspires.ftc.teamcode.ModuleController.ModuleState;



public class RobotController {

    public RobotState robotState;

    public ModuleController leftController = new ModuleController(new ModuleState());
    public ModuleController rightController = new ModuleController(new ModuleState());

    public ModuleState leftTargetModuleState;
    public ModuleState rightTargetModuleState;

    final double HALF_DIST_BETWEEN_WHEELS = 0.15875;

    public RobotController(RobotState robotState){
        this.robotState = robotState;
    }

    public void move(RobotState targetRobotState){
        RobotState modifiedTargetState = targetRobotState.copy();
        modifiedTargetState.heading = calcClosestHeading(robotState.heading, targetRobotState.heading);

        double angVelo = 0*(modifiedTargetState.heading - robotState.heading);
        
        Vector2D targetLeftVelo = new Vector2D(
            modifiedTargetState.linVelo.x - angVelo * HALF_DIST_BETWEEN_WHEELS,
            modifiedTargetState.linVelo.y, 
            Vector2D.Type.CARTESIAN);

        Vector2D targetRightVelo = new Vector2D(
            modifiedTargetState.linVelo.x + angVelo * HALF_DIST_BETWEEN_WHEELS,
            modifiedTargetState.linVelo.y,
            Vector2D.Type.CARTESIAN);

        leftTargetModuleState = new ModuleState(targetLeftVelo.getAngle(), 0, 0, targetLeftVelo.getMagnitude());
        rightTargetModuleState = new ModuleState(targetRightVelo.getAngle(), 0, 0, targetRightVelo.getMagnitude());

        leftController.move(leftTargetModuleState);
        rightController.move(rightTargetModuleState);
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
        double robotHeading,     
        double leftTopEncoderPosition,
        double leftBottomEncoderPosition,
        double leftTopEncoderVelocity,
        double leftBottomEncoderVelocity,
        double rightTopEncoderPosition,
        double rightBottomEncoderPosition,
        double rightTopEncoderVelocity,
        double rightBottomEncoderVelocity
    ){
        robotState.heading = robotHeading;
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
        public double heading;
        public RobotState(Vector2D linVelo, double heading){
            this.linVelo = linVelo;
            this.heading = heading;
        }
        public RobotState(){
            this.linVelo = new Vector2D();
            this.heading = 0;
        }
        public RobotState copy(){
            return new RobotState(linVelo, heading);
        }
    }

    









}