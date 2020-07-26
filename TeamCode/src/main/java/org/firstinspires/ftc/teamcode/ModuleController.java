package org.firstinspires.ftc.teamcode;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.lib.*;
import org.firstinspires.ftc.teamcode.lib.Calculate.*;




public class ModuleController{

    public ModuleState state;
    public ModuleState modifiedTargetState;
    public boolean reversed;

    public ModulePowers modulePowers;

    public Vector2D odometer = new Vector2D();


    final double WHEEL_RADIUS = 0.03175;
    final double TICKS_PER_REV = 103.6;
    final double RINGS_GEAR_RATIO = 9.88;
    final double WHEEL_GEAR_RATIO = 0.168;



    private PIDF anglePIDF = new PIDF(0.5, 0, 0, 0, 0, 0);
    private PIDF forwardPIDF = new PIDF(0.01, 0, 0, 0, 0, 0);

    private SimpleMatrix K = new SimpleMatrix(new double[][] { //from matlab calcs
        { 22.36,    0,    0,    8.06},
        { 22.36,    0,    0,    -8.06}
    });

    public ModuleController(ModuleState initialState){
        state = initialState;
    }

    public ModulePowers move(ModuleState targetState){
        modifiedTargetState = targetState.copy(); //modify to find optimal angle with same results
        modifiedTargetState.moduleAngle = calcClosestModuleAngle(state.moduleAngle, targetState.moduleAngle);
        if(reversed) modifiedTargetState.wheelAngVelo = -targetState.wheelAngVelo;
        
        double rotPower = anglePIDF.loop(state.moduleAngle, modifiedTargetState.moduleAngle);
        if(anglePIDF.inTolerance()) rotPower = 0;
        double forwardPower = forwardPIDF.loop(state.wheelAngVelo, modifiedTargetState.wheelAngVelo);

        double maxPower = Math.abs(rotPower) + Math.abs(forwardPower);

        double topPower = (rotPower + forwardPower) / maxPower;
        double bottomPower = (rotPower - forwardPower) / maxPower;
        modulePowers = new ModulePowers(topPower, bottomPower);
        return modulePowers;
    }

    public double rotateModule(double targetModuleAngle){
        double closestModuleAngle = calcClosestModuleAngle(state.moduleAngle, targetModuleAngle);
        double rotPower = anglePIDF.loop(state.moduleAngle, closestModuleAngle);
        return rotPower;
    }

    public double calcClosestModuleAngle(double currentAngle, double targetAngle){
        double differencePi = (currentAngle - targetAngle) % Math.PI; //angle error from (-180, 180)

        double closestAngle;
        if(Math.abs(differencePi) < (Math.PI / 2.0)){ //chooses closer of the two acceptable angles closest to currentAngle
            closestAngle = currentAngle - differencePi;
        }else{
            closestAngle = currentAngle - differencePi + Math.copySign(Math.PI, differencePi);
        }

        double difference2Pi = (closestAngle - targetAngle) % (2 * Math.PI);
        reversed = Math.abs(difference2Pi) > (Math.PI / 2.0); //if the difference is closer to 180, reverse direction 

        return closestAngle;
    }

    double lastTime = System.nanoTime();
    public void updateState(double topEncoderPos, double bottomEncoderPos, double topEncoderVelo, double bottomEncoderVelo){
        state.moduleAngle = calcModuleAngle(topEncoderPos, bottomEncoderPos);
        state.moduleAngVelo = calcModuleAngle(topEncoderVelo, bottomEncoderVelo);
        state.wheelAngVelo = calcWheelAngle(topEncoderVelo, bottomEncoderVelo);
        state.wheelAngle = calcWheelAngle(topEncoderPos, bottomEncoderPos);

        double dt = (System.nanoTime() - lastTime) * 1e-9;
        lastTime = System.nanoTime();
        Vector2D ds = new Vector2D(state.wheelAngVelo * WHEEL_RADIUS, state.moduleAngle, Vector2D.Type.POLAR).scalarMult(dt);
        odometer = odometer.add(ds);
    }


    private double calcWheelAngle(double motorTop, double motorBottom){
        double avgMotorTicks = (motorTop - motorBottom) / 2.0; //wheel rotation is difference in motors / 2
        double avgMotorRevs = avgMotorTicks / TICKS_PER_REV; //convert encoder ticks to revolutions
        double wheelAngleRevs = avgMotorRevs / (RINGS_GEAR_RATIO * WHEEL_GEAR_RATIO); //wheel angle, in revolutions
        double wheelAngleRad = wheelAngleRevs * 2 * Math.PI; //convert revolutions to radians
        return wheelAngleRad;
    }
    private double calcModuleAngle(double motorTop, double motorBottom){
        double avgMotorTicks = (motorTop + motorBottom) / 2.0; //module rotation is the average of the motors
        double avgMotorRevs = avgMotorTicks / TICKS_PER_REV; //convert encoder ticks to revolutions
        double moduleAngleRevs = avgMotorRevs / RINGS_GEAR_RATIO; //module angle, in revolutions
        double moduleAngleRad = moduleAngleRevs * 2 * Math.PI; //convert revolutions to radians
        return moduleAngleRad;
    }


    public static class ModulePowers{
        public double topPower;
        public double bottomPower;
        public ModulePowers(double topPower, double bottomPower){
            this.topPower = topPower;
            this.bottomPower = bottomPower;
        }
    }

    public static class ModuleState{
        public double moduleAngle;
        public double moduleAngVelo;
        public double wheelAngle;
        public double wheelAngVelo;

        public ModuleState(double moduleAngle, double moduleAngVelo, double wheelAngle, double wheelAngVelo){
            this.moduleAngle = moduleAngle;
            this.moduleAngVelo = moduleAngVelo;
            this.wheelAngle = wheelAngVelo;
            this.wheelAngVelo = wheelAngVelo;
        }

        public ModuleState(){
            this.moduleAngle = 0;
            this.moduleAngVelo = 0;
            this.wheelAngle = 0;
            this.wheelAngVelo = 0;
        }

        public SimpleMatrix getState(){
            return new SimpleMatrix(new double[][]{
                {moduleAngle},
                {moduleAngVelo},
                {wheelAngle},
                {wheelAngVelo}
            });
        }

        public ModuleState copy(){
            return new ModuleState(moduleAngle, moduleAngVelo, wheelAngle, wheelAngVelo);
        }

    }
}