package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ThreadTest", group = "test")

public class ThreadTest extends OpMode {

    double time;
    ModuleThread leftModuleThread;
    ModuleThread rightModuleThread;

    int count = 0;

    @Override
    public void init() {
        leftModuleThread = new ModuleThread("leftModuleThread");
        rightModuleThread =  new ModuleThread("rightModuleThread");
        time = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        count++;

        telemetry.addData( "count left", rightModuleThread.moduleTasks.count);
        telemetry.addData( "count right", rightModuleThread.moduleTasks.count);
        telemetry.addData( "count main", count);

        telemetry.addData( "dt left", leftModuleThread.moduleTasks.dt);
        telemetry.addData( "dt right", rightModuleThread.moduleTasks.dt);
        telemetry.addData( "dt main", System.currentTimeMillis() - time);
        telemetry.update();
        time = System.currentTimeMillis();
    }

    public class ModuleThread implements Runnable{

        ModuleTasks moduleTasks;

        private boolean exit;
        Thread t;
        ModuleThread(String name) {
            moduleTasks = new ModuleTasks("name");
            t = new Thread(this, name);
            exit = false;
            t.start();
        }

        public void run(){
            moduleTasks.initialize();
            while(!exit) {
                moduleTasks.execute();

//                try{
//                    Thread.sleep(20);
//                }catch(InterruptedException e){
//                    e.printStackTrace();
//                }

            }
        }

        public void stop(){
            exit = true;
        }
    }

    class ModuleTasks {
        double time;
        double dt = 0;
        String name;

        int count = 0;

        ModuleTasks(String name_input){
            name = name_input;
        }
        void initialize(){
            time = System.currentTimeMillis();
        }

        void execute(){
            count++;

            dt = System.currentTimeMillis() - time;
            time = System.currentTimeMillis();
        }
    }


}
