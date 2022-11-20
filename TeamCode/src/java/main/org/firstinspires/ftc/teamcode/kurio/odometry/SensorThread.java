package org.firstinspires.ftc.teamcode.kurio.odometry;

import com.qualcomm.hardware.lynx.LynxModule;
import de.esoco.coroutine.Coroutine;
import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.debug.telemetry.Telemeter;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;

import java.util.ArrayList;
import java.util.List;

import static de.esoco.coroutine.Coroutine.first;
import static de.esoco.coroutine.CoroutineScope.launch;

public class SensorThread implements Runnable, Telemeter {
//    private static final Coroutine<LynxModule, Void> bulkDataCoroutine = first(consume(LynxModule::getBulkData));

    private final Robot robot;

    private final ObamaTree odometry;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public SensorThread(Robot robot, Pose pose) {
        this.robot = robot;
        this.odometry = new ObamaTree(robot.getHardwareMap(), pose);
        robot.getTelemetryDump().registerTelemeter(this);
        robot.getTelemetryDump().registerTelemeter(odometry);
    }

    @Override
    public void run() {
        while (robot.running()) {
            long currentTime = System.currentTimeMillis();

//            launch(scope -> bulkDataCoroutine.runAsync(scope, robot.getControlHub()));

            odometry.update();
            robot.getTelemetryDump().sendPose(odometry.getPose());

            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
    }

    @Override
    public List<String> getTelemetryData() {
        List<String> retVal = new ArrayList<>();
        retVal.add("Sensor thread total update time: " + updateTime);

        return retVal;
    }

    @Override
    public String getName() {
        return "SensorThread";
    }

    @Override
    public boolean isOn() {
        return true;
    }

    public ObamaTree getOdometry(){
        return odometry;
    }
}
