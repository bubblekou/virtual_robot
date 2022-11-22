package virtual_robot.robots.classes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.util.AngleUtils;

/**
 * Kuriosity robot
 */
@BotConfig(name = "Kuriosity Bot", filename = "kuriosity_bot")
public class KuriosityBot extends MecanumPhysicsBase {
    private DeadWheelEncoder rightEncoder = null;
    private DeadWheelEncoder leftEncoder = null;
    private DeadWheelEncoder xEncoder = null;

    //Dimensions in inches for encoder wheels.
    //Right and left encoder wheels are oriented parallel to robot-Y axis (i.e., fwd-reverse)
    //X Encoder wheel is oriented parallel to the robot-X axis (i.e., right-left axis)
    private final double ENCODER_WHEEL_DIAMETER = 2.0;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double LEFT_ENCODER_X = -2.5;
    private final double RIGHT_ENCODER_X = 2.5;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_Y = 3.0;

    //Dimensions in pixels -- to be determined in the constructor
    private double encoderWheelRadius;
    private double leftEncoderX;
    private double rightEncoderX;
    private double xEncoderY;

    public KuriosityBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);

        leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");

        //Dimensions in pixels
        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        leftEncoderX = LEFT_ENCODER_X * botWidth / 18.0;
        rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
        xEncoderY = X_ENCODER_Y * botWidth / 18.0;

        hardwareMap.setActive(false);
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("back_servo", new ServoImpl());

        // Remaps default motors to Kuriosity's configuration
        hardwareMap.setActive(true);
        hardwareMap.put("fLeft", hardwareMap.dcMotor.get("front_left_motor"));
        hardwareMap.put("fRight", hardwareMap.dcMotor.get("front_right_motor"));
        hardwareMap.put("bLeft", hardwareMap.dcMotor.get("back_left_motor"));
        hardwareMap.put("bRight", hardwareMap.dcMotor.get("back_right_motor"));

        hardwareMap.put("enc_left", new DeadWheelEncoder(MotorType.NeverestOrbital20));
        hardwareMap.put("enc_right", new DeadWheelEncoder(MotorType.NeverestOrbital20));
        hardwareMap.put("enc_x", new DeadWheelEncoder(MotorType.NeverestOrbital20));

        // Electronics
        hardwareMap.put("Control Hub", new LynxModule());

        hardwareMap.setActive(false);
    }

    public synchronized void updateStateAndSensors(double millis){

        //Save old x, y, and headingRadians values for updating free wheel encoders later
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        //Compute new pose and update various sensors
        super.updateStateAndSensors(millis);

        //For the deadwheel encoders, recalculate dXR and dYR to take into account the fact that the robot
        //may have run into the wall.
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double dxR = deltaX * cos + deltaY * sin;
        double dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double rightEncoderRadians = (dyR + rightEncoderX * headingChange) / encoderWheelRadius;
        double leftEncoderRadians = -(dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = -(dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        rightEncoder.update(rightEncoderRadians, millis);
        leftEncoder.update(leftEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);
    }
}
