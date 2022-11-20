package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;

/**
 * Kuriosity robot
 */
@BotConfig(name = "Kuriosity Bot", filename = "kuriosity_bot")
public class KuriosityBot extends MecanumPhysicsBase {
    private ServoImpl servo = null;

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML
    Rectangle backServoArm;

    public KuriosityBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
//        leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
//        rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
//        xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");
//
//        //Dimensions in pixels
//        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
//        leftEncoderX = LEFT_ENCODER_X * botWidth / 18.0;
//        rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
//        xEncoderY = X_ENCODER_Y * botWidth / 18.0;
//
        hardwareMap.setActive(false);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
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

        hardwareMap.setActive(false);
    }
}
