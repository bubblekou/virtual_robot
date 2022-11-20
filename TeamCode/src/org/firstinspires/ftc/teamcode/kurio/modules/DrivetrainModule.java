package org.firstinspires.ftc.teamcode.kurio.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainModule implements Module {
    public static final double EPSILON = 0.03;
    private final boolean isOn = true;

    private final DcMotor fLeft;
    private final DcMotor fRight;
    private final DcMotor bLeft;
    private final DcMotor bRight;

    private double fL, fR, bL, bR;

    public DrivetrainModule(HardwareMap hardwareMap) {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        setMotorPower(fLeft, fL);
        setMotorPower(fRight, fR);
        setMotorPower(bLeft, bL);
        setMotorPower(bRight, bR);
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < EPSILON) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "DrivetrainModule";
    }

    public void setPowers(double x, double y, double theta) {
        fL = -y + theta + x;
        fR = -y - theta - x;
        bL = -y + theta - x;
        bR = -y - theta + x;
    }
}