package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Drive {

    private final double SLOW_FACTOR = 0.25;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private boolean slowMode;

    public Drive (HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        initMotors();
    }

    public void initMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void setMotorPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void setMotorPower(double lf, double lb, double rb, double rf) {

        leftFront.setPower(Range.clip(lf * (slowMode ? SLOW_FACTOR : 1), -1, 1));
        leftBack.setPower(Range.clip(lb * (slowMode ? SLOW_FACTOR : 1), -1, 1));
        rightBack.setPower(Range.clip(rb * (slowMode ? SLOW_FACTOR : 1), -1, 1));
        rightFront.setPower(Range.clip(rf * (slowMode ? SLOW_FACTOR : 1), -1, 1));
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void setSlowMode(boolean slow) {
        slowMode = slow;
    }

    public void drive(float x, float y, float theta) {
        if (Math.abs(x) < 0.01) {
            x = 0.0f;
        }
        if (Math.abs(y) < 0.01) {
            y = 0.0f;
        }
        if (Math.abs(theta) < 0.01) {
            theta = 0.0f;
        }

        x = x * Math.abs(x);
        y = y * Math.abs(y);
        theta = theta * Math.abs(theta);

        double lf  = -y + theta + x;
        double rf = -y - theta - x;
        double lb   = -y + theta - x;
        double rb  = -y - theta + x;



        setMotorPower(lf, lb, rb, rf);
    }
}
