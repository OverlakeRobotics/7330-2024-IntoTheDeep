package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public static final int ARM_GROUND_POS = 0;
    public static final int ARM_LIFT_POS = 0;
    public static final int ARM_LOW_BASKET_POS = 0;
    public static final int ARM_HIGH_BASKET_POS = 0;

    public static final int ARM_CONTRACT_POS = 0;
    public static final int ARM_EXTEND_POS = 0;

    public static final double ARM_POWER = 1d;
    public static final double ARM_EXTEND_POWER = 1d;
    public static final double ARM_CONTRACT_POWER = 1d;
    public static final double INTAKE_POWER = 1d;
    public static final double OUTTAKE_POWER = -0.4d;

    private static final double SLOW_FACTOR = 0.5;

    private boolean slowMode;


    private DcMotor arm;
    private DcMotor extend;
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        this.arm = hardwareMap.get(DcMotor.class, "leftArm");
        this.extend = hardwareMap.get(DcMotor.class, "armExtend");
        this.intake = hardwareMap.get(DcMotor.class, "intake");


        initMotors();

    }

    public void initMotors() {
        arm.setDirection(DcMotor.Direction.FORWARD);
        extend.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setPower(0);
        extend.setPower(0);
        intake.setPower(0);
    }

    public boolean extendArm() {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(extend, ARM_EXTEND_POS, ARM_EXTEND_POWER);
    }
    public boolean contractArm() {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(extend, ARM_CONTRACT_POS, ARM_CONTRACT_POWER);
    }
    public boolean extendToPos(int pos, double power) {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(extend, pos, power);
    }

    public boolean armToGround() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(arm, ARM_GROUND_POS, ARM_POWER);
    }
    public boolean liftArm() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(arm, ARM_LIFT_POS, ARM_POWER);
    }
    public boolean armToLowBasket() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(arm, ARM_LOW_BASKET_POS, ARM_POWER);
    }
    public boolean armToHighBasket() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(arm, ARM_HIGH_BASKET_POS, ARM_POWER);
    }
    public boolean armToPos(int pos, double power) {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(arm, pos, power);
    }

    public void intake() {
        intake.setPower(INTAKE_POWER);
    }
    public void outtake() {
        intake.setPower(OUTTAKE_POWER);
    }
    public void stop() {
        intake.setPower(0);
    }

    public void driveArm(double power) {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(power);
    }

    public void driveArmUp() {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(ARM_POWER * (slowMode ? SLOW_FACTOR : 1));
    }
    public void driveArmDown() {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(-ARM_POWER * (slowMode ? SLOW_FACTOR : 1));
    }


    public void driveExtend(double power) {
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setPower(power * (slowMode ? SLOW_FACTOR : 1));
    }

    public void driveExtendUp() {
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setPower(ARM_EXTEND_POWER * (slowMode ? SLOW_FACTOR : 1));
    }
    public void driveExtendDown() {
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setPower(ARM_CONTRACT_POWER * (slowMode ? SLOW_FACTOR : 1));
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void setSlowMode (boolean slow) {
        slowMode = slow;
    }

    public boolean driveToPos(DcMotor motor, int targetPos, double power) {
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            throw new IllegalArgumentException(String.format(
                    "Motor %s must be set to \"RUN_TO_POSITION\" mode", motor.getDeviceName())
            );
        }

        motor.setTargetPosition(targetPos);
        motor.setPower(power);

        int offset = Math.abs(motor.getCurrentPosition() - targetPos);
        return offset < 50;
    }

}
