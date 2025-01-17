package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class EncoderArm implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx armMotor;
    private static final double TICKS_PER_REV = 7168; // 28 * 256 = 7168

    //PID Stuff
    private PIDFController armPID;
    private static final PIDCoefficients ARM_PID_COEFFICIENTS = new PIDCoefficients(1, 0, 0);

    private static final double ARM_ACCEPTABLE_ERROR_MARGIN = 0.05;

    public EncoderArm(Robot robot) {
        armMotor = robot.getMotor("armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armPID = new PIDFController(ARM_PID_COEFFICIENTS);
        //In order for the PID controller to find the most efficient way to go to the target position,
        //we need to bound the error inputted to the PID controller from -pi to pi radians
        armPID.setInputBounds(-Math.PI, Math.PI);
    }

    public void setTargetAngle(double targetAngle) {
        armPID.reset();
        armPID.setTargetPosition(targetAngle);
    }

    private double getRawArmAngle() {
        // encoder position * (2pi / TICKS_PER_REV)
        return armMotor.getCurrentPosition() * (2 * Math.PI / TICKS_PER_REV);
    }

    public double getArmAngle() {
        return Angle.norm(getRawArmAngle());
    }

    public double getPIDError() { return armPID.getLastError(); }

    public boolean targetReached() {
        return Math.abs(armPID.getLastError()) <= ARM_ACCEPTABLE_ERROR_MARGIN;
    }

    public enum POS {INIT, SWEEP, DUMP};

    public void setPosition(POS pos){
        if (pos == POS.INIT) {
            setTargetAngle(0);
        }
        else if (pos == POS.SWEEP) {
            setTargetAngle(-150.0/180*Math.PI);
        }
        else if (pos == POS.DUMP) {
            setTargetAngle(10.0/180.0*Math.PI);
        }
    }

    @Override
    public void update(TelemetryPacket packet) {
        double armPower = armPID.update(getArmAngle());
        armMotor.setPower(armPower);
    }
}
