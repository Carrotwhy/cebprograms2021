 package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.CachingSensor;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class SimpleMecanumDrive implements Subsystem {

            
    private DcMotorEx[] motors = new DcMotorEx[4];
    private CachingSensor<Float> headingSensor;
    private DistanceSensor distL, distR;

    //private AnalogSensor scale_n, scale_p;
    private int MOTOR_LF = 0;
    private int MOTOR_LR = 1;
    private int MOTOR_RR = 2;
    private int MOTOR_RF = 3;

    private Double[] powers = {0.0, 0.0, 0.0, 0.0};

    private PIDFController lPID;
    private static final PIDCoefficients L_PID_COEFFICIENTS = new PIDCoefficients(0.01, 0, 0);

    private PIDFController rPID;
    private static final PIDCoefficients R_PID_COEFFICIENTS = new PIDCoefficients(0.01, 0, 0);

    private static final double DIST_ACCEPTABLE_ERROR_MARGIN = 30;


    public SimpleMecanumDrive (Robot robot) {
        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveLR");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveRF");
        distL = robot.getDistanceSensor("distanceL");
        distR = robot.getDistanceSensor("distanceR");

        BNO055IMU imu = robot.getIMU("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        headingSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle);
        robot.addListener(headingSensor);
        //scale_n = robot.getAnalogSensor("NInput");
        //scale_p = robot.getAnalogSensor("PInput");

        lPID = new PIDFController(L_PID_COEFFICIENTS);
        rPID = new PIDFController(R_PID_COEFFICIENTS);
        lPID.reset();
        rPID.reset();

        motors[MOTOR_LF].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MOTOR_LR].setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setDrivePower(Pose2d drivePower) {
        powers[MOTOR_LF] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
        powers[MOTOR_LR] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();

        powers[MOTOR_RR] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
        powers[MOTOR_RF] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
    }

    public double getHeading() {
        return headingSensor.getValue();
    }
    public double getDistanceL() {
        return distL.getDistance(DistanceUnit.MM);
    }
    public double getDistanceR() {
        return distR.getDistance(DistanceUnit.MM);
    }

    private int auto_mode_l = 0;
    private int auto_mode_r = 0;

    public void setTargetDistance(double targetDistance) {
        if (targetDistance < 0){
            auto_mode_l = 0;
            auto_mode_r = 0;
        } else {
            lPID.reset();
            rPID.reset();
            lPID.setTargetPosition(targetDistance);
            rPID.setTargetPosition(targetDistance);
            auto_mode_l = 1;
            auto_mode_r = 1;
        }
    }


    @Override
    public void update(TelemetryPacket packet) {
        double lPower, rPower;
        double lError, rError;

        lPower = 0;
        lError = 0;
        if (auto_mode_l == 0) {
            motors[MOTOR_LF].setPower(powers[MOTOR_LF]);
            motors[MOTOR_LR].setPower(powers[MOTOR_LR]);
        } else {
            lPower = lPID.update(getDistanceL());
            lError = lPID.getLastError();
            if (Math.abs(lError) <= DIST_ACCEPTABLE_ERROR_MARGIN) {
                auto_mode_l = 0;
                motors[MOTOR_LF].setPower(0);
                motors[MOTOR_LR].setPower(0);
            } else {
                motors[MOTOR_LF].setPower(0.3 * lPower);
                motors[MOTOR_LR].setPower(0.3 * lPower);
            }
        }

        rPower = 0;
        rError = 0;
        if (auto_mode_r == 0) {
            motors[MOTOR_RF].setPower(powers[MOTOR_RF]);
            motors[MOTOR_RR].setPower(powers[MOTOR_RR]);
        } else {
            rPower = rPID.update(getDistanceR());
            rError = rPID.getLastError();
            if (Math.abs(rError) <= DIST_ACCEPTABLE_ERROR_MARGIN) {
                auto_mode_r = 0;
                motors[MOTOR_RF].setPower(0);
                motors[MOTOR_RR].setPower(0);
            } else {
                motors[MOTOR_RF].setPower(0.3 * rPower);
                motors[MOTOR_RR].setPower(0.3 * rPower);
            }
        }
        packet.put("lError", lError);
        packet.put("lPower", lPower);
        packet.put("LAutom", auto_mode_l);
        packet.put("rError", rError);
        packet.put("rPower", rPower);
        packet.put("RAutom", auto_mode_r);
    }
}
