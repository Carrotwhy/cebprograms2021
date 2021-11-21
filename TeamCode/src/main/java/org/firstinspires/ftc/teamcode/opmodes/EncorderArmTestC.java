package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.EncoderArm;

@TeleOp
public class EncorderArmTestC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        EncoderArm arm = new EncoderArm(robot);
        robot.registerSubsystem(arm);

        telemetry.addLine("Initialization Finished");
        telemetry.update();

        arm.setTargetAngle(0);

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            boolean keyA = gamepad1.a;
            boolean keyB = gamepad1.b;
            boolean keyX = gamepad1.x;

            if (gamepad1.a == true) {
                //double armAngle = Angle.norm(Math.atan2(0) + Math.PI/2);
                arm.setPosition(EncoderArm.POS.INIT);
                //double armAngle = 0;
                //arm.setTargetAngle(armAngle);
                //telemetry.addData("Target Angle", armAngle);
                telemetry.addData("Arm PID error", arm.getPIDError());
                telemetry.addData("Arm Angle", arm.getArmAngle());
            }
            else if (gamepad1.b == true) {
                //double armAngle = Angle.norm(Math.atan2(0) + Math.PI/2);
                arm.setPosition(EncoderArm.POS.SWEEP);
                //double armAngle = -150.0/180.0 * Math.PI;
                //arm.setTargetAngle(armAngle);EncorderArmTestC
                //telemetry.addData("Target Angle", armAngle);
                telemetry.addData("Arm PID error", arm.getPIDError());
                telemetry.addData("Arm Angle", arm.getArmAngle());
            }
            else if (gamepad1.x == true) {
                //double armAngle = Angle.norm(Math.atan2(0) + Math.PI/2);
                arm.setPosition(EncoderArm.POS.DUMP);
                //double armAngle = 10.0/180.0 * Math.PI;
                //arm.setTargetAngle(armAngle);
                //telemetry.addData("Target Angle", armAngle);
                telemetry.addData("Arm PID error", arm.getPIDError());
                telemetry.addData("Arm Angle", arm.getArmAngle());
            }
            telemetry.update();
        }
    }
}
