package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@TeleOp
public class EncorderSlideTestC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Slide slide = new Slide(robot);
        robot.registerSubsystem(slide);

        telemetry.addLine("Initialization Finished");
        telemetry.update();

        //arm.setTargetAngle(0);

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            boolean keyA = gamepad1.a;
            boolean keyB = gamepad1.b;
            boolean keyX = gamepad1.x;
            boolean keyY = gamepad1.y;

            if (gamepad1.a == true) {
                //double armAngle = Angle.norm(Math.atan2(0) + Math.PI/2);
                slide.goDown();
                telemetry.addData("Level", slide.level);
                //double armAngle = 0;
                //arm.setTargetAngle(armAngle);
                //telemetry.addData("Target Angle", armAngle);
                /*
                telemetry.addData("Arm PID error", arm.getPIDError());
                telemetry.addData("Arm Angle", arm.getArmAngle());
                 */
            }
            else if (gamepad1.y == true) {
                //double armAngle = Angle.norm(Math.atan2(0) + Math.PI/2);
                slide.goUp();
                telemetry.addData("Level", slide.level);
                //double armAngle = 10.0/180.0 * Math.PI;
                //arm.setTargetAngle(armAngle);
                //telemetry.addData("Target Angle", armAngle);
                /*
                telemetry.addData("Arm PID error", arm.getPIDError());
                telemetry.addData("Arm Angle", arm.getArmAngle());
                 */
            }
            telemetry.update();
        }
    }
}
