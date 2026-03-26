package org.firstinspires.ftc.teamcode.Decode.manual;

import static org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs.mod;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Decode.Carousel;

@Configurable
@TeleOp
public class carouselManip extends OpMode{
    private double startTime,
                   prevTime = -0.0001;
    private final double liftTimeToPosition = 1;
    private DcMotor carouselMotor;
    private Servo lift;
    private Carousel carousel = new Carousel();
    private double Kp = 4,
                   Ki = 2,
                   Kd = 0.05;
    private boolean canMoveCarousel = false,
                    carouselMoving = false,
                    inHalfPosition = false;
    private int mode = 0;
    @Override
    public void init(){
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel");
        lift = hardwareMap.get(Servo.class, "lift");
        carousel.assignMotor(carouselMotor);
        carousel.assignLift(lift);
    }

    @Override
    public void loop(){
        double deltaTime = time - prevTime;
        switch(mode){
            case 0:
                carouselMotor.setPower(gamepad1.right_stick_x);
                break;

            case 1:
                if(gamepad1.circle){
                    telemetry.addData("inTolerance?", carousel.approachPosition(deltaTime));
                }
                else{
                    carouselMotor.setPower(0);
                }

                if(gamepad1.leftBumperWasPressed()){
                    carousel.incrementPosition(-1 + mod(carousel.getPosition(), 1));
                }

                if(gamepad1.rightBumperWasPressed()){
                    carousel.incrementPosition(1 - mod(carousel.getPosition(), 1));
                }

                if (gamepad2.dpadUpWasPressed()){
                    Kp += 0.05;
                }
                if(gamepad2.dpadDownWasPressed()){
                    Kp -= 0.05;
                }

                if(gamepad2.dpadRightWasPressed()){
                    Ki += 0.05;
                }
                if(gamepad2.dpadLeftWasPressed()){
                    Ki -= 0.05;
                }

                if(gamepad2.triangleWasPressed()){
                    carousel.PID.setIntegralLimit(carousel.PID.getIntegralLimit()+0.05);
                }
                if(gamepad2.crossWasPressed()){
                    carousel.PID.setIntegralLimit(carousel.PID.getIntegralLimit()-0.05);
                }

                if(gamepad2.rightBumperWasPressed()){
                    Kd += 0.005;
                }
                if(gamepad2.leftBumperWasPressed()){
                    Kd -= 0.005;
                }

                telemetry.addData("Kp", Kp);
                telemetry.addData("Ki", Ki);
                telemetry.addData("Kd", Kd);
                telemetry.addData("integralLimit", carousel.PID.getIntegralLimit());
                telemetry.addData("deltaTime", deltaTime);
                telemetry.addData("power", Math.max(-1, Math.min(carousel.PID.getCorrection(), 1))/3 + (0.05 * Math.signum(carousel.PID.getCorrection())));
                telemetry.addData("proportionalTerm", Kp * carousel.distanceToPos(carousel.getPosition()));
                telemetry.addData("integralTerm", Ki * carousel.PID.getIntegral());
                telemetry.addData("differentialTerm", Kd * carousel.PID.getDifferential()/(Math.min(0.002, deltaTime)));
                carousel.PID.setConstants(Kp, Ki, Kd);
                break;

            case 2:

                if(gamepad1.leftBumperWasPressed()){
                    carousel.incrementPosition(-1 + mod(carousel.getPosition(), 1));
                    inHalfPosition = false;
                }

                if(gamepad1.rightBumperWasPressed()){
                    carousel.incrementPosition(1 - mod(carousel.getPosition(), 1));
                    inHalfPosition = false;
                }

                if(gamepad2.leftBumperWasPressed()){
                    carousel.incrementPosition(-0.5 - mod(carousel.getPosition(), 1));
                    inHalfPosition = true;
                }

                if(gamepad2.rightBumperWasPressed()){
                    carousel.incrementPosition(0.5 + mod(carousel.getPosition(), 1));
                    inHalfPosition = true;
                }

                //Circle pressed and carousel not moving
                if(gamepad2.circle && (!carouselMoving) && inHalfPosition){
                    carousel.liftUp();
                    canMoveCarousel = false;
                }
                //Circle not pressed (doesn't matter if carousel is moving or not)
                else{
                    if(gamepad2.circleWasReleased()){
                        startTime = time;
                    }
                    canMoveCarousel = time > (startTime + liftTimeToPosition);
                    carousel.liftDown();
                }

                //Try carousel movement
                if(canMoveCarousel) {
                    //Unwinding of carousel
                    if (Math.abs(carousel.getPosition()) >= 6) {
                        carousel.setPosition(carousel.getPosition() % 3);
                    }

                    //Moves and checks movement of carousel
                    carouselMoving = !carousel.approachPosition(deltaTime);
                }
                else carouselMotor.setPower(0);
                break;
        }

        if(gamepad1.crossWasPressed()) mode = (mode + 1) % 2;
        if(gamepad1.squareWasPressed()) carousel.resetEncoding();

        telemetry.addData("carouselPower", carouselMotor.getPower());
        telemetry.addData("mode", mode);
        telemetry.addData("distanceToPos", carousel.distanceToPos(carousel.getPosition()));
        telemetry.addData("carouselPos", carousel.getPosition());
        telemetry.addData("carouselTicks", carouselMotor.getCurrentPosition());
        telemetry.addData("canMoveCarousel?", canMoveCarousel);
        telemetry.addData("carouselMoving?", carouselMoving);
        telemetry.addData("servoPosition", lift.getPosition());

        prevTime = time;
    }
}
