@TeleOp(name="StateMachineExample")
public class StateMachine extends OpMode {
    
    //ENUM listing out the steps
    //LIFT_START = 1, LIFT_EXTEND = 2, so on
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    };

    //This embodies the enum and tells us which state the lift is in
    LiftState liftState = LiftState.LIFT_START;

    //Motor for a vertical slide
    public DcMotorEx liftMotor;

    //Bucket actuator
    public Servo liftDump;

    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    //CONSTANTS positions for the 'dump servo'
    final double DUMP_IDLE; // the idle position for the dump servo
    final double DUMP_DEPOSIT; // the dumping position for the dump servo

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME;

    //CONSTANTS the position targets for the vertical slide
    final int LIFT_LOW; // the low encoder position for the lift
    final int LIFT_HIGH; // the high encoder position for the lift

    public void init() {
        //Reset the ElapsedTime
        liftTimer.reset();

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftDump = hardwareMap.get(Servo.class, "liftDump");


        liftMotor.setTargetPosition(LIFT_LOW);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {
        liftMotor.setPower(1.0);

        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.x) {
                    // x is pressed, start extending
                    liftMotor.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                 // check if the lift has finished extending,
                 // otherwise do nothing.
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 1) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    liftDump.setTargetPosition(DUMP_DEPOSIT);

                    liftTimer.reset();
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    liftDump.setTargetPosition(DUMP_IDLE);
                    liftMotor.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                 // should never be reached, as liftState should never be null
                 liftState = LiftState.LIFT_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad1.y && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
        }

        // mecanum drive code goes here
        // But since none of the stuff in the switch case stops
        // the robot, this will always run!
        updateDrive(gamepad1, gamepad2);
   }
}
