private const byte tickRate = 180;         // degrees per second turned by reference rotor (ticks)
private const byte numSegments = 16;       // number of body segments
private const byte rpmToDeg = 6;           // 1RPM = 6 degrees/s
private const float contactRatio = 0.5F;   // how much of each step is spent in contact with ground
private const float liftRatio = 0.25F;     // how much of the time each leg spends off the ground is spent lifting/lowering the leg
private const float nextDelay = 0.25F;     // how much of a step to complete before triggering next segment
private const float turnSensitivity = 18F; // minimum 12 to achieve max turn circle (degrees/s)
private const sbyte maxSpeed = 16;         // (m/s)
private const sbyte maxReverse = -6;       // (m/s)
private const sbyte speedIncrement = 2;    // (m/s)
private const byte inputTime = 60;         // delay between speed input readings (ticks)
private const float strideLength = 17.5F;  // theoretical distance travelled per step (m)
private const byte numTimers = 3;          // number of timers available to each timed actor
private const float sensitivityH = 0.7F;   // horizontal mouse input sensitivity
private const float sensitivityV = 0.7F;   // vertical mouse input sensitivity

private static IMyShipController controller; // pilot's chair
private static IMyMotorStator timeRef;       // rotor that acts as an indicator of sim-speed
private static IMyTextPanel display;         // LCD panel for input processing
private static IMyMotorStator headH;         // horizontal head joint
private static IMyMotorStator headV;         // vertical head joint

private static TimedActor[] triggers = new TimedActor[numSegments*2 + 1]; // array of objects with delayed actions
private static Leg[] leftLegs = new Leg[numSegments];                     // array of left legs (front to back)
private static Leg[] rightLegs = new Leg[numSegments];                    // array of right legs (front to back)
private static Spine[] spines = new Spine[numSegments - 1];               // array of spinal joints (front to back)
private static InputBuffer inputBuffer;                                   // reads speed input

private static sbyte targetSpeed = 0;     // (m/s)
private static sbyte mechanicalSpeed = 0; // current target speed (m/s)
private static ulong time = 0;            // current time (ticks)
private static ulong lastStep = 0;        // time of last step (ticks)
private static ushort lastRef = 0;        // angle of reference rotor at last check (degrees)
private static ushort tickJump = 0;       // number of ticks since last processing run (ticks)

/**
* initializes component handlers, leg objects, spine objects and timing system
*/
public Program(){
    controller = (IMyShipController) GridTerminalSystem.GetBlockWithName("Control Seat Pilot");
    display = (IMyTextPanel) GridTerminalSystem.GetBlockWithName("LCD Panel Display");
    timeRef = (IMyMotorStator) GridTerminalSystem.GetBlockWithName("Rotor Time Ref");
    headH = (IMyMotorStator) GridTerminalSystem.GetBlockWithName("Rotor Head H");
    headV = (IMyMotorStator) GridTerminalSystem.GetBlockWithName("Rotor Head V");
    lastRef =  (ushort) (timeRef.Angle * (180 / Math.PI));
    inputBuffer = new InputBuffer(this);

    for(byte i = numSegments - 1; i < numSegments; i--){
        if(i == numSegments-1){
            leftLegs[i] = new Leg(i, " Rotor L", this, 1, null);
            rightLegs[i] = new Leg(i, " Rotor R", this, -1, null);
        } else{
            leftLegs[i] = new Leg(i, " Rotor L", this, 1, leftLegs[i+1]);
            rightLegs[i] = new Leg(i, " Rotor R", this, -1, rightLegs[i+1]);
            spines[i] = new Spine(i, " Advanced Rotor ", this);
        }
        triggers[i*2] = leftLegs[i];
        triggers[i*2 + 1] = rightLegs[i];
    }

    triggers[numSegments*2] = inputBuffer;
    inputBuffer.arm(time + inputTime);
}

/**
*   - increments percieved time of program (based on sim-speed)
*   - processes mouse input
*   - checks if its time to start another step
*   - checks all timed actions and triggers any that are due
*/
public void Main(string argument){
    incrementTime();
    mouseInput();
    if((time - lastStep) > (leftLegs[0].getCycleTime() * (float) tickRate)){
        step();
    }
    for(int i = 0; i < triggers.Length; i++){
        triggers[i].check(time, tickJump);
    }
}

/**
* sets velocities of head joints using mouse input
*/
private void mouseInput(){
   headH.SetValueFloat("Velocity", -(float)(controller.RotationIndicator.Y *sensitivityH));
   headV.SetValueFloat("Velocity", (float)(controller.RotationIndicator.X *sensitivityV));
}

/**
* sets the current percieved time acording to sim-speed
* (program is triggered at non-regular times, reference rotor ensures 'true' simulation speed based time)
*/
private void incrementTime(){
    ushort angle = (ushort) (timeRef.Angle * (180F / Math.PI)); // current angle of the reference rotor (degrees)

    if(angle > lastRef){
        tickJump = Convert.ToUInt16(angle - lastRef);
    } else{
        tickJump = Convert.ToUInt16(360F - lastRef + angle);
    }

    time += tickJump;
    lastRef = angle;
}

/**
* - increments walking speed towards target speed
* - if new speed = 0, does nothing
* - otherwise, if already moving, starts next step at the front
* - otherwise, if starting from 0, starts next step at regular intervals
*/
private void step(){
    bool startingStep = false; // whether or not it is starting from stationary

    if(mechanicalSpeed == 0){
        startingStep = true;
    }

    if(targetSpeed > mechanicalSpeed){
        mechanicalSpeed += speedIncrement;
    } else if(targetSpeed < mechanicalSpeed){
        mechanicalSpeed -= speedIncrement;
    }

    for(int i = 0; i < numSegments; i++){
        leftLegs[i].setSpeed(mechanicalSpeed);
        rightLegs[i].setSpeed(mechanicalSpeed);
    }

    if(mechanicalSpeed != 0){
        if(startingStep){
            for(int i = 0; i < numSegments; i += (byte) (1 / nextDelay)){
                leftLegs[i].trigger(time);
                if((byte) (i + (1 / (2 * nextDelay))) < numSegments){
                    rightLegs[(byte) (i + (1 / (2 * nextDelay)))].trigger(time);
                }
            }

        } else{
            leftLegs[0].trigger(time);
        }
        rightLegs[0].arm(time + (ulong) (leftLegs[0].getCycleTime() * (float) tickRate / 2F));
    }

    lastStep = time;
}

/**
*   abstract class that handles delayed actions
*/
public abstract class TimedActor {
    protected MyGridProgram progBlock;                      // reference to the running program to allow console output
    protected ulong[] triggerPoints = new ulong[numTimers]; // times at which to trigger
    public bool[] armed = new bool[numTimers];              // whether or not each trigger point is armed

    public TimedActor(MyGridProgram progBlock){
        this.progBlock = progBlock;
        for(int i = 0; i < numTimers; i++){
            triggerPoints[i] = 0;
            armed[i] = false;
        }
    }

    public abstract void trigger(ulong currentTime);

    /**
    * finds a free trigger point and arms it for the given time
    */
    public void arm(ulong trigger){
        for(int i = 0; i < numTimers; i++){
            if(!armed[i]){
                triggerPoints[i] = trigger;
                armed[i] = true;
                i = numTimers;
            }
        }
    }

    /**
    * checks each trigger point and triggers any that are armed and due
    */
    public void check(ulong time, ushort tickJump){
        if(time > time - tickJump){ // no time integer overflow
            for(int i = 0; i < numTimers; i++){
                if(armed[i] && time - tickJump < triggerPoints[i] && time >= triggerPoints[i]){
                    armed[i] = false;
                    trigger(time);
                }
            }
        } else{ // time integer overflow
            for(int i = 0; i < numTimers; i++){
                if(armed[i] && (time - tickJump < triggerPoints[i] || time >= triggerPoints[i])){
                    armed[i] = false;
                    trigger(time);
                }
            }
        }
    }
}

/**
* handles regular reading of speed input
*/
public class InputBuffer : TimedActor{
    // X+ = right
    // Y+ = up
    // Z+ = back
    private Vector3 moveInput; // keyboard input (forward/backward, left/right, up/down)

    public InputBuffer(MyGridProgram progBlock) : base(progBlock){}

    /**
    * processes user movement input
    * - converts 'forward/backward' (-Z/+Z) to 'increment/decrement target speed' (within limits)
    * - converts 'left/right' (-X/+X) to 'bend spine 'left/right' (within mechanical limits of joints)
    * - updates target speed on output display
    * - arms itself to repeat after some time
    */
    public override void trigger(ulong currentTime){
        moveInput = controller.MoveIndicator;

        if(-moveInput.Z > 0 && targetSpeed < maxSpeed){
            targetSpeed += speedIncrement;
        }
        if(-moveInput.Z < 0 && targetSpeed > maxReverse){
            targetSpeed -= speedIncrement;
        }

        for(int i = 0; i < numSegments - 1; i++){
            spines[i].turn(-moveInput.X * turnSensitivity);
        }

        display.WriteText("Speed: " + targetSpeed + "m/s", false);
        arm(currentTime + inputTime);
    }
}

/**
* handles the movement of a single leg and triggering of a following leg
*/
public class Leg : TimedActor {
    // rpm required to turn each joint through its full range in 1 second
    private const float rangeJ1 = 60F / (float) rpmToDeg;
    private const float rangeJ2 = 35F / (float) rpmToDeg;
    private const float rangeJ3 = 70F / (float) rpmToDeg;
    private const float rangeJ4 = 35F / (float) rpmToDeg;

    private Leg nextLeg;

    private sbyte mechanicalSpeed;  // (m/s)
    private byte animationStep = 0; // which part of this legs walking 'animation' it is currently in
    private float cycleTime;        // how long this leg has to complete 1 step (s)
    private float side = 1;         // +1 for left leg, -1 for right leg
    private float direction = 1;    // +1 for forward, -1 for backward

    // the 4 joints that make up the leg, ordered from body to foot
    private IMyMotorStator J1;
    private IMyMotorStator J2;
    private IMyMotorStator J3;
    private IMyMotorStator J4;

    /**
    * initialize the 4 leg joints and orientation with given parameters
    */
    public Leg(byte index, string tag, MyGridProgram progBlock, float side, Leg nextLeg) : base(progBlock) {
        J1 = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "1");
        J2 = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "2");
        J3 = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "3");
        J4 = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "4");
        this.side = side;
        this.nextLeg = nextLeg;
    }

    public float getCycleTime(){
        return cycleTime;
    }

/**
* -sets the walking speed of the leg
* -if the speed is negative, sets the leg to reverse mode
*/
    public void setSpeed(sbyte speed){
        float strideTime = 0; // how long the leg will have on the ground to achieve target speed (s)

        mechanicalSpeed = speed;

        if(speed != 0){
            if(mechanicalSpeed > 0){
                direction = 1;
                strideTime = strideLength/(float) mechanicalSpeed;
            } else{
                direction = -1;
                strideTime = strideLength/(float) -mechanicalSpeed;
            }
            cycleTime = strideTime / contactRatio;
        }
    }

    /**
    * - completes the next movement in a step
    * - sets the time to start the next movement
    * - sets the time for the next leg to start its step
    */
    public override void trigger(ulong currentTime){

        switch(animationStep){

            case 0: // first half of step
                if(mechanicalSpeed != 0){
                    if(nextLeg != null){
                        nextLeg.arm(currentTime + (ulong) (cycleTime * nextDelay * tickRate));
                    }

                    J1.TargetVelocityRPM = direction * side * (rangeJ1) / (cycleTime * contactRatio);
                    J2.TargetVelocityRPM = side * -(rangeJ2 * 2F) / (cycleTime * contactRatio);
                    J3.TargetVelocityRPM = side * -(rangeJ3 * 2F) / (cycleTime * contactRatio);
                    J4.TargetVelocityRPM = side * -(rangeJ4 * 2F) / (cycleTime * contactRatio);
                    animationStep++;
                    arm(currentTime + (ulong) (((float) tickRate * cycleTime * contactRatio) / 2F));
                }
                break;

            case 1: // second half of step
                J2.TargetVelocityRPM = - J2.TargetVelocityRPM;
                J3.TargetVelocityRPM = - J3.TargetVelocityRPM;
                J4.TargetVelocityRPM = - J4.TargetVelocityRPM;
                animationStep++;
                arm(currentTime + (ulong) (((float) tickRate * cycleTime * contactRatio) / 2F));
                break;

            case 2: // lift leg
                J2.TargetVelocityRPM = side * -(rangeJ2) / (cycleTime * (1 - contactRatio) * liftRatio);
                animationStep++;
                arm(currentTime + (ulong) ((float) tickRate * cycleTime * (1 - contactRatio) * liftRatio));
                break;

            case 3: // move leg forward
                J1.TargetVelocityRPM = direction * side * -(rangeJ1) / (cycleTime * (1 - contactRatio) * (1 - (2 * liftRatio)));
                animationStep++;
                arm(currentTime + (ulong) ((float) tickRate * cycleTime * (1 - contactRatio) * (1 - (2 * liftRatio))));
                break;

            case 4: // lower leg
                J2.TargetVelocityRPM = - J2.TargetVelocityRPM;
                animationStep = 0;
                break;
        }
    }
}

/**
* handles turning using the spinal joints between each segment
*/
public class Spine {

    // rotors for each axis connecting to the next segment
    private IMyMotorStator yaw;
    private IMyMotorStator pitch;
    private IMyMotorStator roll;

    public Spine(byte index, string tag, MyGridProgram progBlock) {
        yaw = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "H");
        pitch = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "V");
        roll = (IMyMotorStator) progBlock.GridTerminalSystem.GetBlockWithName(index + tag + "R");
    }

    /**
    * turns the front in the target direction
    */
    public void turn(float turn){
        yaw.TargetVelocityRPM = turn - yaw.Angle;
    }
}