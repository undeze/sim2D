package sim2D;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;


/*
*   sim2.java does all the calculations for lift, drag, flight path, etc.
*   Author: Chris Underwood
*/


public class sim2 implements Runnable  
{
    private Thread runner;
    static private boolean going = true;

    static int CoGtoTailDistance;

    static double dragCoefficient, liftCoefficient, Cl, Cd;

    static double velX, velY, velZ;
    static String VelX, VelY, VelZ;

    static double forceX, forceY, forceZ, thrustX, thrustY, thrustZ;
    static String ForceX, ForceY, ForceZ, ThrustX, ThrustY, ThrustZ;
    static double liftwtX, liftwtY, liftwtZ, dragwtX, dragwtY, dragwtZ;
    static String LiftwtX, LiftwtY, LiftwtZ, DragwtX, DragwtY, DragwtZ;
    static double sideLiftX, sideLiftY, sideLiftZ, sideLiftForceX, sideLiftForceY, sideLiftForceZ;
    static String SideLiftForceX, SideLiftForceY, SideLiftForceZ;
    static double bodyDX, bodyDY, bodyDZ;
    static String BodyDX, BodyDY, BodyDZ;
    static double gravityZ;
    static String GravityZ;
    static double accelerationX, accelerationY, accelerationZ;

    static double positionX, positionY, positionZ;
    static String PositionX, PositionY, PositionZ;
    static double altitudeFeet;

    //G calculations
    static double gX, gY, gZ, G, gPosNeg;
    static String gString;
    static double balance, balCor;

    static double aircraftMass;
    static double bodyArea;

    static double sideSlip, sideSlipDegrees;   //angles. Radians, measured clockwise from north
    static double AoA, AoADegrees; 
    static String AOADegrees, SideSlipDegreesString, AOADegrees2;

    //matrix defining aircraft attitude
    static double a11, a12, a13, a21, a22, a23, a31, a32, a33;
    static double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    static double b11, b12, b13, b21, b22, b23, b31, b32, b33;
    static double ux, uy, uz; //unit vectors
    static double rotate;

    static String A11, A12, A13, A21, A22, A23, A31, A32, A33;

    //projection vector 
    static double wX, wY, wZ; //calculate side slip and AoA methods
    static double dot, norm;  //calculate side slip and AoA methods
    static double ssVel, AoAVel; //sideSlipVelocity

    static int St, Ix, Iy, Iz; //Vane Surface Area, Mass moment of inertia
    static double alphaYaw; //angular acceleration
    static double omegaYaw; //angular velocity
    static double lift, drag, liftTorque, dragTorque, torque, pitchTorque, yawTorque;
    static double alphaAoA;
    static double omegaAoA;

    static double deltaT;
    static boolean print;
    static double dragCorrection;

    static double ias, iasKnots; //airspeed

    static double alphaQ, betaQ, gammaQ; //yaw, pitch, roll 
    static String Alpha, Beta, Gamma;
    static double alphaT, betaT, gammaT;

    static double aileronP, rollRate, rollAccel, rollTorque, wingSpan;
    static double elevatorP, rudderP;
    static double rollTorqueL, rollTorqueR;

    static double Sw;
    static double thrust;
    static double liftWingTail, dragWingTail;

    static double liftX, liftY, liftZ, liftVectorTotal, speed;
    static double fuselageSideArea, sideLiftVectorTotal, liftSideTotal;

    Queue<Double> a11Q = new LinkedList<Double>();
    Queue<Double> a12Q = new LinkedList<Double>();
    Queue<Double> a13Q = new LinkedList<Double>();
    Queue<Double> a21Q = new LinkedList<Double>();
    Queue<Double> a22Q = new LinkedList<Double>();
    Queue<Double> a23Q = new LinkedList<Double>();
    Queue<Double> a31Q = new LinkedList<Double>();
    Queue<Double> a32Q = new LinkedList<Double>();
    Queue<Double> a33Q = new LinkedList<Double>();
    static double a11q, a12q, a13q, a21q, a22q, a23q, a31q, a32q, a33q;
    Queue<Double> AoAQ = new LinkedList<Double>();
    static double AoAq;
    Queue<Double> rollQ = new LinkedList<Double>();
    Queue<Double> pitchQ = new LinkedList<Double>();
    Queue<Double> yawQ = new LinkedList<Double>();
    static double rollq, pitchq, yawq;
    Queue<Double> GQ = new LinkedList<Double>();
    static double gQ;
    Queue<Double> velZQ = new LinkedList<Double>();
    static double velZq;
    static double dummyq;//dummyq to stop graphics skipping
    Queue<Double> thrustZQ = new LinkedList<Double>();
    static double thrustZq;

    static double bodyRateRoll, bodyRatePitch, bodyRateYaw;
    static String BodyRateRoll, BodyRatePitch, BodyRateYaw;

    static long tNow, tThen;
    static String DeltaT;

    static double wingTipVel, tipX, tipY, tipZ, tipAoA, tipAoADegrees;
    static double aileronSize = 0.01; // square meters
    static double leftWingAilLiftC, rightWingAilLiftC;
    static double leftAileronPosition, rightAileronPosition;

    static int rollMode = 2;
    static String RollMode;

    static double leftAoA, rightAoA;

    static boolean landingSuccessful, landed;
    static double groundSpeed;
    static String GroundSpeed;
    
    static double velocityVector;
    
    static double wheelsDownHeight;

    sim2(){ 
        reset();
        wX = 0; wY = 0; wZ = 0;

        tNow = System.currentTimeMillis();

        CoGtoTailDistance = 100; //Length from CoG to tail CoP (horizontal stabilizer & vertical tail)
        wingSpan = 100;

        St = 20; //vertical & horizontal tail surface areas. 
        Sw = 60; //wing area

        alphaYaw = 0; //angular acceleration
        omegaYaw= 0; //angular velocity

        Ix = 100000; Iy = 100000; Iz = 50000;

        deltaT = 0.005;
        print = false;
        dragCorrection = 10;

        gravityZ = 9.81; //m/s^2
        aircraftMass = 8000; //8,000 kilograms
        bodyArea = 0.25;// 0.16;
        GravityZ = String.valueOf(gravityZ * aircraftMass); 
        fuselageSideArea = 10;
        
        wheelsDownHeight = -3.0;

        if (runner == null) {
            runner = new Thread(this);
            runner.start();
        }
    }

    public static void reset(){
        rollRate = 0;
        positionX = 0; positionY = 0; positionZ = 10000/-3.28084; //3.28084 - meters to feet
        accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        alphaYaw = 0; alphaAoA = 0;
        //rollRate = 0;
        velX = 200.0/1.94384; velY = 0; velZ = 0; //1.94384 - m/s to knots
        a11 = 1;  a12 = 0;  a13 = 0; 
        a21 = 0;  a22 = 1;  a23 = 0; 
        a31 = 0;  a32 = 0;  a33 = -1;
        ux = 0; uy = 1; uz = 0; //set unit vector to select a pitch rotation
        R(0.5*Math.PI/180); //pitch 0.5 degrees nose up.
        landed = false;
    }

    public static void resetNorthLanding(){
        rollRate = 0;
        positionX = -900;
        positionY = 0; positionZ = 500/-3.28084; //3.28084 - meters to feet
        accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        bodyRateYaw = 0;    bodyRatePitch = 0;
        alphaYaw = 0; alphaAoA = 0;
        velX = 140.0/1.94384; velY = 0; velZ = 0; //1.94384 - m/s to knots
        a11 = 1;  a12 = 0;  a13 = 0; 
        a21 = 0;  a22 = 1;  a23 = 0; 
        a31 = 0;  a32 = 0;  a33 = -1;
        ux = 0; uy = 1; uz = 0; //set unit vector to select a pitch rotation
        R(0.5*Math.PI/180); //pitch 0.5 degrees nose up.
        landed = false;
    }
    
    public static void resetNorth10nmLanding(){
        rollRate = 0;
        positionX = -16550;
        positionY = 2000; positionZ = 2000/-3.28084; //3.28084 - meters to feet
        accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        bodyRateYaw = 0;    bodyRatePitch = 0;
        alphaYaw = 0; alphaAoA = 0;
        
        velX = 180.0*Math.cos(Math.PI/6.0)/1.94384; 
        velY = -180.0*Math.sin(Math.PI/6.0)/1.94384; 
        velZ = 0; //1.94384 - m/s to knots
        
        a11 = Math.cos(Math.PI/6.0);   a12 = Math.sin(Math.PI/6.0);   a13 = 0; 
        a21 = -Math.sin(Math.PI/6.0);  a22 = Math.cos(Math.PI/6.0);   a23 = 0; 
        a31 = 0;                        a32 = 0;                        a33 = -1;
        ux = 0; uy = 1; uz = 0; //set unit vector to select a pitch rotation
        R(0.5*Math.PI/180); //pitch 0.5 degrees nose up.
        landed = false;
    }
    
    public static void resetCrash(){
        a11 = 0;  a12 = 0;  a13 = 1; 
        a21 = 0;  a22 = 1;  a23 = 0; 
        a31 = 1;  a32 = 0;  a33 = 0;
        
        //accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        //alphaYaw = 0; alphaAoA = 0; omegaAoA = 0; omegaYaw = 0;
        //liftCoefficient = 0; liftTorque = 0; yawTorque = 0; dragTorque = 0; dragCoefficient = 0;
        //rollRate = 0;
        positionX = 2100;
        positionY = 0; positionZ = -10.0; //3.28084 - meters to feet
        //    velZq = 0;
        velX = 0.0; velY = 0; velZ = 100; //1.94384 - m/s to knots
        
        landed = false;
    }
    public static void resetNorthTakeoff(){
        a11 = 1;  a12 = 0;  a13 = 0; 
        a21 = 0;  a22 = 1;  a23 = 0; 
        a31 = 0;  a32 = 0;  a33 = -1;
        
        accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        alphaYaw = 0; alphaAoA = 0; omegaAoA = 0; omegaYaw = 0;
        liftCoefficient = 0; liftTorque = 0; yawTorque = 0; dragTorque = 0; dragCoefficient = 0;
        rollRate = 0;
        positionX = 2100;
        positionY = 0; positionZ = -3.0; //3.28084 - meters to feet
        velZq = 0;
        velX = 0.0; velY = 0; velZ = 0; //1.94384 - m/s to knots
        
        landed = false;
    }
    
    public static void resetSouthLanding(){
        rollRate = 0;
        positionX = ((3000/3.28084)/Math.tan(3*Math.PI/180));
        positionY = 0; positionZ = 1500/-3.28084; //3.28084 - meters to feet
        accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        alphaYaw = 0; alphaAoA = 0;
        velX = -140.0/1.94384; velY = 0; velZ = 0; //1.94384 - m/s to knots
        a11 = -1;  a12 = 0;  a13 = 0; 
        a21 = 0;  a22 = -1;  a23 = 0; 
        a31 = 0;  a32 = 0;  a33 = -1;
        ux = 0; uy = 1; uz = 0; //set unit vector to select a pitch rotation
        R(0.5*Math.PI/180); //pitch 0.5 degrees nose up.
        landed = false;
    }
    
    public static void resetGlide(){
        rollRate = 0;
        positionX = -26000;
        positionY = 3000; 
        positionZ = 20000/-3.28084; //3.28084 - meters to feet
        accelerationX = 0; accelerationY = 0; accelerationZ = 0;
        alphaYaw = 0; alphaAoA = 0;
        velX = 0;
        velY = -(200.0/1.94384)*Math.cos(10*Math.PI/180); 
        velZ = (200.0/1.94384)*Math.sin(10*Math.PI/180);; //1.94384 - m/s to knots
        a11 = 0;  a12 = 1;  a13 = 0; 
        a21 = -1;  a22 = 0;  a23 = 0; 
        a31 = 0;  a32 = 0;  a33 = -1;
        ux = 0; uy = 1; uz = 0; //set unit vector to select a pitch rotation
        R(-10*Math.PI/180); //pitch 0.5 degrees nose up.
        landed = false;
    }
    
    public static void stopHere(){
        going = false;
    }

    public void run(){
        while(going){
            //tThen = tNow;
            //tNow = System.currentTimeMillis();
            //deltaT = ((double)(tNow - tThen))/1000.0;
            //DeltaT = String.valueOf(deltaT);

            calcSideSlip(); //calculates sideSlip, ssVel, sideSlipDegrees
            //Lift
            rudderP = simGraphics1.rudPos/10.0;
            liftCoefficient = liftC(sideSlipDegrees + rudderP);
            lift = liftCoefficient * ssVel * ssVel * St;
            liftTorque = lift * CoGtoTailDistance;
            //Drag
            dragCoefficient = dragC(sideSlipDegrees + rudderP);
            drag = dragCoefficient * ssVel * ssVel * St;
            if (sideSlipDegrees + rudderP > 0 && omegaYaw> 0) drag = drag * dragCorrection;
            if (sideSlipDegrees + rudderP < 0 && omegaYaw< 0) drag = drag * dragCorrection;
            dragTorque = drag * CoGtoTailDistance;
            //Torque
            yawTorque = Math.sqrt((dragTorque*dragTorque)+(liftTorque*liftTorque));
            if(sideSlipDegrees + rudderP > 0) yawTorque = Math.abs(yawTorque) * -1;
            //Angular acceleration
            alphaYaw = yawTorque/Iz; //angular acceleration
            ux = a13; uy = a23; uz = a33; //defines the unit vector for yaw
            //Calculate the angle to yaw through. Then call the rotation matrix with
            //this value.
            rotate = (omegaYaw* deltaT) + (0.5 * alphaYaw * deltaT * deltaT);
            //Call the rotation method. This will define the new aircraft orientation.
            R(rotate);  //R() -> B() -> redefineA()
            bodyRateYaw = (rotate/deltaT)*180/Math.PI;

            //angular velocity = omega
            omegaYaw= omegaYaw+ (alphaYaw * deltaT);
            if (sideSlip < 0.001 && sideSlip > -0.001 && omegaYaw< 0.04){
                omegaYaw= omegaYaw* 0.9;
            }
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            calcAoA();
            //Lift - from horizonal stabilizer and elevator
            elevatorP = -simGraphics1.elePos;//10.0;
            liftCoefficient = liftC(AoADegrees + elevatorP);
            lift = liftCoefficient * AoAVel * AoAVel * St;
            liftTorque = lift * CoGtoTailDistance;
            //Drag - from horizonal stabilizer and elevator
            dragCoefficient = dragC(AoADegrees + elevatorP);
            drag = dragCoefficient * AoAVel * AoAVel * St;
            if ((AoADegrees + elevatorP) > 0 && omegaAoA > 0) drag = drag * dragCorrection;
            if ((AoADegrees + elevatorP) < 0 && omegaAoA < 0) drag = drag * dragCorrection;
            dragTorque = drag * CoGtoTailDistance;
            //Torque - from horizonal stabilizer and elevator
            pitchTorque = Math.sqrt((dragTorque*dragTorque)+(liftTorque*liftTorque));
            if((AoADegrees + elevatorP) > 0) pitchTorque = Math.abs(pitchTorque) * -1;
            //Angular acceleration
            alphaAoA = pitchTorque/Iy; //angular acceleration
            ux = -a12; uy = -a22; uz = -a32; //defines the unit vector for pitch
            //Calculate the angle to yaw through. Then call the rotation matrix with
            //this value.
            rotate = (omegaAoA * deltaT) + (0.5 * alphaAoA * deltaT * deltaT);
            //Call the rotation method. This will define the new aircraft orientation.
            R(rotate);  //R() -> B() -> redefineA()
            bodyRatePitch = (rotate/deltaT)*180/Math.PI;
            //BodyRatePitch = String.valueOf(bodyRatePitch); 
            //if (bodyRatePitch < 0.0000001 && bodyRatePitch > -0.0000001) BodyRatePitch = "0.0";
            //if (BodyRatePitch.length() > 7) BodyRatePitch = BodyRatePitch.substring(0,7);
            //angular velocity = omega
            omegaAoA = omegaAoA + (alphaAoA * deltaT);
            if (AoA < 0.001 && AoA > -0.001 && omegaAoA < 0.04){
                omegaAoA = omegaAoA* 0.9;
            }
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            //Define forces. Thrust, Lift, Drag
            forces();

            //calculateG();

            queues();
            //The queues are present to smooth out the output to the instruments.
            //i.e, sim2 becomes erratic when controls are input. 
            //Without the queues the PFD and compass appear to jerk and bounce.

            alphaQ = Math.atan2(a21q, a11q);
            betaQ = Math.atan2(-a31q, (Math.sqrt((a32q*a32q) + (a33q*a33q))));
            gammaQ = Math.atan2(a32q,-a33q);

            Alpha = String.valueOf(alphaQ);
            Beta = String.valueOf(betaQ);
            Gamma = String.valueOf(gammaQ);

            if (Alpha.length() > 5) Alpha = Alpha.substring(0,5);
            if (Beta.length() > 5) Beta = Beta.substring(0,5);
            if (Gamma.length() > 5) Gamma = Gamma.substring(0,5);

            //alphaQ = Math.atan2(a21, a11);
            betaT = Math.atan2(-a31, (Math.sqrt((a32*a32) + (a33*a33))));
            gammaT = Math.atan2(a32,-a33);

            airspeed();

            RollMode = String.valueOf(rollMode);
            if (rollMode == 1){
                RollMode = RollMode + "Hand fly";
                roll();
                }
                
            if (rollMode == 2){
                RollMode = RollMode + "Autopilot";
                roll2();
            }
                

            Thread.currentThread();
            try {
                Thread.sleep(1);
            } catch (Exception e) {e.printStackTrace();}
        }
    }

    public void queues(){
        if (a11Q.size() >= 100){ a11Q.remove();}
        a11Q.add(a11);
        a11q = 0;      for(double e : a11Q){a11q = a11q + e;}
        a11q = a11q/a11Q.size();

        if (a12Q.size() >= 100){ a12Q.remove();}
        a12Q.add(a12);
        a12q = 0;      for(double e : a12Q){a12q = a12q + e;}
        a12q = a12q/a12Q.size();
        
        if (a13Q.size() >= 100){ a13Q.remove();}
        a13Q.add(a13);
        a13q = 0;      for(double e : a13Q){a13q = a13q + e;}
        a13q = a13q/a13Q.size();
        
        if (a21Q.size() >= 100){ a21Q.remove();}
        a21Q.add(a21);
        a21q = 0;      for(double e : a21Q){a21q = a21q + e;}
        a21q = a21q/a21Q.size();
        
        if (a22Q.size() >= 100){ a22Q.remove();}
        a22Q.add(a22);
        a22q = 0;      for(double e : a22Q){a22q = a22q + e;}
        a22q = a22q/a22Q.size();
        
         if (a23Q.size() >= 100){ a23Q.remove();}
        a23Q.add(a23);
        a23q = 0;      for(double e : a23Q){a23q = a23q + e;}
        a23q = a23q/a23Q.size();

        if (a31Q.size() >= 100){ a31Q.remove();}
        a31Q.add(a31);
        a31q = 0;      for(double e : a31Q){a31q = a31q + e;}
        a31q = a31q/a31Q.size();

        if (a32Q.size() >= 100){ a32Q.remove();}
        a32Q.add(a32);
        a32q = 0;      for(double e : a32Q){a32q = a32q + e;}
        a32q = a32q/a32Q.size();

        if (a33Q.size() >= 100){ a33Q.remove();}    
        a33Q.add(a33);
        a33q = 0;      for(double e : a33Q){a33q = a33q + e;}
        a33q = a33q/a33Q.size();

        if (AoAQ.size() >= 50){ AoAQ.remove();}    
        AoAQ.add(AoA);
        dummyq = 0;      for(double e : AoAQ){dummyq = dummyq + e;}
        dummyq = (dummyq/AoAQ.size())*180/Math.PI;
        AoAq = dummyq;

        if (yawQ.size() >= 100){ yawQ.remove();}    
        yawQ.add(bodyRateYaw);
        yawq = 0;      for(double e : yawQ){yawq = yawq + e;}
        yawq = (yawq/yawQ.size());
        BodyRateYaw = String.valueOf(yawq); 
        if (yawq < 0.0000001 && yawq > -0.0000001) BodyRateYaw = "0.0";
        if (BodyRateYaw.length() > 5) BodyRateYaw = BodyRateYaw.substring(0,5);

        if (pitchQ.size() >= 100){ pitchQ.remove();}    
        pitchQ.add(bodyRatePitch);
        pitchq = 0;      for(double e : pitchQ){pitchq = pitchq + e;}
        pitchq = (pitchq/pitchQ.size());
        BodyRatePitch = String.valueOf(pitchq); 
        if (pitchq < 0.0000001 && pitchq > -0.0000001) BodyRatePitch = "0.0";
        if (BodyRatePitch.length() > 5) BodyRatePitch = BodyRatePitch.substring(0,5);

        if (rollQ.size() >= 100){ rollQ.remove();}    
        rollQ.add(bodyRateRoll);
        rollq = 0;      for(double e : rollQ){rollq = rollq + e;}
        rollq = (rollq/rollQ.size());
        BodyRateRoll = String.valueOf(rollq); 
        if (rollq < 0.0000001 && rollq > -0.0000001) BodyRateRoll = "0.0";
        if (BodyRateRoll.length() > 5) BodyRateRoll = BodyRateRoll.substring(0,5);

        if (GQ.size() >= 100){ GQ.remove();}    
        GQ.add(G);
        dummyq = 0;      for(double e : GQ){dummyq = dummyq + e;}
        dummyq = (dummyq/GQ.size());  gQ = dummyq;
        gString = String.valueOf(gQ+0.05); 
        if (gQ < 0.0000001 && gQ > -0.0000001) gString = "0.0";
        if (gQ >= 0 && gString.length() > 3) gString = gString.substring(0,3);
        if (gQ < 0 && gString.length() > 4) gString = gString.substring(0,4);

        if (velZQ.size() >= 100){ velZQ.remove();}    
        velZQ.add(velZ);
        dummyq = 0;      for(double e : velZQ){dummyq = dummyq + e;}
        dummyq = (dummyq/rollQ.size());  velZq = dummyq;

        if (thrustZQ.size() >= 100){ thrustZQ.remove();}    
        thrustZQ.add(thrustZ);
        dummyq = 0;      for(double e : thrustZQ){dummyq = dummyq + e;}
        dummyq = (dummyq/thrustZQ.size());  thrustZq = dummyq;
    }

    public void forces(){
        thrust = simGraphics1.actThrust * 600; //max thrust = 60000N
        thrustX = thrust * a11; thrustY = thrust * a21; thrustZ = thrust * a31;
        ThrustX = String.valueOf(thrustX); if (ThrustX.length() > 7) ThrustX = ThrustX.substring(0,5);
        ThrustY = String.valueOf(thrustY); if (ThrustY.length() > 7) ThrustY = ThrustY.substring(0,5);
        ThrustZ = String.valueOf(thrustZ); if (ThrustZ.length() > 7) ThrustZ = ThrustZ.substring(0,5);

        speed = Math.sqrt((velX*velX)+(velY*velY)+(velZ*velZ));

        liftCoefficient = liftC(AoADegrees);
        liftVector(); //define the direction lift is acting in. 90 degrees to RAF and wings.
        liftWingTail = liftCoefficient * speed * speed * (Sw + St); //wing & tail
        liftwtX = liftWingTail * liftX;
        liftwtY = liftWingTail * liftY;
        liftwtZ = liftWingTail * liftZ;

        LiftwtX = String.valueOf(liftwtX); if (LiftwtX.length() > 7) LiftwtX = LiftwtX.substring(0,7);
        LiftwtY = String.valueOf(liftwtY); if (LiftwtY.length() > 7) LiftwtY = LiftwtY.substring(0,7);
        LiftwtZ = String.valueOf(liftwtZ); if (LiftwtZ.length() > 7) LiftwtZ = LiftwtZ.substring(0,7);

        dragCoefficient = Math.abs(dragC(AoADegrees));
        dragWingTail = dragCoefficient * speed * speed * (Sw + St); //wing
        dragwtX = dragWingTail * -velX/speed; //in the direction opposity velocity
        dragwtY = dragWingTail * -velY/speed; //in the direction opposity velocity 
        dragwtZ = dragWingTail * -velZ/speed; //in the direction opposity velocity
        DragwtX = String.valueOf(dragwtX); if (DragwtX.length() > 7) DragwtX = DragwtX.substring(0,7);
        DragwtY = String.valueOf(dragwtY); if (DragwtY.length() > 7) DragwtY = DragwtY.substring(0,7);
        DragwtZ = String.valueOf(dragwtZ); if (DragwtZ.length() > 7) DragwtZ = DragwtZ.substring(0,7);

        //lift coeffienct must come from slideslip
        liftCoefficient = liftC(sideSlipDegrees); //have you considered drag from sideSlip?
        sideBodyForceDirection(); 
        liftSideTotal = liftCoefficient * speed * speed * fuselageSideArea;
        sideLiftForceX = -liftSideTotal * sideLiftX;
        sideLiftForceY = -liftSideTotal * sideLiftY;
        sideLiftForceZ = -liftSideTotal * sideLiftZ;
        SideLiftForceX = String.valueOf(sideLiftForceX);
        if (SideLiftForceX.length() > 7) SideLiftForceX = SideLiftForceX.substring(0,7);
        SideLiftForceY = String.valueOf(sideLiftForceY);
        if (SideLiftForceY.length() > 7) SideLiftForceY = SideLiftForceY.substring(0,7);
        SideLiftForceZ = String.valueOf(sideLiftForceZ);
        if (SideLiftForceZ.length() > 7) SideLiftForceZ = SideLiftForceZ.substring(0,7);

        if(!simGraphics1.gearUp){
            bodyArea = 0.50;
        }
        else {
            bodyArea = 0.25;
        }
        
        bodyDX = bodyArea*velX*velX; if (velX > 0) bodyDX = -bodyDX;
        bodyDY = bodyArea*velY*velY; if (velY > 0) bodyDY = -bodyDY;
        bodyDZ = bodyArea*velZ*velZ; if (velZ > 0) bodyDZ = -bodyDZ;
        BodyDX = String.valueOf(bodyDX); if (BodyDX.length() > 7) BodyDX = BodyDX.substring(0,7);
        BodyDY = String.valueOf(bodyDY); if (BodyDY.length() > 7) BodyDY = BodyDY.substring(0,7);
        BodyDZ = String.valueOf(bodyDZ); if (BodyDZ.length() > 7) BodyDZ = BodyDZ.substring(0,7);

        groundSpeed = Math.sqrt((velX*velX)+(velY*velY))*1.94384;
        GroundSpeed = String.valueOf(groundSpeed); 
        GroundSpeed = GroundSpeed.substring(0,3);
        if (groundSpeed < 100) GroundSpeed = GroundSpeed.substring(0,2);
        if (groundSpeed < 10) GroundSpeed = GroundSpeed.substring(0,1);
        
        landingSuccessful = false;
        if(betaQ >= -2*Math.PI/180 && betaQ <= 10*Math.PI/180){ //nose attitude -2 to 10 degrees nose up
            if(gammaQ >= -5*Math.PI/180 && gammaQ <= 5*Math.PI/180){ //angle of bank < 2 degrees
                if(velZ < 20.0/3.2808){ //20 feet per second = 1,200 feet per minute
                    landingSuccessful = true;
                }
            }            
        }
        
        if(simGraphics1.gearDn){
            wheelsDownHeight = -3.0;
        }
        else{
            wheelsDownHeight = 0.0;
        }
        
        if( positionZ >= wheelsDownHeight){
            positionZ = wheelsDownHeight;
            velZ = 0;
            velZq = 0;

            if(gammaT > 0.25*Math.PI/180 || gammaT < -0.25*Math.PI/180){
                ux = a11; uy = a21; uz = a31; //defines the unit vector for roll
                R(-gammaT); //to roll to wings level on ground
            }

            if(betaT < 0){
                ux = -a12; uy = -a22; uz = -a32; //defines the unit vector for pitch
                R(betaT);
            }

            if (!landingSuccessful) {   //aircraft on ground. Zero altitude
                simGraphics1.takeoffNorthReset();
            }
            if(landingSuccessful){
                if (!landed){
                    simGraphics1.setLandingOff();
                    landed = true;
                }

                if(thrust <= 0){
                    if (groundSpeed > 0.01){
                        if (bodyDX > 0){bodyDX = bodyDX - (40000*velX/groundSpeed);}
                        else {bodyDX = bodyDX - (40000*velX/groundSpeed);}
                        if (bodyDY > 0){bodyDY = bodyDY - (40000*velY/groundSpeed);}
                        else {bodyDY = bodyDY - (40000*velY/groundSpeed);}
                    }
                }
            }
        }
        if (positionZ < wheelsDownHeight) landed = false;
        

        forceX = thrustX + liftwtX + dragwtX + bodyDX + sideLiftForceX;
        forceY = thrustY + liftwtY + dragwtY + bodyDY + sideLiftForceY;
        forceZ = thrustZ + liftwtZ + dragwtZ  + bodyDZ + sideLiftForceZ + (gravityZ*aircraftMass);

        if (positionZ == wheelsDownHeight){
            if (forceZ >= 0){forceZ = 0;}
        }

        ForceX = String.valueOf(forceX); if (ForceX.length() > 7) ForceX = ForceX.substring(0,7);
        ForceY = String.valueOf(forceY); if (ForceY.length() > 7) ForceY = ForceY.substring(0,7);
        ForceZ = String.valueOf(forceZ); if (ForceZ.length() > 7) ForceZ = ForceZ.substring(0,7);

        accelerationX = forceX/aircraftMass;
        accelerationY = forceY/aircraftMass;
        accelerationZ = forceZ/aircraftMass;

        //positions for G calculation
        gX = -accelerationX;
        gY = -accelerationY;        
        gZ = -accelerationZ + gravityZ;
        G = Math.sqrt((gX*gX)+(gY*gY)+(gZ*gZ))/gravityZ;
        gPosNeg = (gX*a13)+(gY*a23)+(gZ*a33);
        if(gPosNeg > 0) G = G * -1;
        //gString = String.valueOf(G); if (gString.length() > 4) gString = gString.substring(0,4);
        calcBalance();

        //now update position
        positionX = positionX + (velX*deltaT) + (accelerationX*deltaT*deltaT/2);
        positionY = positionY + (velY*deltaT) + (accelerationY*deltaT*deltaT/2);
        positionZ = positionZ + (velZ*deltaT) + (accelerationZ*deltaT*deltaT/2);
        PositionX = String.valueOf(positionX); if (PositionX.length() > 7) PositionX = PositionX.substring(0,7);
        PositionY = String.valueOf(positionY); if (PositionY.length() > 7) PositionY = PositionY.substring(0,7);
        PositionZ = String.valueOf(positionZ); if (PositionZ.length() > 7) PositionZ = PositionZ.substring(0,7);
        altitudeFeet = positionZ * -3.28084;

        velX = velX + (accelerationX*deltaT);
        velY = velY + (accelerationY*deltaT);
        velZ = velZ + (accelerationZ*deltaT);
        VelX = String.valueOf((int)velX); //if (VelX.length() > 7) VelX = VelX.substring(0,7);
        VelX = "          ".substring(VelX.length())+VelX;
        VelY = String.valueOf((int)velY); //if (VelY.length() > 7) VelY = VelY.substring(0,7);
        VelY = "          ".substring(VelY.length())+VelY;
        VelZ =  String.format( "%.2f", velZ );//if (VelZ.length() > 7) VelZ = VelZ.substring(0,7);
        VelZ = "          ".substring(10-VelZ.length())+VelZ;
        
        velocityVector = Math.atan(velZ/Math.sqrt((velX*velX)+(velY*velY)));
        //String.format( "%.2f", velZ ); 
        //String.valueOf((int)velZ);
    }


    
    public void liftVector(){
        //cross product: velocityVector x wingVector
        liftX = (velY*a32) - (velZ*a22);
        liftY = (velZ*a12) - (velX*a32);
        liftZ = (velX*a22) - (velY*a12);
        //normalise the vector
        liftVectorTotal = Math.sqrt((liftX*liftX)+(liftY*liftY)+(liftZ*liftZ));
        liftX = liftX/liftVectorTotal;
        liftY = liftY/liftVectorTotal;
        liftZ = liftZ/liftVectorTotal;
    }

    public void sideBodyForceDirection(){
        //cross product: velocityVector x tailVector
        sideLiftX = (velY*a33) - (velZ*a23);
        sideLiftY = (velZ*a13) - (velX*a33);
        sideLiftZ = (velX*a23) - (velY*a13);
        //normalise the vector
        sideLiftVectorTotal = Math.sqrt((sideLiftX*sideLiftX)+(sideLiftY*sideLiftY)+(sideLiftZ*sideLiftZ));
        sideLiftX = sideLiftX/sideLiftVectorTotal;
        sideLiftY = sideLiftY/sideLiftVectorTotal;
        sideLiftZ = sideLiftZ/sideLiftVectorTotal;
    }

    //rotational matrix
    /**
     * R matrix is the rotation matrix.
     * ux, uy, uz are the unit vectors.
     */
    public static void R(double angle){
        //Define R, the rotation matrix
        r11 = Math.cos(angle) + ((ux*ux)*(1.0-Math.cos(angle)));
        r12 = ((ux*uy)*(1.0-Math.cos(angle))-(uz*Math.sin(angle)));
        r13 = ((ux*uz)*(1.0-Math.cos(angle))+(uy*Math.sin(angle)));

        r21 = ((ux*uy)*(1.0-Math.cos(angle))+(uz*Math.sin(angle)));
        r22 = Math.cos(angle) + ((uy*uy)*(1.0-Math.cos(angle)));
        r23 = ((uy*uz)*(1.0-Math.cos(angle))-(ux*Math.sin(angle)));

        r31 = ((ux*uz)*(1.0-Math.cos(angle))-(uy*Math.sin(angle)));
        r32 = ((uz*uy)*(1.0-Math.cos(angle))+(ux*Math.sin(angle)));
        r33 = Math.cos(angle) + ((uz*uz)*(1.0-Math.cos(angle)));

        B();
    }

    /**
     * RA = B
     */
    public static void B(){
        // RA = B
        b11 = (r11*a11)+(r12*a21)+(r13*a31);b12 = (r11*a12)+(r12*a22)+(r13*a32);b13 = (r11*a13)+(r12*a23)+(r13*a33);
        b21 = (r21*a11)+(r22*a21)+(r23*a31);b22 = (r21*a12)+(r22*a22)+(r23*a32);b23 = (r21*a13)+(r22*a23)+(r23*a33);
        b31 = (r31*a11)+(r32*a21)+(r33*a31);b32 = (r31*a12)+(r32*a22)+(r33*a32);b33 = (r31*a13)+(r32*a23)+(r33*a33);

        redefineA();
    }

    /**
     * Redefines the matrix A and matrix B and the completion of the manouvre.
     */
    public static void redefineA(){
        a11 = b11; a12 = b12; a13 = b13;
        a21 = b21; a22 = b22; a23 = b23;
        a31 = b31; a32 = b32; a33 = b33;

        //the following ifs are to prevent very small magnitude numbers from appearing
        //as large numbers on the graphics pane. The ifs only affect B matrix after
        //A has been redefined.
        if (b11 < 0.0000001 && b11 > -0.0000001) b11 = 0;
        if (b12 < 0.0000001 && b12 > -0.0000001) b12 = 0;
        if (b13 < 0.0000001 && b13 > -0.0000001) b13 = 0;
        if (b21 < 0.0000001 && b21 > -0.0000001) b21 = 0;
        if (b22 < 0.0000001 && b22 > -0.0000001) b22 = 0;
        if (b23 < 0.0000001 && b23 > -0.0000001) b23 = 0;
        if (b31 < 0.0000001 && b31 > -0.0000001) b31 = 0;
        if (b32 < 0.0000001 && b32 > -0.0000001) b32 = 0;
        if (b33 < 0.0000001 && b33 > -0.0000001) b33 = 0;

        A11 = String.valueOf(b11); if (A11.length() > 5) A11 = A11.substring(0,5);
        A12 = String.valueOf(b12); if (A12.length() > 5) A12 = A12.substring(0,5);
        A13 = String.valueOf(b13); if (A13.length() > 5) A13 = A13.substring(0,5);
        A21 = String.valueOf(b21); if (A21.length() > 5) A21 = A21.substring(0,5);
        A22 = String.valueOf(b22); if (A22.length() > 5) A22 = A22.substring(0,5);
        A23 = String.valueOf(b23); if (A23.length() > 5) A23 = A23.substring(0,5);
        A31 = String.valueOf(b31); if (A31.length() > 5) A31 = A31.substring(0,5);
        A32 = String.valueOf(b32); if (A32.length() > 5) A32 = A32.substring(0,5);
        A33 = String.valueOf(b33); if (A33.length() > 5) A33 = A33.substring(0,5);
    }

    public double liftC(double t){ //Cl is a function of slidSlip/AoA
        //if (t < 0) t = Math.abs(t);
        if (t > 30 && t <= 90){
            Cl = (-0.005*t)+0.45;} 
        if (t > 15 && t <= 30) {
            Cl = (-0.7/15*t)+1.7;}
        if (t >= -15 && t <= 15) {
            Cl = t/15;}
        if (t >= -30 && t < -15){
            Cl = (-0.7/15*t)-1.7;}
        if (t < -30 && t >= -90){
            Cl = (-0.005)*t-0.45;}

        if (simGraphics1.spoilersOn){Cl = Cl * 0.9;}
        return Cl;
    }

    public double dragC(double t){  //Cd is a function of slidSlip angle.
        //if (t < 0) t = Math.abs(t);
        if (t >= -90 && t <= 90) { Cd = t/90; }

        if (simGraphics1.spoilersOn){Cd = Cd * 1.2;}
        return Cd;
    }

    public static void calcSideSlip(){
        //First, project the velocity vector onto the x,y plane. 
        //The resulting vector is 'w'.
        //Then, determine the angle w makes with the x,z plane. 
        //This angle is the slidslip.
        //matrix defining aircraft attitude
        //double a11, a12, a13, a21, a22, a23, a31, a32, a33;
        //projection vector 
        //double wX, wY, wZ;
        //wbar = vbar - (vbar dot zbar)*zbar, where zbar is the vector normal to the x,y axis
        //determine wbar
        wX = velX - (((velX*a13)+(velY*a23)+(velZ*a33))*a13);
        wY = velY - (((velX*a13)+(velY*a23)+(velZ*a33))*a23);
        wZ = velZ - (((velX*a13)+(velY*a23)+(velZ*a33))*a33);
        //now find the angle between wbar and the x,z plane
        //now ybar is the normal to the x,z plane
        //90degrees - arccos((wbar dot ybar)/(||wbar|| * ||ybar||))
        //dot product        
        dot = ((wX*a12)+(wY*a22)+(wZ*a32));
        //||wbar|| * ||ybar||  
        norm = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ)) * Math.sqrt((a12*a12) + (a22*a22) + (a32*a32));
        sideSlip = (Math.PI/2) - Math.acos(dot/norm);

        ssVel = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ));
        sideSlipDegrees = sideSlip * (180/Math.PI);
        SideSlipDegreesString = String.valueOf(sideSlipDegrees); 
        if (SideSlipDegreesString.length() > 5) SideSlipDegreesString = SideSlipDegreesString.substring(0,5);
        if (sideSlipDegrees < 0.0000001 && sideSlipDegrees > -0.0000001) SideSlipDegreesString = "0.00";
    }

    public static void calcAoA(){
        //First, project the velocity vector onto the x,z plane. 
        //The resulting vector is 'w'.
        //Then, determine the angle w makes with the x,y plane. 
        //This angle is the AoA.
        //projection vector 
        //double wX, wY, wZ;
        //wbar = vbar - (vbar dot ybar)*ybar, where ybar is the vector normal to the x,z axis
        //determine wbar
        wX = velX - (((velX*a12)+(velY*a22)+(velZ*a32))*a12);
        wY = velY - (((velX*a12)+(velY*a22)+(velZ*a32))*a22);
        wZ = velZ - (((velX*a12)+(velY*a22)+(velZ*a32))*a32);
        //now find the angle between wbar and the x,y plane
        //now zbar is the normal to the x,y plane
        //90degrees - arccos((wbar dot zbar)/(||wbar|| * ||zbar||))
        //dot product        
        dot = ((wX*a13)+(wY*a23)+(wZ*a33));
        //||wbar|| * ||zbar||  
        norm = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ)) * Math.sqrt((a13*a13) + (a23*a23) + (a33*a33));
        AoA = (Math.PI/2) - Math.acos(dot/norm);

        AoAVel = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ));
        AoADegrees = AoA * (180/Math.PI);
        AOADegrees = String.valueOf(AoADegrees); 
        if (AOADegrees.length() > 5) AOADegrees = AOADegrees.substring(0,5);
        if (AoADegrees < 0.0000001 && AoADegrees > -0.0000001) AOADegrees = "0.00";
    }

    public void calcBalance(){
        //First, project the G vector onto the y,z plane. 
        //The resulting vector is 'w'.
        //Then, determine the angle w makes with the x,z plane. or just the aircraft's
        //z vector
        //This angle is the balance angle.
        //projection vector 

        wX = gX - (((gX*a11)+(gY*a21)+(gZ*a31))*a11);
        wY = gY - (((gX*a11)+(gY*a21)+(gZ*a31))*a21);
        wZ = gZ - (((gX*a11)+(gY*a21)+(gZ*a31))*a31);
        dot = ((wX*a13)+(wY*a23)+(wZ*a33));
        balCor = ((wX*a12)+(wY*a22)+(wZ*a32)); //projection onto aircraft y vector

        norm = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ)) * Math.sqrt((a13*a13) + (a23*a23) + (a33*a33));
        balance = (Math.PI/2) - Math.acos(dot/norm);

    }

    public void roll2(){
        //aileron right, aileron position +ve. Produces roll right.
        //Therefore, left aileron down to increase AoA on left side.
        //Right aileron up to reduce AoA on the right side.
        wingSpan = 600;
        //roll right, roll rate +ve.
        //roll left, roll rate -ve.

        leftAileronPosition = -simGraphics1.ailPos/2.0; //negative -> aileron below wing
        rightAileronPosition = -leftAileronPosition;

        //calculate right wing tip velocity 
        //v = 2 * pi * r * omega.rad
        tipX = velX - (a31 * 2 * Math.PI * rollRate * wingSpan);//tipX = wing tip velocity in X direction
        tipY = velY - (a32 * 2 * Math.PI * rollRate * wingSpan);
        tipZ = velZ - (a33 * 2 * Math.PI * rollRate * wingSpan);
        rightAoA = calcWingTipAoA(); //angle of attack at wing tip. i.e. aileron position
        rightWingAilLiftC = liftC(rightAoA + rightAileronPosition);
        //wingSpan = 1;
        rollTorqueR = rightWingAilLiftC * 1 *  ias * aileronSize;

        //calculate left wing tip velocity 
        tipX = velX + (a31 *  2 * Math.PI * rollRate * wingSpan);//tipX = wing tip velocity in X direction
        tipY = velY + (a32 *  2 * Math.PI * rollRate * wingSpan);
        tipZ = velZ + (a33 *  2 * Math.PI * rollRate * wingSpan);
        leftAoA = calcWingTipAoA();
        leftWingAilLiftC = liftC(leftAoA + leftAileronPosition);
        rollTorqueL = leftWingAilLiftC * 1 * ias * aileronSize;

        rollTorque = rollTorqueR - rollTorqueL;

        //rollAccel = rollTorque/Ix;
        rollAccel = rollTorque/20000;

        rollRate = rollRate + rollAccel;
        ux = a11; uy = a21; uz = a31; //defines the unit vector for roll
        R(rollRate);
        bodyRateRoll = (rollRate/deltaT)*180/Math.PI;
    }

    public static double calcWingTipAoA(){
        //First, project the velocity vector onto the x,z plane. 
        //The resulting vector is 'w'.
        //Then, determine the angle w makes with the x,y plane. 
        //This angle is the AoA.
        //projection vector 
        //double wX, wY, wZ;
        //wbar = vbar - (vbar dot ybar)*ybar, where ybar is the vector normal to the x,z axis
        //determine wbar
        wX = tipX - (((tipX*a12)+(tipY*a22)+(tipZ*a32))*a12);
        wY = tipY - (((tipX*a12)+(tipY*a22)+(tipZ*a32))*a22);
        wZ = tipZ - (((tipX*a12)+(tipY*a22)+(tipZ*a32))*a32);
        //now find the angle between wbar and the x,y plane
        //now zbar is the normal to the x,y plane
        //90degrees - arccos((wbar dot zbar)/(||wbar|| * ||zbar||))
        //dot product        
        dot = ((wX*a13)+(wY*a23)+(wZ*a33));
        //||wbar|| * ||zbar||  
        norm = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ)) * Math.sqrt((a13*a13) + (a23*a23) + (a33*a33));
        tipAoA = (Math.PI/2) - Math.acos(dot/norm);
        tipAoADegrees = tipAoA * (180/Math.PI);

        return tipAoADegrees;
        //AoAVel = Math.sqrt((wX*wX) + (wY*wY) + (wZ*wZ));
    }

    public void roll(){
        aileronP = simGraphics1.ailPos/2000;
        rollTorque = aileronP * wingSpan;
        if (ias < 50){
            rollTorque = rollTorque * (ias/50);
        }

        //rollTorque = aileronP * ((1.0 - Math.exp(-ias/25.0)) * wingSpan);
        rollAccel = rollTorque/Ix;
        rollRate = rollRate + rollAccel;
        if (rollRate > 0){
            rollRate = Math.min(0.02, rollRate);
        }
        if (rollRate < 0){
            rollRate = Math.max(-0.02, rollRate);
        }
        if (aileronP == 0 && rollRate > 0 && rollRate < 0.005){
            rollRate = Math.max(0, (rollRate - 0.00002));
        }
        if (aileronP == 0 && rollRate > 0.005){
            rollRate = Math.max(0, (rollRate - 0.00005));
        }
        if (aileronP == 0 && rollRate < 0 && rollRate > -0.005){
            rollRate = Math.min(0, (rollRate + 0.00002));
        }
        if (aileronP == 0 && rollRate < -0.005){
            rollRate = Math.min(0, (rollRate + 0.00005));
        }
        ux = a11; uy = a21; uz = a31; //defines the unit vector for roll
        R(rollRate);
        bodyRateRoll = (rollRate/deltaT)*180/Math.PI;
    }

    public static void rollModeSelect(){
        if (rollMode == 1) {rollMode = 2; return;}
        rollMode = 1;
    }

    public void airspeed(){ //dot product
        ias = ((a11 * velX) + (a21 * velY) + (a31 * velZ));
        ias = Math.max(ias, 0);
        iasKnots = ias * 1.94384;
    }
    //public void setLandedFalse(){
    //    landed = false;
    //}
}
