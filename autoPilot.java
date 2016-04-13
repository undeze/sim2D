package sim2D;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;
/** 
 * Chris Underwood Sept 2014
 */
public class autoPilot implements Runnable
{
    // instance variables - replace the example below with your own
    private Thread runner;
    private double pVal, iVal, dVal;
    private static double currVal, error, target, lastError, cumError, maxCor, minCor, slope;
    private double iCorrection, pCorrection, dCorrection, correction;
    private double lastBetaI; //pitch alphaQ, betaQ, gammaQ

    private double pAttitude;
    static double currAtt, attError, targetAtt;
    private double pitchRateReq, attCor;
    private static double currPitchRate;
    private static int attHoldMode = 3;
    private static int altHoldMode = 3;
    static String ATTHoldMode, ALTHoldMode;

    private double pAltitude;
    private static double currAlt, altError, targetAlt;
    static double currClimbRate, climbRateReq, changeAtt;

    private double p, b, gz, tz;

    static String ClimbRateReq;

    Queue<Double> AoBQ = new LinkedList<Double>();
    static double aobq;
    Queue<Double> AttitudeQ = new LinkedList<Double>();
    static double attitudeq;
    static double dummyq;
    static double targetAoB, targetRollRate; 
    static double reqdHDG;
    static double currentHDG;
    private static double cHmrH, rHmcH;
    static String turnDirection;
    static double aobCor, s, sLAtt;

    static boolean APON, ATHON;
    static double alt2Go , clbDesAttCor, clbDesAtt, sCor, zeroPitchElePos;
    static boolean CLB, DES, LVL;
    static double sAPstn, sAPstnSmall;


    /**
     * Constructor for objects of class autoPilot
     */
    public autoPilot()
    {
        // initialise instance variables
        ClimbRateReq = "0";

        pVal = 5;         iVal = 0.01;         dVal = 0;
        target = 200; //knots

        cumError = 0;

        //attitude hold values

        targetAtt = 5*Math.PI/180;

        pAltitude = 0.1;

        ATTHoldMode = String.valueOf(attHoldMode);
        ALTHoldMode = String.valueOf(altHoldMode);

        ATHON = true;

        if (runner == null) {
            runner = new Thread(this);
            runner.start();
        }
    }

    public void run(){
        while(true){

            APON = simGraphics1.apOn;
            //autoThrust Speed
            if (simGraphics1.speedOn){
                target = simGraphics1.mouseWheelSpeed;
                currVal = sim2.iasKnots;
                error = target - currVal;
                pCorrection = pVal * error;
                if (sim2.iasKnots < (target - 2) && cumError < 0) cumError = 0;
                if (sim2.iasKnots > (target + 2) && cumError > 0) cumError = 0;
                cumError = cumError + error;
                iCorrection = iVal * cumError;

                lastError = error;
                correction = pCorrection + iCorrection;// + dCorrection;

                minCor = (2 * (sim2.betaQ*180/Math.PI) + (0.1*target)) - 5.0;
                maxCor = (2 * (sim2.betaQ*180/Math.PI) + (0.1*target)) + 20.0;
                correction = Math.max(correction, minCor);
                correction = Math.min(correction, maxCor);
                correction = Math.max(correction, 15);
                correction = Math.min(correction, 100);

                simGraphics1.setTargetThrust(correction);
            }

            //Attitude Hold
            if (simGraphics1.attitudeOn){
                targetAtt = simGraphics1.mouseWheelAttitude*Math.PI/180;
                currAtt = sim2.betaQ;     // radians
                attError = targetAtt - currAtt; // radians

                p = sim2.betaQ; b = sim2.gammaQ;

                if(attHoldMode==1){
                    if (AttitudeQ.size() >= 500){ AttitudeQ.remove();}    
                    AttitudeQ.add(sim2.betaQ);
                    dummyq = 0;      for(double e : AttitudeQ){dummyq = dummyq + e;}
                    dummyq = (dummyq/AttitudeQ.size());  attitudeq = dummyq;

                    pAttitude = 3.0/(Math.cos(b)*Math.cos(b)*Math.cos(b));
                    pitchRateReq = (-pAttitude * attError*(180/Math.PI));
                    currPitchRate = sim2.bodyRatePitch; //degrees per second
                    if (pitchRateReq < currPitchRate) attCor = -0.01;
                    if (pitchRateReq > currPitchRate) attCor = 0.01;
                    if (APON)
                        simGraphics1.setElevatorPosition(attCor);
                }

                if(attHoldMode==2){
                    gz = sim2.gravityZ*sim2.aircraftMass;
                    tz = sim2.thrustZq;
                    pAttitude = ((gz/tz)*Math.sin(b)*Math.sin(b)*Math.cos(p)*Math.cos(p)/Math.cos(b));
                    pitchRateReq = (pAttitude * attError*(180/Math.PI)); // pitch rate required
                    currPitchRate = sim2.bodyRatePitch; //degrees per second
                    if (pitchRateReq < currPitchRate) attCor = -0.01;
                    if (pitchRateReq > currPitchRate) attCor = 0.01;
                    if (APON)
                        simGraphics1.setElevatorPosition(attCor);
                }

                //pitchRateReq = Math.max(-3, pitchRateReq);
                //pitchRateReq = Math.min(3, pitchRateReq);

                if (simGraphics1.glideOn) {
                    targetAtt = -10.0*Math.PI/180;
                    if(sim2.iasKnots > 215){targetAtt = -2.0*Math.PI/180;}
                    if(sim2.iasKnots < 195){targetAtt = -12.0*Math.PI/180;}
                    if(sim2.iasKnots < 190){targetAtt = -15.0*Math.PI/180;}
                    if(sim2.gammaQ > 25*Math.PI/180 || sim2.gammaQ < -25*Math.PI/180){
                        targetAtt = targetAtt -1.0*Math.PI/180;
                    }
                    //simGraphics1.setMouseWheelHDG();
                }

                if(attHoldMode==3){
                    if (targetAtt >= 15.0*Math.PI/180 ) {targetAtt = targetAtt - 0.5*Math.PI/180;} //correction
                    aobCor = (0.7096*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ)+(0.535*sim2.gammaQ*sim2.gammaQ);
                    //s = simGraphics1.mouseWheelSpeed;//
                    s = sim2.iasKnots;
                    sCor = (-0.00000000699999*s*s*s*s)+(0.0000066999*s*s*s)+(-0.0024324999*s*s)+(0.40574999*s)-27.64999;
                    sCor = sCor-((sCor*100)%5)/100;  

                    if (s > 300 && s <= 310) sCor = -0.65;
                    if (s > 310 && s <= 330) sCor = -0.60;
                    if (s > 330 && s <= 340) sCor = -0.55;
                    if (s > 340 && s <= 360) sCor = -0.50;
                    if (s > 360 && s <= 380) sCor = -0.45;
                    if (s > 380 && s <= 395) sCor = -0.40;
                    if (s > 395 ) sCor = -0.40;

                    // if the attitude is between -0.3 degrees and 0.3 degrees of targetAtt,
                    // no elevator correction will be applied.

                    if (APON){
                        if(sim2.betaQ < targetAtt + (-0.3*Math.PI/360) && sim2.betaQ > targetAtt + -1*Math.PI/360){
                            correction = sCor;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        if(sim2.betaQ < targetAtt + -1*Math.PI/360){
                            correction = sCor - 0.1;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        //Attitude Hold
                        if(sim2.betaQ > targetAtt + 0.3*Math.PI/360 && sim2.betaQ < targetAtt + Math.PI/360){
                            correction = sCor + 0.05;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        if (sim2.betaQ > targetAtt + Math.PI/360){
                            correction = sCor + 0.2;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                    }
                }//if(attHoldMode==3){
            } //Attitude Hold

            //altitude height hold
            if (simGraphics1.altitudeOn){

                if(altHoldMode==1){
                    targetAlt = simGraphics1.mouseWheelAltitude/(-3.28084);
                    currAlt = sim2.positionZ;
                    altError = targetAlt - currAlt;

                    climbRateReq = pAltitude * altError;
                    climbRateReq = Math.max(-5, climbRateReq); //rates in m/s
                    climbRateReq = Math.min(5, climbRateReq);
                    ClimbRateReq  = String.valueOf(climbRateReq); if (ClimbRateReq.length() > 5) ClimbRateReq = ClimbRateReq.substring(0,5);

                    currClimbRate = sim2.velZ;
                    //5 meters per second = 984 feet per minute
                    if (altError > 10 || altError < -10){
                        if (climbRateReq < currClimbRate - 0.5) {changeAtt = 0.003;}
                        if (climbRateReq > currClimbRate + 0.5) {changeAtt = -0.003;}
                    } else {
                        if (climbRateReq < currClimbRate - 0.5) {changeAtt = 0.003;}
                        if (climbRateReq > currClimbRate + 0.5) {changeAtt = -0.003;}                    
                    }
                    if (APON)
                        simGraphics1.setAttitude(changeAtt);
                } //if(altHoldMode==1){

                if(altHoldMode==2){
                    targetAlt = simGraphics1.mouseWheelAltitude;

                    currAlt = -sim2.positionZ*(3.28084);
                    //altError = targetAlt - currAlt;
                    s = sim2.iasKnots;
                    zeroPitchElePosSet();
                    //climb
                    if ( currAlt < targetAlt - 2000){
                        targetAtt = 20*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(20.0);
                    } 
                    if ( currAlt > targetAlt - 2000 && currAlt < targetAlt - 1000){
                        targetAtt = 10*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(10.0);
                    }
                    if ( currAlt > targetAlt - 1000 && currAlt < targetAlt - 500){
                        targetAtt = 5*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(sLAtt + 4.5);
                    }
                    if ( currAlt > targetAlt - 500 && currAlt < targetAlt - 100){
                        targetAtt = 3*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(sLAtt + 1.5);
                    }
                    if ( currAlt > targetAlt - 100 && currAlt < targetAlt - 50){
                        targetAtt = 2.5*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(sLAtt + 1.0);
                    }
                    if ( currAlt > targetAlt - 50 && currAlt < targetAlt - 5){
                        targetAtt = 2.0*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(sLAtt + 0.5);
                    }
                    //y = 1.533333333·10-9 x4 - 1.983333333·10-6 x3 + 9.816666667·10-4 x2 - 2.236666667·10-1 x + 20.38
                    if (currAlt > targetAlt - 5 && currAlt < targetAlt + 30){
                        //targetAtt = (0.00000000153333*s*s*s*s)-(0.000001983333*s*s*s)+(0.000981666*s*s)-(0.223666*s)+20.38;
                        //simGraphics1.setMouseWheelAttitudeString();
                        targetAtt = 1.5*Math.PI/180; // for 200 kts
                        simGraphics1.setMouseWheelAttitude(sLAtt);
                    }

                    //descend
                    if ( currAlt > targetAlt + 500){
                        targetAtt = -2*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(sLAtt - 3.5);
                    }
                    if ( currAlt <= targetAlt + 500 && currAlt > targetAlt + 100){
                        targetAtt = -1*Math.PI/180;
                        simGraphics1.setMouseWheelAttitude(sLAtt - 2.5);
                    }
                    if ( currAlt <= targetAlt + 100 && currAlt > targetAlt + 50){
                        targetAtt = 0;
                        simGraphics1.setMouseWheelAttitude(sLAtt - 1.5);
                    }
                    if ( currAlt <= targetAlt + 50 && currAlt > targetAlt + 30){
                        targetAtt = 1.0;
                        simGraphics1.setMouseWheelAttitude(sLAtt - 0.5);
                    }
                    simGraphics1.setMouseWheelAttitudeString();
                    //if (simGraphics1.mouseWheelSpeed==200){
                    aobCor = (0.7096*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ)+(0.535*sim2.gammaQ*sim2.gammaQ);
                    //s = simGraphics1.mouseWheelSpeed;//

                    // if the attitude is between -0.3 degrees and 0.3 degrees of targetAtt,
                    // no elevator correction will be applied.
                    //if(altHoldMode==2){
                    if (APON){
                        if(sim2.betaQ < targetAtt + (-0.3*Math.PI/180) && sim2.betaQ > targetAtt + -1*Math.PI/180){
                            correction = sCor;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        if(sim2.betaQ < targetAtt + -1*Math.PI/180){
                            correction = sCor-0.1;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        //Attitude Hold
                        if(sim2.betaQ > targetAtt + 0.3*Math.PI/180 && sim2.betaQ < targetAtt + Math.PI/180){
                            correction = sCor+0.05;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        if (sim2.betaQ > targetAtt + Math.PI/180){
                            correction = sCor+0.2;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                    }
                }//if(altHoldMode==2){

                    
                if(altHoldMode==3){   
                    currAlt = -sim2.positionZ*(3.28084);
                    currClimbRate = -sim2.velZ*(3.28084)*60;

                    targetAlt = simGraphics1.mouseWheelAltitude;

                    alt2Go = targetAlt - currAlt;

                    //Get climb/descent initiated.
                    if (alt2Go > 200) clbDesAtt = 15;
                    if (alt2Go < -200) clbDesAtt = -2;

                    //calculate attitude
                    levelFlightAttitude();
                    //climb: attitude up. i.e, aircraft below altitude
                    //                     if((alt2Go > 0 && currClimbRate > 1)){
                    //                         clbDesAtt = sLAtt + ( 10 * alt2Go / (Math.max(100,currClimbRate)));
                    //                         CLB = true; DES = false; LVL = false;
                    //                     }
                    //                     if((alt2Go > 0 && currClimbRate < -1)){
                    //                         clbDesAtt = sLAtt - ( 10 * alt2Go / (Math.min(-100, currClimbRate)));
                    //                         CLB = true; DES = false; LVL = false;
                    //                     }
                    // 
                    //                     //descend: attitude down. i.e, aircraft above altitude
                    //                     if ((alt2Go < 0 && currClimbRate < -1) ){
                    //                         clbDesAtt = sLAtt - (3.5 * alt2Go / (Math.min(-100, currClimbRate)));
                    //                         CLB = false; DES = true; LVL = false;
                    //                     }
                    //                     if ((alt2Go < 0 && currClimbRate > 1) ){
                    //                         clbDesAtt = sLAtt + (3.5 * alt2Go / (Math.max(100,currClimbRate)));
                    //                         CLB = false; DES = true; LVL = false;
                    //                     }

                    if (alt2Go > 0)
                        clbDesAtt = sLAtt + ((alt2Go/1000)*((-0.005*currClimbRate)+20));

                    if (alt2Go < 0)
                        clbDesAtt = sLAtt + ((-alt2Go/1000)*((-0.0025*currClimbRate)-10));

                    clbDesAtt = Math.max(-2,clbDesAtt);
                    clbDesAtt = Math.min(20,clbDesAtt);

                    targetAtt = clbDesAtt * Math.PI/180;

                    simGraphics1.setMouseWheelAttitude(targetAtt* 180/Math.PI);

                    currAtt = sim2.betaQ;     // radians
                    attError = targetAtt - currAtt; // radians
                    if (targetAtt >= 15.0*Math.PI/180 ) {targetAtt = targetAtt - 0.5*Math.PI/180;} //correction
                    aobCor = (0.7096*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ)+(0.535*sim2.gammaQ*sim2.gammaQ);

                    s = sim2.iasKnots;
                    sCor = (-0.00000000699999*s*s*s*s)+(0.0000066999*s*s*s)+(-0.0024324999*s*s)+(0.40574999*s)-27.64999;
                    sCor = sCor-((sCor*100)%5)/100;  

                    if (s > 300 && s <= 310) sCor = -0.65;
                    if (s > 310 && s <= 330) sCor = -0.60;
                    if (s > 330 && s <= 340) sCor = -0.55;
                    if (s > 340 && s <= 360) sCor = -0.50;
                    if (s > 360 && s <= 380) sCor = -0.45;
                    if (s > 380 && s <= 395) sCor = -0.40;
                    if (s > 395 ) sCor = -0.40;

                    // if the attitude is between -0.3 degrees and 0.3 degrees of targetAtt,
                    // no elevator correction will be applied.
                    if (s >= 200)//to compensate elevator angle correction with speed
                        aobCor = aobCor * ((-0.0053*s) + 2.0067);
                    if (s >= 110 && s < 200)//to compensate elevator angle correction with speed
                        aobCor = aobCor * ((-0.02*s) + 5);
                    if (s < 110) aobCor = aobCor * 4;

                    if (APON){
                        if(sim2.betaQ < targetAtt + (-0.3*Math.PI/360) && sim2.betaQ > targetAtt + -1*Math.PI/360){
                            correction = sCor;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        if(sim2.betaQ < targetAtt + -1*Math.PI/360){
                            correction = sCor - 0.1;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        //Attitude Hold
                        if(sim2.betaQ > targetAtt + 0.3*Math.PI/360 && sim2.betaQ < targetAtt + Math.PI/360){
                            correction = sCor + 0.05;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                        if (sim2.betaQ > targetAtt + Math.PI/360){
                            correction = sCor + 0.2;
                            simGraphics1.ElevatorPositionSet(correction-aobCor);
                        }
                    }//if (APON){

                }     //if(altHoldMode==3){
            }//if (simGraphics1.altitudeOn){

            //GLIDE PATH to correct for aircraft below glide path required.
            //error = (current glide path - glide path required)
            //FPR = flight path required
            //GP = current glide path (i.e. the angle from the req'd touch down pstn to the aircraft)
            //GPR = glide path required - nominally a 3 degree glide path, i.e. -3 degrees
            //FPR = GPR + error + error * range * 0.2
            //max 3 degrees nose up,
            //min 0 degrees
            if (simGraphics1.glideSlope){
                levelFlightAttitude();

                targetAtt = (sLAtt*Math.PI/180.0) + simGraphics1.fltPathReqd;
                targetAtt = targetAtt + (0.3*Math.PI/180.0);
                
                targetAtt = Math.max(-5*Math.PI/180.0, targetAtt);
                targetAtt = Math.min( 5*Math.PI/180.0, targetAtt);
                
                simGraphics1.setMouseWheelAttitude(targetAtt* 180/Math.PI);

                currAtt = sim2.betaQ;     // radians
                attError = targetAtt - currAtt; // radians
                if (targetAtt >= 15.0*Math.PI/180 ) {targetAtt = targetAtt - 0.5*Math.PI/180;} //correction
                aobCor = (0.7096*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ*sim2.gammaQ)+(0.535*sim2.gammaQ*sim2.gammaQ);

                s = sim2.iasKnots;
                sCor = (-0.00000000699999*s*s*s*s)+(0.0000066999*s*s*s)+(-0.0024324999*s*s)+(0.40574999*s)-27.64999;
                sCor = sCor-((sCor*100)%5)/100;  

                if (s > 300 && s <= 310) sCor = -0.65;
                if (s > 310 && s <= 330) sCor = -0.60;
                if (s > 330 && s <= 340) sCor = -0.55;
                if (s > 340 && s <= 360) sCor = -0.50;
                if (s > 360 && s <= 380) sCor = -0.45;
                if (s > 380 && s <= 395) sCor = -0.40;
                if (s > 395 ) sCor = -0.40;

                // if the attitude is between -0.3 degrees and 0.3 degrees of targetAtt,
                // no elevator correction will be applied.
                if (s >= 200)//to compensate elevator angle correction with speed
                    aobCor = aobCor * ((-0.0053*s) + 2.0067);
                if (s >= 110 && s < 200)//to compensate elevator angle correction with speed
                    aobCor = aobCor * ((-0.02*s) + 5);
                if (s < 110) aobCor = aobCor * 4;

                if (APON){
                    if(sim2.betaQ < targetAtt + (-0.3*Math.PI/360) && sim2.betaQ > targetAtt + -1*Math.PI/360){
                        correction = sCor;
                        simGraphics1.ElevatorPositionSet(correction-aobCor);
                    }
                    if(sim2.betaQ < targetAtt + -1*Math.PI/360){
                        correction = sCor - 0.1;
                        simGraphics1.ElevatorPositionSet(correction-aobCor);
                    }
                    //Attitude Hold
                    if(sim2.betaQ > targetAtt + 0.3*Math.PI/360 && sim2.betaQ < targetAtt + Math.PI/360){
                        correction = sCor + 0.05;
                        simGraphics1.ElevatorPositionSet(correction-aobCor);
                    }
                    if (sim2.betaQ > targetAtt + Math.PI/360){
                        correction = sCor + 0.2;
                        simGraphics1.ElevatorPositionSet(correction-aobCor);
                    }
                }//if (APON){
            }//if (simGraphics1.glideSlope){

            //LLZ
            if (simGraphics1.llzActive){

                reqdHDG = (simGraphics1.HDGReqd*180.0/Math.PI);
                simGraphics1.setMouseWheelHeading(reqdHDG);
                
                currentHDG = sim2.alphaQ*180/Math.PI;
                if (currentHDG < 0)currentHDG = 360 + currentHDG;

                rHmcH = reqdHDG - currentHDG; if (rHmcH < 0) rHmcH = 360 + rHmcH;
                cHmrH = currentHDG - reqdHDG; if (cHmrH < 0) cHmrH = 360 + cHmrH;

                if (rHmcH < cHmrH) turnDirection = "R";
                if (rHmcH >= cHmrH) turnDirection = "L";

                if (turnDirection.equals("R")){
                    targetAoB = Math.min(rHmcH*Math.PI/180, 30*Math.PI/180);                    
                }
                if (turnDirection.equals("L")){
                    targetAoB = Math.max(-cHmrH*Math.PI/180, -30*Math.PI/180);                    
                }

                if (AoBQ.size() >= 100){ AoBQ.remove();}    
                AoBQ.add(sim2.gammaQ);
                dummyq = 0;      for(double e : AoBQ){dummyq = dummyq + e;}
                dummyq = (dummyq/AoBQ.size());  aobq = dummyq;

                s = sim2.iasKnots;
                sAPstnSmall = 0.125;
                if (s < 115) {
                    sAPstn = 3.5;       //checked at 100 kts
                    sAPstnSmall = 0.8;  //checked at 100 kts
                }
                if (s >= 115 && s < 125) {
                    sAPstn = 2.2;       //checked
                    sAPstnSmall = 0.7;  //checked
                }
                if (s >= 125 && s < 135) {
                    sAPstn = 1.9;
                    sAPstnSmall = 0.6;
                }
                if (s >= 135 && s < 145) {
                    sAPstn = 1.50;      //checked
                    sAPstnSmall = 0.5;  //checked
                }
                if (s >= 145 && s < 155) {
                    sAPstn = 1.30;
                    sAPstnSmall = 0.4;
                }
                if (s >= 155 && s < 165) {
                    sAPstn = 1.20;      //checked
                    sAPstnSmall = 0.2;  //checked
                }
                if (s >= 165 && s < 180) {
                    sAPstn = 1.30;
                }
                if (s >= 180 && s < 200) {
                    sAPstn = 1.0;
                }
                if (s >= 200 && s < 300){
                    sAPstn = 0.75;}
                if (s >= 300 && s < 340) {
                    sAPstn = 0.50;}
                if (s >= 340 && s < 380) {
                    sAPstn = 0.40;}
                if (s >= 380) {
                    sAPstn = 0.30;}

                if (APON){
                    if (rHmcH > 2 && turnDirection.equals("R")){
                        if (aobq < targetAoB){
                            simGraphics1.setAileronPosition(sAPstn);
                        }
                        if (aobq > (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(-sAPstn);
                        }
                        if (aobq > targetAoB && aobq < (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }

                    if(cHmrH > 2 && turnDirection.equals("L")){
                        if (aobq > targetAoB){
                            simGraphics1.setAileronPosition(-sAPstn);
                        }
                        if (aobq < (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(sAPstn);
                        }
                        if (aobq < targetAoB && aobq > (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }

                    if (rHmcH <= 2 && turnDirection.equals("R")){
                        if (aobq < targetAoB ){
                            simGraphics1.setAileronPosition(sAPstnSmall);
                        }
                        if (aobq > (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(-sAPstnSmall);
                        }
                        if (aobq > targetAoB && aobq < (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }

                    if(cHmrH <= 2 && turnDirection.equals("L")){
                        if (aobq > targetAoB ){
                            simGraphics1.setAileronPosition(-sAPstnSmall);
                        }
                        if (aobq < (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(sAPstnSmall);
                        }
                        if (aobq < targetAoB && aobq > (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }
                } //if (APON)
            } //LLZ
                
            //heading
            if (simGraphics1.hdgOn){

                reqdHDG = simGraphics1.mouseWheelHDG;
             
                
                currentHDG = sim2.alphaQ*180/Math.PI;
                if (currentHDG < 0)currentHDG = 360 + currentHDG;

                rHmcH = reqdHDG - currentHDG; if (rHmcH < 0) rHmcH = 360 + rHmcH;
                cHmrH = currentHDG - reqdHDG; if (cHmrH < 0) cHmrH = 360 + cHmrH;

                if (rHmcH < cHmrH) turnDirection = "R";
                if (rHmcH >= cHmrH) turnDirection = "L";

                if (turnDirection.equals("R")){
                    targetAoB = Math.min(rHmcH*Math.PI/180, 30*Math.PI/180);                    
                }
                if (turnDirection.equals("L")){
                    targetAoB = Math.max(-cHmrH*Math.PI/180, -30*Math.PI/180);                    
                }

                if (AoBQ.size() >= 100){ AoBQ.remove();}    
                AoBQ.add(sim2.gammaQ);
                dummyq = 0;      for(double e : AoBQ){dummyq = dummyq + e;}
                dummyq = (dummyq/AoBQ.size());  aobq = dummyq;

                s = sim2.iasKnots;
                sAPstnSmall = 0.125;
                if (s < 115) {
                    sAPstn = 3.5;       //checked at 100 kts
                    sAPstnSmall = 0.8;  //checked at 100 kts
                }
                if (s >= 115 && s < 125) {
                    sAPstn = 2.2;       //checked
                    sAPstnSmall = 0.7;  //checked
                }
                if (s >= 125 && s < 135) {
                    sAPstn = 1.9;
                    sAPstnSmall = 0.6;
                }
                if (s >= 135 && s < 145) {
                    sAPstn = 1.50;      //checked
                    sAPstnSmall = 0.5;  //checked
                }
                if (s >= 145 && s < 155) {
                    sAPstn = 1.30;
                    sAPstnSmall = 0.4;
                }
                if (s >= 155 && s < 165) {
                    sAPstn = 1.20;      //checked
                    sAPstnSmall = 0.2;  //checked
                }
                if (s >= 165 && s < 180) {
                    sAPstn = 1.30;
                }
                if (s >= 180 && s < 200) {
                    sAPstn = 1.0;
                }
                if (s >= 200 && s < 300){
                    sAPstn = 0.75;}
                if (s >= 300 && s < 340) {
                    sAPstn = 0.50;}
                if (s >= 340 && s < 380) {
                    sAPstn = 0.40;}
                if (s >= 380) {
                    sAPstn = 0.30;}

                if (APON){
                    if (rHmcH > 2 && turnDirection.equals("R")){
                        if (aobq < targetAoB){
                            simGraphics1.setAileronPosition(sAPstn);
                        }
                        if (aobq > (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(-sAPstn);
                        }
                        if (aobq > targetAoB && aobq < (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }

                    if(cHmrH > 2 && turnDirection.equals("L")){
                        if (aobq > targetAoB){
                            simGraphics1.setAileronPosition(-sAPstn);
                        }
                        if (aobq < (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(sAPstn);
                        }
                        if (aobq < targetAoB && aobq > (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }

                    if (rHmcH <= 2 && turnDirection.equals("R")){
                        if (aobq < targetAoB ){
                            simGraphics1.setAileronPosition(sAPstnSmall);
                        }
                        if (aobq > (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(-sAPstnSmall);
                        }
                        if (aobq > targetAoB && aobq < (targetAoB+(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }

                    if(cHmrH <= 2 && turnDirection.equals("L")){
                        if (aobq > targetAoB ){
                            simGraphics1.setAileronPosition(-sAPstnSmall);
                        }
                        if (aobq < (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(sAPstnSmall);
                        }
                        if (aobq < targetAoB && aobq > (targetAoB-(2*Math.PI/180))){
                            simGraphics1.setAileronPosition(0.0);
                        }
                    }
                } //if (APON)
            }   //heading

            Thread.currentThread();
            try {
                Thread.sleep(5);
            } catch (Exception e) {e.printStackTrace();}
        }
    }

    public static  void altHoldModeChange(){
        altHoldMode++;
        if (altHoldMode > 3 )
            altHoldMode = 1;
        ALTHoldMode = String.valueOf(altHoldMode);
    }

    public static  void attHoldModeChange(){
        attHoldMode++;
        if (attHoldMode > 3 )
            attHoldMode = 1;
        ATTHoldMode = String.valueOf(attHoldMode);
    }

    public static void zeroPitchElePosSet(){ //elevator positions: they're negative
        s = sim2.iasKnots;
        zeroPitchElePos = (-0.00000000699999*s*s*s*s)+(0.0000066999*s*s*s)+(-0.0024324999*s*s)+(0.40574999*s)-27.64999;
        zeroPitchElePos = zeroPitchElePos-((zeroPitchElePos*100)%5)/100;  
        if (s > 300 && s <= 310) zeroPitchElePos = -0.65;
        if (s > 310 && s <= 330) zeroPitchElePos = -0.60;
        if (s > 330 && s <= 340) zeroPitchElePos = -0.55;
        if (s > 340 && s <= 360) zeroPitchElePos = -0.50;
        if (s > 360 && s <= 380) zeroPitchElePos = -0.45;
        if (s > 380 && s <= 395) zeroPitchElePos = -0.40;
        if (s > 395 ) zeroPitchElePos = -0.40;
    }

    public static void levelFlightAttitude(){
        s = sim2.iasKnots;
        if (s <= 100) sLAtt = (-0.125 * s) + 18.5;
        if (s > 100 && s <= 120) sLAtt = (-0.1 * s) + 16.0;
        if (s > 120 && s <= 140) sLAtt = (-0.05 * s) + 10.0;
        if (s > 140 && s <= 150) sLAtt = (-0.04 * s) + 8.6;
        if (s > 150 && s <= 180) sLAtt = (-0.03 * s) + 7.1;
        if (s > 180 && s <= 200) sLAtt = (-0.015 * s) + 4.4;
        if (s > 200 && s <= 240) sLAtt = (-0.01 * s) + 3.4;
        if (s > 240 && s <= 360) sLAtt = (-0.005 * s) + 2.2;
        if (s > 360) sLAtt = 0.4;
    }

    public static void resetCumError(){
        cumError = 0;
    }

}