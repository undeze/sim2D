package sim2D;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseWheelListener;
import java.awt.event.MouseWheelEvent;
import java.awt.Canvas;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferStrategy;
import java.awt.Toolkit;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Random;

//
import java.awt.Color;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.geom.*;
import javax.swing.*;

import java.awt.Font;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

class Plane3D { // defined by a point and a normal vector. i.e. aircraft
				// position & z vector
	public double ptpx0, ptpy0, ptpz0, nva, nvb, nvc;

	public Plane3D(double x1, double y1, double z1, double x2, double y2, double z2) {
		ptpx0 = x1;
		ptpy0 = y1;
		ptpz0 = z1;
		nva = x2;
		nvb = y2;
		nvc = z2;
	}
}

class Point3D {
	public double ptptx, ptpty, ptptz;

	public Point3D(double X, double Y, double Z) {
		ptptx = X;
		ptpty = Y;
		ptptz = Z;
	}
}

class Line3D { // defined by two points
	public double ptlx1, ptly1, ptlz1, ptlx2, ptly2, ptlz2;

	public Line3D(Point3D pt1, Point3D pt2) {
		ptlx1 = pt1.ptptx;
		ptly1 = pt1.ptpty;
		ptlz1 = pt1.ptptz;
		ptlx2 = pt2.ptptx;
		ptly2 = pt2.ptpty;
		ptlz2 = pt2.ptptz;
	}
}

class Line2D { // defined by two points
	public int ptlx1, ptly1, ptlx2, ptly2;

	public Line2D(double x1, double y1, double x2, double y2) {
		ptlx1 = (int) x1;
		ptly1 = (int) y1;
		ptlx2 = (int) x2;
		ptly2 = (int) y2;
	}
}

public class simGraphics1 extends Canvas implements KeyListener, MouseListener, MouseWheelListener, Runnable {
	private Graphics bufferGraphics = null; // The graphics for the backbuffer
	private BufferStrategy bufferStrategy = null; // the stategy our app uses

	private Thread thread;
	private boolean running;
	private Rectangle rect, rectSys, rectRea;

	private ArrayList<Integer> keysDown;

	static int hdgX, hdgY; // heading indicator reference
	static int Nx, Ny, Ex, Ey, Sx, Sy, Wx, Wy; // heading indicator
	static int NxS, NyS, WxS, WyS, SxS, SyS, ExS, EyS; // heading indicator
														// String
	static double hdg; // heading indicator
	static int heading;
	static int NxSND, NySND, WxSND, WySND, SxSND, SySND, ExSND, EySND; // ND
																		// String

	static int pfdX, pfdY; // PFD reference
	static int pitchFactor;
	static int PDx, PDy;
	static int hXp00, hYp00;
	static int pfdXC, pfdYC;

	static int hX1, hY1, hX2, hY2, hXp, hYp;
	static boolean inX1, inY1, inX2, inY2, inX1a, inY1a, inX2a, inY2a, inX1b, inY1b, inX2b, inY2b;
	static boolean inX1c, inY1c, inX2c, inY2c;
	static double x1, x2, x3, x4, y1, y2, y3, y4, P1x, P1y, P2x, P2y;
	static double P1xa, P1ya, P2xa, P2ya;

	static int[] PDXg = new int[5]; // ground
	static int[] PDYg = new int[5];
	static int npoints, j;
	static int spoints;
	static int[] PDXs = new int[5]; // sky
	static int[] PDYs = new int[5];
	static int[] PDXv = new int[5]; // pfd visible ground
	static int[] PDYv = new int[5];

	static double attitude, conversion, Beta, Gamma, AoB;
	static boolean intercept, polygonDrawn;

	// Color
	static Color ground = new Color(216, 199, 169); // 216,199,169
	static Color sky = new Color(202, 225, 255); // lightsteelblue 1
	static Color pink = new Color(255, 192, 203);
	static Color darkGreen = new Color(0, 100, 0);
	static Color button = new Color(105, 105, 105); // gammaQ, betaQ, alphaQ
	static Color colonialWhite = new Color(244, 237, 218);
	static int intValue = Integer.parseInt("F8E744", 16);
	static Color yellowy = new Color(intValue);
	static int intValue2 = Integer.parseInt("FABACA", 16);
	static Color pinky = new Color(intValue2);
	static Color altGrd = new Color(179, 202, 142);
	// http://www.tayloredmktg.com/rgb/

	static int pfdWidth;
	static double pfdMaxAngle;

	double beta, gamma;

	static double ailPos, ailPosReq;
	static String AilPos, ElePos, RudPos;
	static double elePos, elePosReq;
	static double rudPos, rudPosReq;
	static int throttlePosition, targetThrust, thrust;
	static int elePR, eleP, ailPR, ailP, rudPR, rudP;
	static double actThrust;

	static String Heading;
	static int[] Hdgs = new int[36];
	static double[] HdgD = new double[6];
	static int[] HdgsX = new int[6];
	static String[] HdgsS = new String[36];
	static double hdgDiff, doubleHdgDegrees, exactHeading, pixelHdg;
	static int pixelOffset;
	static String H1, H2, H3, H4, H5, H6;
	static int H1int, H2int, H3int, H4int, H5int, H6int;

	static String Airspeed, Altitude;
	private static int airspeed, altitude;
	static int[] iasXLeft = new int[] { 215, 215, 220 };
	static int[] iasYLeft = new int[] { 207, 213, 210 };
	static int[] iasXRight = new int[] { 251, 251, 246 };
	static int[] iasYRight = new int[] { 207, 213, 210 };

	static double mouseWheelHDG = 0.0;
	static String MouseWheelHDG = "000";
	static int mouseWheelSpeed = 200;
	static String MouseWheelSpeed = "200";
	static double mouseWheelAttitude = 5;
	static String MouseWheelAttitude = "5";
	static int mouseWheelAltitude = 10000;
	static String MouseWheelAltitude = "10000";

	static boolean hdgOn, speedOn, attitudeOn, altitudeOn;

	// private static Arc2D.Double arc = new Arc2D.Double();
	private static Arc2D.Double arc;
	// private static Arc2D.Double arc = new Arc2D.Double(600, 200, 100, 50, 45,
	// 90, Arc2D.OPEN);

	private static int AoBx01, AoBx02, AoBy01, AoBy02;
	static String changeParameter;
	static String lateralMode, verticalMode;

	static int sButtX, sButtY, ATTHButtX, ATTHButtY, ALTButtX, ALTButtY, ApprX, ApprY;
	static int cButtX, cButtY;
	static int attMX, attMY;// attitude matrix
	static int thVX, thVY; // thrust vector
	static int vVX, vVY; // velocity vector
	static int bRX, bRY; // body rates
	static int posX, posY; // position
	static int resetX, resetY; // reset button
	static int stopX, stopY; // stop button
	static int balX, balY;
	static int fontX, fontY; // font button
	static int visX, visY; // projection visuals

	static int resLNX, resLNY, resLSX, resLSY; // reset to landing
	static int resGX, resGY; // reset to glide
	static int resTOX, resTOY; // reste to takeoff, runway 36
	static int res10X, res10Y;

	static int ndX, ndY, ndWidth, ndXC, ndYC;
	static int boyX, boyY; // Boy/Girl button
	static int dataX, dataY; // Data button
	static int pfdOnX, pfdOnY; // PFD button
	static int attHX, attHY; // attitude mode button
	static int rollMX, rollMY; // roll mode button
	static int hdgBX, hdgBY; // heading select
	static int fdBX, fdBY; // flight director
	static int apBX, apBY; // auto pilot
	static int vvBX, vvBY; // velocity vector, flight path marker
	static int altBX, altBY; // altitude mode select button
	static int fmOX, fmOY; // flame out button

	static boolean boy = false;
	static boolean drawData = true;
	static boolean vvOn = true;

	static int nd1DX, nd1DY, nd2DX, nd2DY;
	static double hdgND;

	static double bearing, range, bear2;

	static String Bearing, Range, Bear2;
	static double aX1, aY1, aX2, aY2; // head of arrow
	static double aXL, aYL, aXR, aYR; // head
	static double bX1, bY1, bX2, bY2; // tail of arrow
	static double bXL, bYL, bXR, bYR; // tail

	static int ailRudX, ailRudY;
	static int eleX, eleY;
	static int aoaX, aoaY, gStripX, gStripY;

	static double vs, vsIX, vsIY, vsIX1, vsIY1, vsIX2, vsIY2, vsIX3, vsIY3, vsIX4, vsIY4;
	static String VS;

	static int commandedSpeed;
	static double ndSX, ndSY;
	static String Hdg;
	static int hdHdg;

	static double sim2g;

	static Font stringFont = new Font("SansSerif", Font.PLAIN, 12);
	static Font smallFont = new Font("SansSerif", Font.PLAIN, 9);
	static Font largeFont = new Font("SansSerif", Font.PLAIN, 15);
	static Font largeBold = new Font("SansSerif", Font.BOLD, 15);
	Font font = new Font(null, Font.PLAIN, 13);
	AffineTransform affineTransform = new AffineTransform();
	Font rotatedFont = new Font("SansSerif", Font.PLAIN, 12);
	static boolean rotateFont = true;
	Font uac = new Font("Harlow Solid Italic", Font.PLAIN, 13);

	static double r11, r12, r21, r22;
	static double p1x, p1y, p2x, p2y, n1x, n1y, n2x, n2y;

	static boolean pfdOn = true;

	static long timeNow, timeThen, timeDiff;
	static long adjustTime = 3000;

	static int rollAoAStrip;

	static int[] hdgBugX = new int[3];
	static int[] hdgBugY = new int[3];// poly
	static int[] cmdAoBX = new int[3];
	static int[] cmdAoBY = new int[3];

	static double d1, d2;
	static String D1, D2;

	static boolean fdOn, apOn, spoilersOn;
	static int fdRoll, fdPitch;
	static String eleBase;
	Queue<Double> fdPitchQ = new LinkedList<Double>();
	static double fdPitchq, fdPitchD, dummyq;
	static int tPX, tPY; // throttle position
	static int g, aoa;

	static int[] VSX = new int[4];// poly
	static int[] VSY = new int[4];

	static int ctlInt, rAoA, lAoA, qAoA, rAilP, lAilP;
	static int spX, spY; // spoiler switch

	// static int pfdHdgBug, bugDifInt;
	// private static double pfdHdgBugDouble, bugDifDou, rVL, rVR, rHmcH, cHmrH,
	// currentHDG, mwHD, aH;
	static int grd;
	static int dL; // data line

	static boolean glideOn, glideSlope;

	static int wPX, wPY;
	static double ndScale = 6.0;
	static String NDScale = "6";

	static double thrLX, thrLY, thrRX, thrRY, farLX, farLY, farRX, farRY;
	static int thGLX, thGLY, thGRX, thGRY, faGLX, faGLY, faGRX, faGRY; // graphics.
	static double distD, distE, phiD, phiE;

	static int[] RWGX = new int[5];// poly
	static int[] RWGY = new int[5];
	static double[] RWYX = new double[5];// poly
	static double[] RWYY = new double[5];
	static double[] RWYZ = new double[5];

	static double[] RNDY = new double[5]; // this to make the runway appear
											// wider on the ND

	static int resetPX, resetPY;
	static boolean nav1On;
	static int nav1X, nav1Y; // vor needle button
	static int drt1X, drt1Y; // direct to button
	static int wpSelX, wpSelY; // select waypoint button
	static int rng6X, rng6Y, rngPX, rngPY, rngMX, rngMY;

	private Area ndR, ndC;
	private Rectangle2D ndRect;
	private Ellipse2D ndCircle;
	private Area bkG, ibkG, pfdH;
	private Rectangle2D backGnd, intBckGrd, pfdHole;

	private static double fOHDG;

	// static double objAngle;// angle of object for pfd runway
	// static double bear3, relB;// relative bearing of object from aircraft
	// nose
	static String Bear3, RelB, HDGND, ObjAngle, whichIf;

	static double cDX1, cDY1, cDX2, cDY2, cDX3, cDY3, cDX4, cDY4;

	static boolean runwayOn;

	static int npolygon;

	static int newX, newY;

	static String HXp, HYp, CDX2, CDY2;

	static Line3D rwy[] = new Line3D[4];
	static Line2D drawLines[] = new Line2D[4];
	static double relB1, relB2;

	static double range1, objAngle1, bear31, range2, objAngle2, bear32;

	static double toPointX, toPointY, Xpos, Ypos;
	static String ToPoint = "RWY36";

	static double RWY36[] = new double[2];
	static double RWY18[] = new double[2];
	static double RWY36TDX, RWY36TDY;

	static int velVecX, velVecY;
	static double vvLXin, vvLYin, vvRXin, vvRYin, vvTXin, vvTYin;
	static double vvLXou, vvLYou, vvRXou, vvRYou, vvTXou, vvTYou;

	static int gearX, gearY;
	static int upDnX, upDnY;
	static boolean gearDn, gearTx, gearUp;
	static long gearThen, gearDiff;
	static long gearTime = 5000; // 5 seconds for gear to travel
	static long noseGTime, rightTime, leftGTime;

	static double currGP, gPError, glideSlopeReqd, gPRange, fltPathReqd;
	static String FPR, GPR, GPE, CGP;
	static int gpDiamondX[] = new int[4];
	static int gpDiamondY[] = new int[4];
	static boolean apprArmed;

	static double currHDG, llzError, HDGReqd, llzRange;
	// static String FPR, GPR, GPE, CGP;
	static int llzDiamondX[] = new int[4];
	static int llzDiamondY[] = new int[4];
	static boolean llzArmed, llzActive;
	static double LLZ36X, LLZ36Y;
	static int LLZX, LLZY; // buttons
	static double NDhdgBug, mouseWheelHeading;

	static double cl1X, cl1Y, cl2X, cl2Y; // centerline markings

	static double[] RCLX = new double[5];// poly
	static double[] RCLY = new double[5];

	static boolean altArmed;

	public simGraphics1(Dimension size) {
		// Construct
		gearDn = true;
		glideSlope = false;
		apprArmed = false;

		altArmed = false;

		pfdX = 250;
		pfdY = 60;
		pfdWidth = 300;
		runwayOn = true;
		// circle = new Area(shape1);
		// rect2 = new Rectangle(400, 0, 30, 30);
		// arc2 = new Arc2D.Double(0, 0, 24, 24, 45, 90, Arc2D.CHORD);

		// Runway position on map (meters). I.E. not on graphics pane
		RWYX[0] = 2000;
		RWYY[0] = -50;
		RNDY[0] = -300;
		RWYZ[0] = 0;
		RWYX[1] = 2000;
		RWYY[1] = 50;
		RNDY[1] = 300;
		RWYZ[1] = 0;
		RWYX[2] = 5500;
		RWYY[2] = 50;
		RNDY[2] = 300;
		RWYZ[2] = 0;
		RWYX[3] = 5500;
		RWYY[3] = -50;
		RNDY[3] = -300;
		RWYZ[3] = 0;
		RWYX[4] = 3250;
		RWYY[4] = 0;
		RWYZ[4] = 0;

		RWY36[0] = 2000;
		RWY36[1] = 0; // x and y coordinates of runway 36 threshold
		RWY18[0] = 5500;
		RWY18[1] = 0; // x and y coordinates of runway 18 threshold

		RWY36TDX = 2300;
		RWY36TDY = 0;
		glideSlopeReqd = -3.0 * Math.PI / 180.0;
		LLZ36X = 5500;
		LLZ36Y = 0; // localizer 36 position (far end threshold)

		cl1X = 2500;
		cl1Y = 0;
		cl2X = 3000;
		cl2Y = 0;
		RCLX[0] = 2500;
		RCLY[0] = 0;
		RCLX[1] = 2600;
		RCLY[1] = 0;

		glideOn = false;
		dL = 455;
		ndX = 750;
		ndY = 50;
		ndWidth = 320;
		ndXC = ndX + ndWidth / 2;
		ndYC = ndY + ndWidth / 2;
		ndRect = new Rectangle2D.Double(720, 50 - 30, 375, 320 + 53);
		ndCircle = new Ellipse2D.Double(750 + 40, 50 + 40, 240, 240);
		ndR = new Area(ndRect);
		ndC = new Area(ndCircle);
		ndR.subtract(ndC);

		backGnd = new Rectangle2D.Double(0, 0, 1220, 720);
		bkG = new Area(backGnd);
		pfdHole = new Rectangle2D.Double(pfdX, pfdY, pfdWidth, pfdWidth);
		pfdH = new Area(pfdHole);
		intBckGrd = new Rectangle2D.Double(0, 0, 1105, dL);
		ibkG = new Area(intBckGrd);
		bkG.subtract(pfdH);
		ibkG.subtract(pfdH);

		nav1X = ndX - 10;
		nav1Y = 25;
		drt1X = ndX + 30;
		drt1Y = 25;
		wpSelX = ndX + 70;
		wpSelY = 25;

		// rng6X = ndX + ndWidth + 5; rng6Y = 265;
		rngPX = ndX + ndWidth - 60;
		rngPY = 360;
		rngMX = ndX + ndWidth - 20;
		rngMY = 360;

		balX = 1100;
		balY = 460; // balance
		fdBX = 645;
		fdBY = 60; // flight director button
		apBX = 645;
		apBY = 100; // auto pilot button
		vvBX = 645;
		vvBY = 140; // velocity vector button
		spX = 645;
		spY = 300; //

		fmOX = 50;
		fmOY = 210; // flame out button

		sButtX = 150;
		sButtY = 400; // speed button
		ALTButtX = 240;
		ALTButtY = 400; // alt button
		ApprX = 510;
		ApprY = 400; // Approach guidance button
		hdgBX = 420;
		hdgBY = 400; // heading button
		LLZX = 600;
		LLZY = 400; // LLZ button
		ATTHButtX = 330;
		ATTHButtY = 400; // attitude hold button

		attHX = 1000;
		attHY = 460; // attitude hold mode button
		rollMX = 1000;
		rollMY = 520; // roll mode button
		altBX = 1000;
		altBY = 580; // altitude mode button

		attMX = 700;
		attMY = 480; // attitude matrix position
		thVX = 840;
		thVY = 480;
		vVX = 900;
		vVY = 480;
		bRX = 520;
		bRY = 480;
		posX = 600;
		posY = 480;

		resetPX = 0;
		resetPY = 590;
		resGX = 20;
		resGY = 650;
		resetX = 20;
		resetY = 610;
		resLNX = 100;
		resLNY = 610;
		resLSX = 100;
		resLSY = 650;
		resTOX = 180;
		resTOY = 650;
		res10X = 180;
		res10Y = 610;

		stopX = 420;
		stopY = 590;
		boyX = 340;
		boyY = 635;
		dataX = 420;
		dataY = 635;
		fontX = 260;
		fontY = 590;
		visX = 260;
		visY = 635;
		pfdOnX = 340;
		pfdOnY = 590;

		gearX = 30;
		gearY = 400; // gear wheel panel
		upDnX = 20;
		upDnY = 410;

		rollAoAStrip = 1145;
		eleX = 0;
		eleY = 250;
		ailRudX = eleX + 30;
		ailRudY = 250;
		gStripX = 160;
		aoaX = gStripX + 25;
		tPX = 40;
		tPY = 50;

		lateralMode = " ";
		verticalMode = " ";
		keysDown = new ArrayList<Integer>();
		this.setPreferredSize(size);
		this.addKeyListener(this);
		this.addMouseListener(this);
		this.addMouseWheelListener(this);

		this.thread = new Thread(this);
		running = true;

		rect = new Rectangle(0, 0, 24, 24);

		// From previous graphics constructor
		hdgX = 130;
		hdgY = 150;
		hdg = 0;
		// The line below is just to stop a line being draw from the origin when
		// the program starts.
		Nx = hdgX;
		Ny = hdgY;
		Ex = hdgX;
		Ey = hdgY;
		Sx = hdgX;
		Sy = hdgY;
		Wx = hdgX;
		Wy = hdgY;

		pitchFactor = 450;
		pfdXC = pfdX + (pfdWidth / 2);
		pfdYC = pfdY + (pfdWidth / 2);
		PDx = pfdX + 150;
		PDy = pfdY + 150;
		arc = new Arc2D.Double(pfdX + 20, pfdY + 20, pfdWidth - 40, pfdWidth - 40, 60, 60, Arc2D.OPEN);

		conversion = 360 / (2 * Math.PI);
		pfdWidth = 300;
		pfdMaxAngle = (pfdWidth / 2.0) / ((Math.PI / 180) * pitchFactor);

		vsIX1 = pfdX + pfdWidth + 60; // vertical speed indicator
		vsIX2 = vsIX1 + 105;
		vsIY2 = pfdYC;
		vsIX3 = vsIX1 + 20;
		vsIY3 = pfdY;
		vsIX4 = vsIX1 + 20;
		vsIY4 = pfdY + pfdWidth;

		elePos = 0;
		throttlePosition = 0;

		apOn = false;
		hdgOn = false;
		speedOn = false;
		attitudeOn = false;
		altitudeOn = false;
		changeParameter = "Speed";
		spoilersOn = false;
		gearTx = false;

		VSX[0] = 610;
		VSX[1] = 610;
		VSX[2] = 625;
		VSX[3] = 625;
		VSY[0] = pfdYC - 150;
		VSY[1] = pfdYC + 150;
		VSY[2] = pfdYC + 130;
		VSY[3] = pfdYC - 130;
	}

	// THRUST LEVER
	public void paint(Graphics g) {
		// this method draws on to the screen
		if (bufferStrategy == null) {
			this.createBufferStrategy(2);
			bufferStrategy = this.getBufferStrategy();
			bufferGraphics = bufferStrategy.getDrawGraphics();
			this.thread.start(); // starts thread, initiates run method
		}
	}

	@Override
	public void run() {
		// This is what runs when the level editior is running
		this.requestFocus();
		reset();
		while (running) {
			// Program's logic: math and calculations

			timeNow = System.currentTimeMillis();
			timeDiff = timeNow - timeThen;
			gearDiff = timeNow - gearThen;

			if (!fdOn && !apOn) {
				verticalMode = " ";
			}
			// if (!altitudeOn){verticalMode = " ";}

			if (gearDiff > gearTime) {
				gearTx = false;
			}

			if (apprArmed) {
				gPRange = RWY36TDX - sim2.positionX;
				currGP = Math.atan(sim2.positionZ / gPRange);
				gPError = currGP - glideSlopeReqd;
				if (gPError < 0.3 * Math.PI / 180.0 && gPError > -0.3 * Math.PI / 180.0) {
					glideSlope = true;
					apprArmed = false;
					altitudeOn = false;
					verticalMode = "GS";
					attitudeOn = false;
					mouseWheelSpeed = 140;
					MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
				}
			}
			if (glideSlope) { // calculates the flight path required to maintain
								// the glide slope
				// currGP, gPError, glideSlopeReqd, gPRange;
				gPRange = RWY36TDX - sim2.positionX;
				currGP = Math.atan(sim2.positionZ / gPRange);
				gPError = currGP - glideSlopeReqd;
				fltPathReqd = glideSlopeReqd + gPError + (gPError * gPRange / 1852.0);// *
																						// 0.8);
				// FPR = String.valueOf(fltPathReqd*180/Math.PI);
				// GPR = String.valueOf(glideSlopeReqd*180/Math.PI);
				// CGP = String.valueOf(currGP*180.0/Math.PI);
				// GPE = String.valueOf(gPError*180/Math.PI);
			}

			if (llzArmed) {
				llzError = Math.atan((LLZ36Y - sim2.positionY) / (LLZ36X - sim2.positionX));
				if (llzError < 2.0 * Math.PI / 180.0 && llzError > -2.0 * Math.PI / 180.0) {
					llzActive = true;
					llzArmed = false;
					hdgOn = false;
				}
			}
			if (llzActive) {
				llzRange = LLZ36X - sim2.positionX;
				llzError = Math.atan((LLZ36Y - sim2.positionY) / (LLZ36X - sim2.positionX));
				currHDG = sim2.alphaQ;
				HDGReqd = (2 * llzError) + (llzError * llzRange / 1852.0);
				// bG2D.drawString(MouseWheelHDG, hdgBX+22, hdgBY+45);
			}
			// static double currHDG, llzError, HDGReqd;
			// static String FPR, GPR, GPE, CGP;

			// static boolean llzArmed, llzActive;
			if (altArmed) { // vertical mode
				if (sim2.positionZ * -3.2808 < mouseWheelAltitude + 1000) {
					altitudeOn = true;
					altArmed = false;
					attitudeOn = false;
					glideSlope = false;
					apprArmed = false;
					verticalMode = "ALT";
				}
			}

			controlPositions();
			vs = -sim2.velZq * 3.28084 * 60; // m/s to ft/min
			VS = String.valueOf((int) vs / 100);

			hdgND = sim2.alphaQ;
			bearingRange();

			hdgIndicatorAndND();
			PFD();

			Draw();
			DrawBackbufferToScreen();

			Thread.currentThread();
			try {
				Thread.sleep(1);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	// A/THR
	public void Draw() {
		// Clearing the secondary screen - that we can't see
		bufferGraphics = bufferStrategy.getDrawGraphics();
		try {
			bufferGraphics.clearRect(0, 0, this.getSize().width, this.getSize().height);
			// this is where everything will be drawn to the backbuffer
			Graphics2D bG2D = (Graphics2D) bufferGraphics;
			// add code below here.

			if (pfdOn) {
				polygonDrawn = false;
				// check to ensure the green horizon line lies on the PFD.
				if (P1x > (pfdX - 0.001) && P1x < (pfdX + 300.001) && P2x > (pfdX - 0.001) && P2x < (pfdX + 300.001)) {
					if (P1y > (pfdY - 0.001) && P1y < (pfdY + 300.001) && P2y > (pfdY - 0.001)
							&& P2y < (pfdY + 300.001)) {
						bG2D.setColor(ground);
						bG2D.fillPolygon(PDXg, PDYg, npoints); // draw the
																// ground
																// polygon
						bG2D.setColor(sky);
						bG2D.fillPolygon(PDXs, PDYs, spoints); // draw the sky
																// polygon
						bG2D.setColor(darkGreen); // draw the green horizon line
						bG2D.drawLine((int) P1x, (int) P1y, (int) P2x, (int) P2y);
						polygonDrawn = true;
					}
				}

				if (!polygonDrawn && attitude <= -pfdMaxAngle) {
					fillGround();
					bG2D.setColor(ground); // draw the ground polygon

					bG2D.fillPolygon(PDXg, PDYg, npoints);
				}

				if (!polygonDrawn && attitude >= pfdMaxAngle) {
					fillSky();
					bG2D.setColor(sky); // draw the sky polygon
					bG2D.fillPolygon(PDXs, PDYs, spoints);
				}

			} // if(pfdOn){
				// draw runway on pfd
			if (runwayOn && pfdOn) { // runway projection
				PFDRunway();
				bG2D.setColor(Color.gray);
				// bG2D.fillPolygon(RWGX, RWGY, npolygon);
				// bG2D.setColor(Color.yellow);
				bG2D.setColor(Color.darkGray);
				for (int i = 0; i <= 3; i++) {
					if (drawLines[i] != null) {
						bG2D.drawLine(drawLines[i].ptlx1, drawLines[i].ptly1, drawLines[i].ptlx2, drawLines[i].ptly2);
					}
				}
				bG2D.setColor(Color.white);

				if (llzActive || sim2.positionZ > -50) {
					for (int j = 0; j <= 66; j = j + 2) {
						RCLX[0] = 2050 + (j * 50);
						RCLY[0] = 0;
						RCLX[1] = 2050 + ((j + 1) * 50);
						RCLY[1] = 0;
						PFDRunwayCL();
						if (drawLines[0] != null) {
							bG2D.drawLine(drawLines[0].ptlx1, drawLines[0].ptly1, drawLines[0].ptlx2,
									drawLines[0].ptly2);
						}
					}
				}
			} // if(runwayOn){//runway projection

			// PFD
			if (pfdOn) {

				bG2D.setColor(Color.BLUE);
				for (int i = 1; i < 9; i++) { // draw attitudes above horizon
					attitudesAbove(i);
					if (inX1a && inY1a && inX2a && inY2a) {
						bG2D.drawLine((int) P1xa, (int) P1ya, (int) P2xa, (int) P2ya);
					}
					if (inX2b && inY2b) { // draw attitude numerical values
						j = i * 10;
						if (rotateFont) {
							affineTransform.rotate(-sim2.gammaQ, 0, 0);
							rotatedFont = font.deriveFont(affineTransform);
							bG2D.setFont(rotatedFont);
							affineTransform.rotate(sim2.gammaQ, 0, 0);
						}

						bG2D.drawString(String.valueOf(j), (int) P2xa, (int) P2ya);
						bG2D.setFont(stringFont);
					}
					if (inX1c && inY1c) {
						bG2D.drawOval(hXp - 4, hYp - 4, 8, 8);
					}
				}
				for (int i = 1; i <= 9; i++) { // draw 5 degree attitudes below
												// the horizon
					attitudesAbove5(i);
					if (inX1a && inY1a && inX2a && inY2a) {
						bG2D.drawLine((int) P1xa, (int) P1ya, (int) P2xa, (int) P2ya);
					}
				}
				bG2D.setColor(darkGreen);
				for (int i = 1; i < 9; i++) { // draw 10 degree attitudes below
												// the horizon
					attitudesBelow(i);
					if (inX1a && inY1a && inX2a && inY2a) {
						bG2D.drawLine((int) P1xa, (int) P1ya, (int) P2xa, (int) P2ya);
					}
					if (inX2b && inY2b) { // draw attitude numerical values
						j = i * -10;
						if (rotateFont) {
							affineTransform.rotate(-sim2.gammaQ, 0, 0);
							rotatedFont = font.deriveFont(affineTransform);
							bG2D.setFont(rotatedFont);
							affineTransform.rotate(sim2.gammaQ, 0, 0);
						}
						bG2D.drawString(String.valueOf(j), (int) P2xa, (int) P2ya);
						bG2D.setFont(stringFont);
					}
					if (inX1c && inY1c) {
						bG2D.fillOval(hXp - 4, hYp - 4, 8, 8);
					}
				} // draw 10 degree attitudes below the horizon
				for (int i = 1; i <= 9; i++) { // draw 5 degree attitudes below
												// the horizon
					attitudesBelow5(i);
					if (inX1a && inY1a && inX2a && inY2a) {
						bG2D.drawLine((int) P1xa, (int) P1ya, (int) P2xa, (int) P2ya);
					}
				} // draw 5 degree attitudes below the horizon

				if (vvOn) {
					velocityVectorCalc();
					bG2D.setColor(Color.green);
					bG2D.drawOval(velVecX - 8, velVecY - 8, 16, 16);
					bG2D.drawOval(velVecX - 7, velVecY - 7, 14, 14);
					bG2D.drawLine((int) vvLXin, (int) vvLYin, (int) vvLXou, (int) vvLYou);
					bG2D.drawLine((int) vvRXin, (int) vvRYin, (int) vvRXou, (int) vvRYou);
					bG2D.drawLine((int) vvTXin, (int) vvTYin, (int) vvTXou, (int) vvTYou);
				}

				if (fdOn) {
					bG2D.setColor(Color.magenta);
					if (hdgOn || llzActive) { // flight director heading
						fdRoll = pfdXC + (int) ((autoPilot.targetAoB - sim2.gammaQ) * 100);
						fdRoll = Math.min(fdRoll, pfdXC + 100);
						fdRoll = Math.max(fdRoll, pfdXC - 100);
						bG2D.drawLine(fdRoll, pfdYC - 80, fdRoll, pfdYC + 80);
						bG2D.drawLine(fdRoll + 1, pfdYC - 80, fdRoll + 1, pfdYC + 80);
						bG2D.drawLine(fdRoll - 1, pfdYC - 80, fdRoll - 1, pfdYC + 80);

						cmdAoBX[0] = pfdXC - (int) (130 * Math.sin(autoPilot.targetAoB));
						cmdAoBX[1] = pfdXC - (int) (138 * Math.sin(autoPilot.targetAoB + 0.03));
						cmdAoBX[2] = pfdXC - (int) (138 * Math.sin(autoPilot.targetAoB - 0.03));
						cmdAoBY[0] = pfdYC - (int) (130 * Math.cos(autoPilot.targetAoB));
						cmdAoBY[1] = pfdYC - (int) (138 * Math.cos(autoPilot.targetAoB + 0.03));
						cmdAoBY[2] = pfdYC - (int) (138 * Math.cos(autoPilot.targetAoB - 0.03));
						bG2D.fillPolygon(cmdAoBX, cmdAoBY, 3);
					} // if (hdgOn){
					if (attitudeOn || altitudeOn || glideSlope) { // flight
																	// director
																	// pitch
						fdPitch = pfdYC - (int) ((autoPilot.targetAtt - sim2.betaQ) * 100);
						fdPitch = Math.min(fdPitch, pfdYC + 100);
						fdPitch = Math.max(fdPitch, pfdYC - 100);
						bG2D.drawLine(pfdXC - 80, fdPitch, pfdXC + 80, fdPitch);
						bG2D.drawLine(pfdXC - 80, fdPitch + 1, pfdXC + 80, fdPitch + 1);
						bG2D.drawLine(pfdXC - 80, fdPitch - 1, pfdXC + 80, fdPitch - 1);
					} // if (attitudeOn || altitudeOn || glideSlope){
				} // if (fdOn){

				// aircraft, tail and wings
				bG2D.setColor(Color.yellow);
				bG2D.fillOval(pfdX + 147, pfdY + 147, 6, 6);
				bG2D.setColor(Color.black);
				bG2D.fillOval(pfdX + 148, pfdY + 148, 4, 4);
				bG2D.drawLine(pfdX + 135, pfdY + 150, pfdX + 165, pfdY + 150); // wings
				bG2D.drawLine(pfdX + 150, pfdY + 140, pfdX + 150, pfdY + 150);// tail
				bG2D.drawRect(pfdX, pfdY, pfdWidth, pfdWidth); // surrounding
																// box
				bG2D.drawLine(pfdX + 50, pfdY + 150, pfdX + 100, pfdY + 150);
				bG2D.drawLine(pfdX + 200, pfdY + 150, pfdX + 250, pfdY + 150);
				bG2D.drawLine(pfdX + 100, pfdY + 150, pfdX + 100, pfdY + 157);
				bG2D.drawLine(pfdX + 200, pfdY + 150, pfdX + 200, pfdY + 157);

				// pfd angle of bank
				bG2D.setColor(Color.black);
				bG2D.draw(arc);
				bG2D.setColor(button);
				AoBx01 = pfdXC - (int) (110 * Math.sin(sim2.gammaQ)); // pointer
				AoBy01 = pfdYC - (int) (110 * Math.cos(sim2.gammaQ));
				AoBx02 = pfdXC - (int) (130 * Math.sin(sim2.gammaQ));
				AoBy02 = pfdYC - (int) (130 * Math.cos(sim2.gammaQ));
				bG2D.drawLine(AoBx01, AoBy01, AoBx02, AoBy02);
				AoBx01 = pfdXC - (int) (120 * Math.sin(sim2.gammaQ - 0.04));
				AoBy01 = pfdYC - (int) (120 * Math.cos(sim2.gammaQ - 0.04));
				AoBx02 = pfdXC - (int) (130 * Math.sin(sim2.gammaQ));
				AoBy02 = pfdYC - (int) (130 * Math.cos(sim2.gammaQ));
				bG2D.drawLine(AoBx01, AoBy01, AoBx02, AoBy02);
				AoBx01 = pfdXC - (int) (120 * Math.sin(sim2.gammaQ + 0.04));
				AoBy01 = pfdYC - (int) (120 * Math.cos(sim2.gammaQ + 0.04));
				AoBx02 = pfdXC - (int) (130 * Math.sin(sim2.gammaQ));
				AoBy02 = pfdYC - (int) (130 * Math.cos(sim2.gammaQ));
				bG2D.drawLine(AoBx01, AoBy01, AoBx02, AoBy02);
				bG2D.setColor(Color.black);
				for (int j = -18; j <= 18; j++) {
					if (j % 3 == 0) {
						AoBx01 = pfdXC - (int) (130 * Math.sin(j * Math.PI / 18));
						AoBy01 = pfdYC - (int) (130 * Math.cos(j * Math.PI / 18));
						AoBx02 = pfdXC - (int) (149 * Math.sin(j * Math.PI / 18));
						AoBy02 = pfdYC - (int) (149 * Math.cos(j * Math.PI / 18));
					}
					if (j <= 2 && j >= -2) {
						AoBx01 = pfdXC - (int) (130 * Math.sin(j * Math.PI / 18));
						AoBy01 = pfdYC - (int) (130 * Math.cos(j * Math.PI / 18));
						AoBx02 = pfdXC - (int) (140 * Math.sin(j * Math.PI / 18));
						AoBy02 = pfdYC - (int) (140 * Math.cos(j * Math.PI / 18));
					}
					bG2D.drawLine(AoBx01, AoBy01, AoBx02, AoBy02);
				}
				AoBx01 = pfdXC - (int) (130 * Math.sin(Math.PI / 4)); // 45
																		// degree
																		// AoB
				AoBy01 = pfdYC - (int) (130 * Math.cos(Math.PI / 4));
				AoBx02 = pfdXC - (int) (140 * Math.sin(Math.PI / 4));
				AoBy02 = pfdYC - (int) (140 * Math.cos(Math.PI / 4));
				bG2D.drawLine(AoBx01, AoBy01, AoBx02, AoBy02);
				AoBx01 = pfdXC - (int) (130 * Math.sin(-Math.PI / 4)); // 45
																		// degree
																		// AoB
				AoBy01 = pfdYC - (int) (130 * Math.cos(-Math.PI / 4));
				AoBx02 = pfdXC - (int) (140 * Math.sin(-Math.PI / 4));
				AoBy02 = pfdYC - (int) (140 * Math.cos(-Math.PI / 4));
				bG2D.drawLine(AoBx01, AoBy01, AoBx02, AoBy02);

				if (glideSlope || apprArmed) {
					bG2D.setColor(Color.black);
					bG2D.fillRect(pfdX + pfdWidth - 25, pfdY + 50, 25, 200);
					bG2D.setColor(Color.gray);
					bG2D.drawLine(pfdX + pfdWidth - 1, pfdY + 50, pfdX + pfdWidth - 1, pfdY + 250);
					bG2D.drawLine(pfdX + pfdWidth - 23, pfdY + 60, pfdX + pfdWidth - 3, pfdY + 60);
					bG2D.drawLine(pfdX + pfdWidth - 20, pfdY + 90, pfdX + pfdWidth - 6, pfdY + 90);
					bG2D.drawLine(pfdX + pfdWidth - 20, pfdY + 120, pfdX + pfdWidth - 6, pfdY + 120);
					bG2D.setColor(Color.white);
					bG2D.drawLine(pfdX + pfdWidth - 23, pfdY + 150, pfdX + pfdWidth - 3, pfdY + 150);// GP
					bG2D.setColor(Color.gray);
					bG2D.drawLine(pfdX + pfdWidth - 20, pfdY + 180, pfdX + pfdWidth - 6, pfdY + 180);
					bG2D.drawLine(pfdX + pfdWidth - 20, pfdY + 210, pfdX + pfdWidth - 6, pfdY + 210);
					bG2D.drawLine(pfdX + pfdWidth - 23, pfdY + 240, pfdX + pfdWidth - 3, pfdY + 240);
					bG2D.setColor(Color.magenta);
					int highlow = (int) (-gPError * 90 * 180.0 / Math.PI);
					highlow = Math.max(highlow, -100);
					highlow = Math.min(highlow, 100);
					highlow = highlow + pfdY + 150;
					gpDiamondX[0] = pfdX + pfdWidth - 19;
					gpDiamondY[0] = highlow;
					gpDiamondX[1] = pfdX + pfdWidth - 13;
					gpDiamondY[1] = highlow - 5;
					gpDiamondX[2] = pfdX + pfdWidth - 7;
					gpDiamondY[2] = highlow;
					gpDiamondX[3] = pfdX + pfdWidth - 13;
					gpDiamondY[3] = highlow + 5;
					bG2D.fillPolygon(gpDiamondX, gpDiamondY, 4);
				}
				// bG2D.setColor(Color.white);
				// bG2D.drawOval(pfdX, pfdY, pfdWidth, pfdWidth);
				if (llzArmed || llzActive) {
					bG2D.setColor(Color.black);
					bG2D.fillRect(pfdX + 50, pfdY + pfdWidth - 25, 200, 25);
					bG2D.setColor(Color.gray);
					bG2D.drawLine(pfdX + 50, pfdY + pfdWidth - 1, pfdX + 250, pfdY + pfdWidth - 1);
					bG2D.drawLine(pfdX + 60, pfdY + pfdWidth - 23, pfdX + 60, pfdY + pfdWidth - 3);
					// bG2D.drawLine(pfdX + 90, pfdY + pfdWidth - 20, pfdX + 90,
					// pfdY + pfdWidth - 6);
					bG2D.drawOval(pfdX + 88, pfdY + pfdWidth - 15, 4, 4);
					// bG2D.drawLine(pfdX + 120, pfdY + pfdWidth - 20, pfdX +
					// 120, pfdY + pfdWidth - 6);
					bG2D.drawOval(pfdX + 118, pfdY + pfdWidth - 15, 4, 4);
					bG2D.setColor(Color.white);
					bG2D.drawLine(pfdX + 150, pfdY + pfdWidth - 23, pfdX + 150, pfdY + pfdWidth - 3);// LLZ
					bG2D.setColor(Color.gray);
					// bG2D.drawLine(pfdX + 180, pfdY + pfdWidth - 20, pfdX +
					// 180, pfdY + pfdWidth - 6);
					bG2D.drawOval(pfdX + 178, pfdY + pfdWidth - 15, 4, 4);
					// bG2D.drawLine(pfdX + 210, pfdY + pfdWidth - 20, pfdX +
					// 210, pfdY + pfdWidth - 6);
					bG2D.drawOval(pfdX + 208, pfdY + pfdWidth - 15, 4, 4);
					bG2D.drawLine(pfdX + 240, pfdY + pfdWidth - 23, pfdX + 240, pfdY + pfdWidth - 3);
					bG2D.setColor(Color.magenta);
					int llzhighlow = (int) (llzError * 90 * 180.0 / Math.PI);
					llzhighlow = Math.max(llzhighlow, -100);
					llzhighlow = Math.min(llzhighlow, 100);
					llzhighlow = llzhighlow + pfdX + 150;
					llzDiamondY[0] = pfdY + pfdWidth - 19;
					llzDiamondX[0] = llzhighlow;
					llzDiamondY[1] = pfdY + pfdWidth - 13;
					llzDiamondX[1] = llzhighlow - 5;
					llzDiamondY[2] = pfdY + pfdWidth - 7;
					llzDiamondX[2] = llzhighlow;
					llzDiamondY[3] = pfdY + pfdWidth - 13;
					llzDiamondX[3] = llzhighlow + 5;
					bG2D.fillPolygon(llzDiamondX, llzDiamondY, 4);
				}
			} else {
				bG2D.setColor(Color.blue);
				bG2D.drawString("PFD Off", pfdXC - 25, pfdYC);
			} // if (pfdOn){

			bG2D.setColor(colonialWhite);
			bG2D.fill(bkG);
			// bG2D.fillRect(0, 0, 1220, 720);

			bG2D.setColor(Color.lightGray);
			bG2D.fill(ibkG);
			// bG2D.fillRect(0, 0, 1105, dL);

			// ----------------------------------------------------------------------------------------
			bG2D.setColor(Color.BLACK);
			bG2D.setFont(uac);
			bG2D.drawString("Underwood Aircraft Corporation", 830, 450);
			bG2D.setFont(stringFont);

			bG2D.drawLine(1105, 0, 1105, dL);
			bG2D.drawLine(0, dL, 1105, dL);
			bG2D.drawLine(1, 0, 1, dL);
			// bG2D.drawLine(0, 1, 1105, 1);

			bG2D.drawString("Elevator", 700, 620);
			eleBase = String.valueOf(autoPilot.sLAtt);
			if (eleBase.length() > 5)
				eleBase = eleBase.substring(0, 5);
			bG2D.setColor(Color.blue);
			bG2D.drawString(eleBase, 750, 620);
			bG2D.setColor(Color.BLACK);
			bG2D.drawString("undeze@gmail.com", 800, 620);

			bG2D.drawString("Y", 1085, 580);
			bG2D.drawString("P", 1085, 595);
			bG2D.drawString("R", 1085, 610);
			bG2D.drawString("alpha", 1100, 580);
			bG2D.drawString(sim2.Alpha, 1150, 580);
			bG2D.drawString("beta", 1100, 595);
			bG2D.drawString(sim2.Beta, 1150, 595);
			bG2D.drawString("gamma", 1100, 610);
			bG2D.drawString(sim2.Gamma, 1150, 610);

			bG2D.drawString("Bearing", 1100, 635);
			bG2D.drawString(Bearing, 1150, 635);

			// technique to rotate an array of x,y points around a particular
			// point.++++++++++++++++++++++
			bG2D.drawLine(780, 400, 780, 440);
			p1x = 760;
			p1y = 420;
			p2x = 800;
			p2y = 420;
			bG2D.drawLine((int) p1x, (int) p1y, (int) p2x, (int) p2y);
			// move to origin for rotation
			p1x = p1x - 780;
			p1y = p1y - 420;
			p2x = p2x - 780;
			p2y = p2y - 420;
			// rotation
			// define rotation matrix
			r11 = Math.cos(sim2.gammaQ);
			r12 = -Math.sin(sim2.gammaQ);
			r21 = Math.sin(sim2.gammaQ);
			r22 = Math.cos(sim2.gammaQ);
			// perform rotation
			n1x = (r11 * p1x) + (r12 * p1y);
			n1y = (r21 * p1x) + (r22 * p1y);
			n2x = (r11 * p2x) + (r12 * p2y);
			n2y = (r21 * p2x) + (r22 * p2y);
			// return to original position
			p1x = n1x + 780;
			p1y = n1y + 420;
			p2x = n2x + 780;
			p2y = n2y + 420;
			// redraw
			bG2D.drawLine((int) p1x, (int) p1y, (int) p2x, (int) p2y);// +++++++++++++++++++++++++++++++++++

			// ND
			// *************************************************************************************
			bG2D.fillRect(ndX - 30, ndY - 30, ndWidth + 60, ndWidth + 53);

			NDGraphics();

			if (RWGX[4] > ndX + 10 && RWGX[4] < ndX + ndWidth - 10 && RWGY[4] > ndY + 10
					&& RWGY[4] < ndY + ndWidth - 10) {
				bG2D.setColor(Color.yellow);
				bG2D.drawPolygon(RWGX, RWGY, 4); // runway pollygon on nd
			}

			bG2D.setColor(Color.black); // the square with the circle subtracted
										// from it
			bG2D.fill(ndR); // to cover the waypoints as the move of the nd.

			bG2D.setColor(Color.yellow);
			bG2D.drawLine(ndXC, ndYC - 10, ndXC, ndYC + 12);
			bG2D.drawLine(ndXC - 10, ndYC, ndXC + 10, ndYC);
			bG2D.drawLine(ndXC - 5, ndYC + 10, ndXC + 5, ndYC + 10);

			bG2D.setColor(Color.yellow);
			bG2D.fillRect(ndXC - 1, ndYC - 132, 3, 12); // yellow heading
														// pointer on nd
			bG2D.fillRect(ndXC - 1, ndYC + 120, 3, 12); // yellow tail pointer
			bG2D.fillRect(ndXC - 132, ndYC - 1, 12, 3);
			bG2D.fillRect(ndXC + 120, ndYC - 1, 12, 3);

			bG2D.setColor(Color.white);
			bG2D.drawOval(ndX + 40, ndY + 40, ndWidth - 80, ndWidth - 80);

			bG2D.setColor(Color.white);
			if (nav1On) {
				bG2D.drawLine((int) aX1, (int) aY1, (int) aX2, (int) aY2); // head
																			// of
																			// needle
				bG2D.drawLine((int) aXL, (int) aYL, (int) aX2, (int) aY2);
				bG2D.drawLine((int) aXR, (int) aYR, (int) aX2, (int) aY2);
				bG2D.drawLine((int) bX1, (int) bY1, (int) bX2, (int) bY2); // tail
																			// of
																			// needle
				bG2D.drawLine((int) bXL, (int) bYL, (int) bX2, (int) bY2);
				bG2D.drawLine((int) bXR, (int) bYR, (int) bX2, (int) bY2);
			}

			for (int j = -18; j <= 18; j++) {
				if (j % 3 == 0) {
					nd1DX = ndXC - (int) (120 * Math.sin(hdgND + j * Math.PI / 18));
					nd1DY = ndYC - (int) (120 * Math.cos(hdgND + j * Math.PI / 18));
					nd2DX = ndXC - (int) (132 * Math.sin(hdgND + j * Math.PI / 18));
					nd2DY = ndYC - (int) (132 * Math.cos(hdgND + j * Math.PI / 18));

					hdHdg = j * -10;
					if (hdHdg < 0)
						hdHdg = 360 + hdHdg;
					Hdg = String.valueOf(hdHdg);
					Hdg = "000".substring(Hdg.length()) + Hdg;

					if (rotateFont) {
						ndSX = ndXC - (int) (136 * Math.sin(0.09 + hdgND + j * Math.PI / 18));
						ndSY = ndYC - (int) (136 * Math.cos(0.09 + hdgND + j * Math.PI / 18));
						affineTransform.rotate(-(hdgND + j * Math.PI / 18), 0, 0);
						rotatedFont = font.deriveFont(affineTransform);
						bG2D.setFont(rotatedFont);
						affineTransform.rotate(hdgND + j * Math.PI / 18, 0, 0);
					} else {
						ndSX = ndXC - (int) (145 * Math.sin(hdgND + j * Math.PI / 18)) - 12;
						ndSY = ndYC - (int) (143 * Math.cos(hdgND + j * Math.PI / 18)) + 5;
					}
					bG2D.drawString(Hdg, (int) ndSX, (int) ndSY);
				} else {
					nd1DX = ndXC - (int) (120 * Math.sin(hdgND + j * Math.PI / 18));
					nd1DY = ndYC - (int) (120 * Math.cos(hdgND + j * Math.PI / 18));
					nd2DX = ndXC - (int) (128 * Math.sin(hdgND + j * Math.PI / 18));
					nd2DY = ndYC - (int) (128 * Math.cos(hdgND + j * Math.PI / 18));
				}
				bG2D.drawLine(nd1DX, nd1DY, nd2DX, nd2DY);
				// bG2D.drawRect(650,20,23,10);
				// bG2D.drawString("000",650,30);
			}
			bG2D.setFont(stringFont);

			bG2D.setColor(Color.green);
			// bG2D.setFont(largeBold);
			if (nav1On) {
				bG2D.drawString(ToPoint + " " + Range, ndX + 10, ndY + 25);
			}
			bG2D.drawString("GS " + sim2.GroundSpeed, ndX + 10, ndY + ndWidth - 15);
			bG2D.drawString(NDScale + " nm", ndX + ndWidth - 45, ndY + ndWidth - 20);
			// bG2D.setFont(stringFont);VOR

			// heading bug
			if (llzActive) {
				NDhdgBug = mouseWheelHeading;
			} else {
				NDhdgBug = mouseWheelHDG;
			}
			bG2D.setColor(Color.pink);
			hdgBugX[0] = ndXC - (int) (121 * Math.sin(hdgND - (NDhdgBug * Math.PI / 180)));
			hdgBugX[1] = ndXC - (int) (130 * Math.sin(hdgND + 0.04 - (NDhdgBug * Math.PI / 180)));
			hdgBugX[2] = ndXC - (int) (130 * Math.sin(hdgND - 0.04 - (NDhdgBug * Math.PI / 180)));
			hdgBugY[0] = ndYC - (int) (121 * Math.cos(hdgND - (NDhdgBug * Math.PI / 180)));
			hdgBugY[1] = ndYC - (int) (130 * Math.cos(hdgND + 0.04 - (NDhdgBug * Math.PI / 180)));
			hdgBugY[2] = ndYC - (int) (130 * Math.cos(hdgND - 0.04 - (NDhdgBug * Math.PI / 180)));
			bG2D.fillPolygon(hdgBugX, hdgBugY, 3);

			// ***************************************************************************************

			// airspeed, m/s
			bG2D.setColor(Color.black);
			bG2D.drawString("IAS", vVX, 600);
			bG2D.drawString("m/s", vVX + 50, 600);
			bG2D.setColor(Color.blue);
			bG2D.drawString(String.valueOf((int) sim2.ias), vVX + 20, 600);

			// pfd airspeed knots
			bG2D.setColor(Color.black);
			bG2D.fillRect(215, 0 + 50, 35, pfdY + pfdWidth - 40);
			// barbers pole speed limit max ias
			airspeed = pfdY + 90 - (410 * 6) + (int) (sim2.iasKnots * 6); // 410
																			// knots
			airspeed = Math.min(airspeed, 300); // to stop the barbers pole
												// appearing
			bG2D.setColor(Color.red); // below the pfd (at high speed).
			bG2D.fillRect(pfdX - 4, pfdY, 4, airspeed); // end of barbers pole
			bG2D.setColor(Color.white);
			bG2D.drawLine(pfdX - 35, pfdY + 1, pfdX - 35, pfdY + pfdWidth);
			for (int j = 0; j <= 550; j = j + 10) {
				airspeed = pfdY + 155 - (j * 6) + (int) (sim2.iasKnots * 6);
				if (airspeed > 0 && airspeed < pfdY + pfdWidth + 10 && airspeed > pfdY) {
					Airspeed = String.valueOf(j);
					bG2D.drawString(Airspeed, 220, airspeed);
					bG2D.drawLine(215, airspeed - 5, 218, airspeed - 5);
					bG2D.drawLine(pfdX - 4, airspeed - 5, pfdX, airspeed - 5);
					bG2D.drawLine(pfdX - 4, airspeed - 35, pfdX, airspeed - 35);
				}
			}
			bG2D.setColor(Color.black);
			bG2D.fillRect(pfdX - 45, 30, 10, 361);

			bG2D.setColor(Color.white);
			commandedSpeed = pfdY + 150 + (int) (sim2.iasKnots * 6) - (mouseWheelSpeed * 6);
			if (speedOn && commandedSpeed < pfdY + pfdWidth + 10 && commandedSpeed > pfdY) {
				bG2D.setColor(pink);
				bG2D.drawLine(pfdX - 45, commandedSpeed - 6, pfdX - 45, commandedSpeed + 6);
				bG2D.drawLine(pfdX - 44, commandedSpeed - 6, pfdX - 44, commandedSpeed + 5);
				bG2D.drawLine(pfdX - 45, commandedSpeed - 5, pfdX - 35, commandedSpeed);
				bG2D.drawLine(pfdX - 45, commandedSpeed - 6, pfdX - 35, commandedSpeed - 1);
				bG2D.drawLine(pfdX - 45, commandedSpeed + 5, pfdX - 35, commandedSpeed);
				bG2D.drawLine(pfdX - 45, commandedSpeed + 4, pfdX - 35, commandedSpeed - 1);
			}

			bG2D.setColor(Color.yellow);
			bG2D.fillPolygon(iasXLeft, iasYLeft, 3);
			bG2D.fillPolygon(iasXRight, iasYRight, 3);

			// pfd altitude feet
			bG2D.setColor(Color.black);
			bG2D.fillRect(pfdX + pfdWidth, 0 + 50, 45, pfdY + pfdWidth - 40);
			bG2D.setColor(Color.gray);
			bG2D.fillRect(pfdX + pfdWidth, pfdY + (pfdWidth / 2) - 10, 45, 20);
			bG2D.setColor(Color.yellow);
			for (int j = 0; j <= 50000; j = j + 100) {
				altitude = pfdY + 155 - j + (int) (sim2.altitudeFeet);

				if (altitude > 0 && altitude < pfdY + pfdWidth + 10 && altitude > pfdY) {
					if (j == mouseWheelAltitude) {
						bG2D.setColor(Color.white);
					}
					Altitude = String.valueOf(j);
					bG2D.drawString(Altitude, pfdX + pfdWidth + 2, altitude);
					bG2D.setColor(Color.yellow);
				}
			}
			// ground on altimeter
			grd = pfdY + 155 + (int) (sim2.altitudeFeet);
			bG2D.setColor(altGrd); //
			bG2D.fillRect(pfdX + pfdWidth + 15, grd, 10, pfdY + pfdWidth - grd); //

			bG2D.setColor(Color.black);
			bG2D.fillRect(pfdX - 35, 0 + 30, pfdWidth + 80, 30);// to cover
																// altitude &
																// airspeed
																// appearing
																// above pfd
			bG2D.fillRect(0, 20, 638, 10);

			// vertical speed
			bG2D.setColor(Color.black);
			bG2D.fillRect(595, pfdYC - 180, 43, pfdWidth + 61);
			bG2D.setColor(Color.darkGray);
			// bG2D.fillRect(610,pfdYC-150,15,pfdWidth);
			bG2D.fillPolygon(VSX, VSY, 4);
			bG2D.setColor(Color.white);
			bG2D.drawLine(600, pfdYC, 610, pfdYC); // 0 ft/min
			bG2D.drawLine(600, pfdYC + 1, 610, pfdYC + 1); // 0 ft/min
			bG2D.drawLine(608, pfdYC - 25, 610, pfdYC - 25); // + 500 ft/min
			bG2D.drawLine(607, pfdYC - 50, 610, pfdYC - 50); // +1000 ft/min
			bG2D.drawString("1", 600, pfdYC - 45);
			bG2D.drawLine(608, pfdYC - 73, 610, pfdYC - 73); // +1500 ft/min
			bG2D.drawLine(607, pfdYC - 90, 610, pfdYC - 90); // +2000 ft/min
			bG2D.drawString("2", 600, pfdYC - 85);
			bG2D.drawLine(608, pfdYC - 127, 610, pfdYC - 127); // +4000 ft/min
			bG2D.drawLine(607, pfdYC - 150, 610, pfdYC - 150); // +6000 ft/min
			bG2D.drawString("6", 600, pfdYC - 145);
			bG2D.drawLine(608, pfdYC + 25, 610, pfdYC + 25); // - 500 ft/min
			bG2D.drawLine(607, pfdYC + 50, 610, pfdYC + 50); // -1000 ft/min
			bG2D.drawString("1", 600, pfdYC + 55);
			bG2D.drawLine(608, pfdYC + 73, 610, pfdYC + 73); // -1500 ft/min
			bG2D.drawLine(607, pfdYC + 90, 610, pfdYC + 90); // -2000 ft/min
			bG2D.drawString("2", 600, pfdYC + 95);
			bG2D.drawLine(608, pfdYC + 127, 610, pfdYC + 127); // -4000 ft/min
			bG2D.drawLine(607, pfdYC + 150, 610, pfdYC + 150); // -6000 ft/min
			bG2D.drawString("6", 600, pfdYC + 155);
			bG2D.drawLine(595, pfdYC - 150, 595, pfdYC + 150); // vertical
																// dividing line
			bG2D.setColor(Color.yellow);
			if (vs > 0 && vs <= 1000) {
				vsIY1 = pfdYC - (int) (vs / 20.0);
			}
			if (vs > 1000 && vs <= 6000) {
				vsIY1 = pfdYC - (int) ((55.702 * Math.log(vs)) - 334.25);
			}
			if (vs > 6000) {
				vsIY1 = pfdYC - (pfdWidth / 2);
			}
			if (vs > 100)
				bG2D.drawString(VS, (int) vsIX1 + 5, (int) vsIY1 - 2);
			if (vs < 0 && vs >= -1000) {
				vsIY1 = pfdYC - (int) (vs / 20.0);
			}
			if (vs < -1000 && vs >= -6000) {
				vsIY1 = pfdYC + (int) ((55.702 * Math.log(-vs)) - 334.25);
			}
			if (vs < -6000) {
				vsIY1 = pfdYC + (pfdWidth / 2);
			}
			if (vs < -100)
				bG2D.drawString(VS, (int) vsIX1 + 2, (int) vsIY1 + 12);
			verticalSpeedIntercept();
			bG2D.drawLine((int) vsIX1, (int) vsIY1, (int) vsIX, (int) vsIY); // vertical
																				// speed
																				// needle
			bG2D.drawLine((int) vsIX1, (int) vsIY1 + 1, (int) vsIX, (int) vsIY + 1);

			// aileron position
			bG2D.setColor(Color.black);
			bG2D.fillRect(ailRudX, 250, 135, 60);
			bG2D.setColor(Color.white);
			ctlInt = (int) (ailPosReq * 10);
			ctlInt = Math.min(50, ctlInt);
			ctlInt = Math.max(-50, ctlInt);
			ailPR = ailRudX + 70 + ctlInt;
			bG2D.drawLine(ailPR, 289, ailPR + 3, 286);
			bG2D.drawLine(ailPR, 289, ailPR - 3, 286);
			bG2D.drawLine(ailPR + 3, 286, ailPR - 3, 286);
			bG2D.setColor(Color.green);
			bG2D.drawString("Aileron", ailRudX + 50, 265);
			bG2D.drawLine(ailRudX + 20, 300, ailRudX + 120, 300);
			bG2D.drawLine(ailRudX + 70, 300, ailRudX + 70, 304);
			ctlInt = (int) (ailPos * 10);
			ctlInt = Math.min(50, ctlInt);
			ctlInt = Math.max(-50, ctlInt);
			bG2D.drawLine(ailRudX + 70 + ctlInt, 300, ailRudX + 70 + ctlInt, 290);
			AilPos = String.valueOf(ailPos);
			if (AilPos.length() > 5)
				AilPos = AilPos.substring(0, 5);
			bG2D.drawString(AilPos, ailRudX + 50, 277);

			// elevator position
			bG2D.setColor(Color.black);
			bG2D.fillRect(eleX, eleY, 40, 141);
			bG2D.setColor(Color.white);
			ctlInt = (int) (elePosReq * 8);
			ctlInt = Math.min(50, ctlInt);
			ctlInt = Math.max(-50, ctlInt);
			elePR = eleY + 70 + ctlInt;
			bG2D.drawLine(eleX + 20, elePR, eleX + 23, elePR + 3);
			bG2D.drawLine(eleX + 20, elePR, eleX + 23, elePR - 3);
			bG2D.drawLine(eleX + 23, elePR - 3, eleX + 23, elePR + 3);
			bG2D.setColor(Color.green);
			bG2D.drawString("Elevator", eleX + 5, 265);
			bG2D.drawLine(eleX + 10, 270, eleX + 10, 370);
			bG2D.drawLine(eleX + 6, 320, eleX + 10, 320);
			ctlInt = (int) (elePos * 8);
			ctlInt = Math.min(50, ctlInt);
			ctlInt = Math.max(-50, ctlInt);
			eleP = eleY + 70 + ctlInt;
			bG2D.drawLine(eleX + 10, eleP, eleX + 20, eleP);
			ElePos = String.valueOf(elePos);
			if (ElePos.length() > 5)
				ElePos = ElePos.substring(0, 5);
			bG2D.drawString(ElePos, eleX + 25, 277);

			// rudder position
			bG2D.setColor(Color.black);
			bG2D.fillRect(ailRudX, 310, 135, 81);
			bG2D.setColor(Color.white);
			ctlInt = (int) (rudPosReq * 2);
			ctlInt = Math.min(50, ctlInt);
			ctlInt = Math.max(-50, ctlInt);
			rudPR = ailRudX + 70 + ctlInt;
			bG2D.drawLine(rudPR, 339, rudPR + 3, 336);
			bG2D.drawLine(rudPR, 339, rudPR - 3, 336);
			bG2D.drawLine(rudPR + 3, 336, rudPR - 3, 336);
			bG2D.setColor(Color.green);
			bG2D.drawString("Rudder", ailRudX + 50, 385);
			bG2D.drawLine(ailRudX + 70, 350, ailRudX + 70, 354);
			bG2D.drawLine(ailRudX + 20, 350, ailRudX + 120, 350);
			ctlInt = (int) (rudPos * 2);
			ctlInt = Math.min(50, ctlInt);
			ctlInt = Math.max(-50, ctlInt);
			bG2D.drawLine(ailRudX + 70 + ctlInt, 350, ailRudX + 70 + ctlInt, 340);
			RudPos = String.valueOf(rudPos);
			bG2D.drawString(RudPos, ailRudX + 50, 372);

			// AoA strip
			bG2D.setColor(Color.black);
			bG2D.fillRect(gStripX - 5, 30, 50, 361); // AoA, G
			// bG2D.setColor(Color.darkGray);
			// bG2D.fillRect(aoaX+5, 55, 10, 301);
			bG2D.setColor(Color.green);
			bG2D.setFont(largeFont);
			bG2D.drawString("α", aoaX + 3, 45);// αα
			bG2D.setFont(smallFont);
			bG2D.drawString("max", aoaX - 5, 98);
			aoa = (int) (sim2.AoAq * 10);
			if (aoa <= 105 && aoa >= -195)
				bG2D.drawLine(aoaX + 10, 250 + aoa, aoaX + 15, 250 + aoa);

			if (aoa < -195) {
				bG2D.drawLine(aoaX + 15, 250 - 195, aoaX + 18, 250 - 192);
				bG2D.drawLine(aoaX + 15, 250 - 195, aoaX + 12, 250 - 192);
			}
			if (aoa > 105) {
				bG2D.drawLine(aoaX + 15, 250 + 105, aoaX + 18, 250 + 102);
				bG2D.drawLine(aoaX + 15, 250 + 105, aoaX + 12, 250 + 102);
			}
			aoa = Math.min(105, aoa);
			aoa = Math.max(-195, aoa);
			bG2D.drawLine(aoaX + 15, 250, aoaX + 15, 250 + aoa);

			bG2D.drawLine(aoaX + 5, 250, aoaX + 10, 250); // 0 degree AOA
			bG2D.drawLine(aoaX + 5, 100, aoaX + 10, 100); // 15 degree AOA
			bG2D.setFont(stringFont);

			// roll leftAoA, rightAoA strip
			bG2D.setColor(Color.black);
			bG2D.fillRect(rollAoAStrip - 10, 30, 55, 361);
			bG2D.setColor(Color.red);
			bG2D.setFont(smallFont);
			bG2D.drawString("Ailerons", rollAoAStrip - 2, 40);
			bG2D.setColor(Color.green);
			bG2D.drawString("Wing tips", rollAoAStrip - 4, 55);
			bG2D.setFont(stringFont);
			rAoA = (int) (sim2.rightAoA * 10);
			lAoA = (int) (sim2.leftAoA * 10);
			rAoA = Math.min(105, rAoA);
			rAoA = Math.max(-195, rAoA);
			lAoA = Math.min(105, lAoA);
			lAoA = Math.max(-195, lAoA);
			bG2D.drawLine(rollAoAStrip + 30, 250, rollAoAStrip + 30, 250 + rAoA);
			bG2D.drawLine(rollAoAStrip + 20, 250 + rAoA, rollAoAStrip + 30, 250 + rAoA);
			bG2D.drawLine(rollAoAStrip + 15, 250, rollAoAStrip + 20, 250); // 0
																			// degree
																			// AOA
			bG2D.drawLine(rollAoAStrip + 15, 100, rollAoAStrip + 20, 100); // 15
																			// degree
																			// AOA
			bG2D.drawLine(rollAoAStrip + 5, 250, rollAoAStrip + 5, 250 + lAoA);
			bG2D.drawLine(rollAoAStrip + 5, 250 + lAoA, rollAoAStrip + 15, 250 + lAoA);
			bG2D.setColor(Color.white);
			qAoA = (int) (sim2.AoAq * 10);
			qAoA = Math.min(105, qAoA);
			qAoA = Math.max(-195, qAoA);
			bG2D.drawLine(rollAoAStrip + 15, 250 + qAoA, rollAoAStrip + 20, 250 + qAoA);
			bG2D.drawLine(rollAoAStrip - 10, 250 + qAoA, rollAoAStrip - 5, 250 + qAoA);
			bG2D.drawLine(rollAoAStrip + 40, 250 + qAoA, rollAoAStrip + 45, 250 + qAoA);
			bG2D.setColor(Color.red);
			rAilP = (int) ((sim2.rightAoA + sim2.rightAileronPosition) * 10);
			lAilP = (int) ((sim2.leftAoA + sim2.leftAileronPosition) * 10);
			rAilP = Math.min(105, rAilP);
			rAilP = Math.max(-195, rAilP);
			lAilP = Math.min(105, lAilP);
			lAilP = Math.max(-195, lAilP);
			bG2D.drawLine(rollAoAStrip - 5, 250 + lAilP, rollAoAStrip + 5, 250 + lAilP);
			bG2D.drawLine(rollAoAStrip + 30, 250 + rAilP, rollAoAStrip + 40, 250 + rAilP);
			bG2D.setColor(Color.gray);
			bG2D.drawString("Roll 2", rollAoAStrip + 3, 368);
			bG2D.drawString("AoA", rollAoAStrip + 6, 382);
			if (sim2.rollMode != 2) {
				bG2D.setColor(Color.white);
				bG2D.drawString("OFF", rollAoAStrip + 6, 330);
			}

			// throttle position
			bG2D.setColor(Color.black);
			bG2D.fillRect(tPX - 40, tPY - 20, 155, 220);
			bG2D.setColor(Color.gray);
			bG2D.drawLine(tPX + 85, tPY + 10, tPX + 85, tPY + 135);
			bG2D.setColor(Color.white);
			bG2D.drawString("Thrust", tPX + 12, tPY + 22);
			bG2D.drawString("100%", tPX + 37, tPY + 35);
			bG2D.drawString(" 50%", tPX + 35, tPY + 85);
			bG2D.drawString("  0%", tPX + 35, tPY + 135);
			bG2D.setColor(Color.lightGray);
			bG2D.fillRect(tPX + 23, tPY + 30 + (100 - (int) actThrust), 6, (int) actThrust);
			bG2D.setColor(Color.white);
			bG2D.drawLine(tPX + 20, tPY + 130, tPX + 32, tPY + 130);
			bG2D.drawLine(tPX + 20, tPY + 30, tPX + 32, tPY + 30);
			bG2D.drawLine(tPX + 20, tPY + 80, tPX + 23, tPY + 80);
			if (speedOn) {
				bG2D.setColor(Color.magenta);
			} else {
				bG2D.setColor(Color.blue);
			}
			bG2D.drawLine(tPX + 20, tPY + 30 + (100 - throttlePosition), tPX + 32, tPY + 30 + (100 - throttlePosition));
			bG2D.drawLine(tPX + 17, tPY + 30 + (100 - throttlePosition) - 3, tPX + 17,
					tPY + 30 + (100 - throttlePosition) + 3);
			bG2D.drawLine(tPX + 17, tPY + 30 + (100 - throttlePosition) - 3, tPX + 20,
					tPY + 30 + (100 - throttlePosition));
			bG2D.drawLine(tPX + 17, tPY + 30 + (100 - throttlePosition) + 3, tPX + 20,
					tPY + 30 + (100 - throttlePosition));
			bG2D.drawLine(tPX + 35, tPY + 30 + (100 - throttlePosition) - 3, tPX + 35,
					tPY + 30 + (100 - throttlePosition) + 3);
			bG2D.drawLine(tPX + 35, tPY + 30 + (100 - throttlePosition) - 3, tPX + 32,
					tPY + 30 + (100 - throttlePosition));
			bG2D.drawLine(tPX + 35, tPY + 30 + (100 - throttlePosition) + 3, tPX + 32,
					tPY + 30 + (100 - throttlePosition));
			if (sim2.positionZ < -0.1 && actThrust == 0) {
				bG2D.setColor(Color.red);
				bG2D.drawString("THRUST ZERO", tPX + 0, tPY + 150);
			}

			// G strip
			bG2D.setColor(Color.darkGray);
			bG2D.fillRect(gStripX + 5, 55, 10, 301);
			bG2D.setColor(Color.white);
			bG2D.drawString("G", gStripX - 13, 45);
			sim2g = sim2.gQ;
			g = (int) (sim2g * 30);
			g = Math.min(195, g);
			g = Math.max(-105, g);
			if (sim2g > 6 || sim2g < -3)
				bG2D.setColor(Color.red);
			bG2D.drawString(sim2.gString, gStripX, 45);
			bG2D.drawLine(gStripX + 15, 250, gStripX + 15, 250 - g);
			bG2D.drawLine(gStripX + 10, 250 - g, gStripX + 15, 250 - g);
			bG2D.setColor(Color.white);
			bG2D.drawLine(gStripX + 5, 340, gStripX + 10, 340); // -3G
			bG2D.drawString("-3", gStripX - 6, 345);
			bG2D.drawLine(gStripX + 5, 280, gStripX + 10, 280); // -1G
			bG2D.drawString("-1", gStripX - 6, 285);
			bG2D.drawLine(gStripX + 5, 250, gStripX + 10, 250); // 0G
			bG2D.drawString("0", gStripX - 3, 255);
			bG2D.drawLine(gStripX + 5, 220, gStripX + 10, 220); // 1G
			bG2D.drawString("1", gStripX - 3, 225);
			bG2D.drawLine(gStripX + 5, 130, gStripX + 10, 130); // 4G
			bG2D.drawString("4", gStripX - 3, 135);
			bG2D.drawLine(gStripX + 5, 70, gStripX + 10, 70); // 6G
			bG2D.drawString("6", gStripX - 3, 75);

			if (drawData) {
				drawThrustVector(bG2D);
				drawVelocityVector(bG2D);
				drawSideSlip(bG2D);
				drawAircraftMatrix(bG2D);
				drawBodyRates(bG2D);
				drawForces(bG2D);
				drawPosition(bG2D);
			}

			drawGearPanel(bG2D);

			// reset button panel
			bG2D.setColor(Color.lightGray);
			bG2D.fillRect(resetPX, resetPY, 250, 100);
			bG2D.setColor(Color.black);
			bG2D.drawString("Reset", 110, resetPY + 15);

			// reset button
			bG2D.setColor(button);
			bG2D.fillRect(resetX, resetY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("10K", resetX + 12, resetY + 20);

			// stop button
			bG2D.setColor(button);
			bG2D.fillRect(stopX, stopY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Stop", stopX + 15, stopY + 20);

			// Boy/Girl display button
			bG2D.setColor(button);
			bG2D.fillRect(boyX, boyY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Boy/Girl", boyX + 5, boyY + 20);

			// Data button
			bG2D.setColor(button);
			bG2D.fillRect(dataX, dataY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Data", dataX + 15, dataY + 20);

			// rotate font button
			bG2D.setColor(button);
			bG2D.fillRect(fontX, fontY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Font", fontX + 10, fontY + 20);

			// projection visuals button
			bG2D.setColor(button);
			bG2D.fillRect(visX, visY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("3D Proj", visX + 4, visY + 20);
			if (runwayOn) {
				bG2D.setColor(Color.yellow);
				bG2D.drawRect(visX + 1, visY + 1, 52, 27);
			}

			// pfd On button
			bG2D.setColor(button);
			bG2D.fillRect(pfdOnX, pfdOnY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("PFD", pfdOnX + 15, pfdOnY + 20);

			// reset north landing button
			bG2D.setColor(button);
			bG2D.fillRect(resLNX, resLNY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.setFont(smallFont);
			bG2D.drawString("LDG RWY36", resLNX + 1, resLNY + 20);

			// reset 10nm final north landing button
			bG2D.setColor(button);
			bG2D.fillRect(res10X, res10Y, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.setFont(smallFont);
			bG2D.drawString("RWY36 10m", res10X + 1, res10Y + 20);

			// reset south landing button
			bG2D.setColor(button);
			bG2D.fillRect(resLSX, resLSY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("LDG RWY18", resLSX + 1, resLSY + 20);

			// reset takeoff RWY36
			bG2D.setColor(button);
			bG2D.fillRect(resTOX, resTOY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("TO RWY36 ", resTOX + 3, resTOY + 20);
			bG2D.setFont(stringFont);

			// reset glide button
			bG2D.setColor(button);
			bG2D.fillRect(resGX, resGY, 55, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Glide", resGX + 10, resGY + 20);

			// nav1needle on/off button
			bG2D.setColor(button);
			bG2D.fillRect(nav1X, nav1Y, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.setFont(smallFont);
			bG2D.drawString("NAV1", nav1X + 3, nav1Y + 20);
			bG2D.setFont(stringFont);
			if (nav1On) {
				bG2D.setColor(Color.yellow);
				bG2D.drawRect(nav1X + 1, nav1Y + 1, 27, 27);
			}

			// direct to NAV 1 button
			bG2D.setColor(button);
			bG2D.fillRect(drt1X, drt1Y, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.setFont(smallFont);
			bG2D.drawString("DRT1", drt1X + 3, drt1Y + 20);
			bG2D.setFont(stringFont);

			// waypoint select button
			bG2D.setColor(button);
			bG2D.fillRect(wpSelX, wpSelY, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.setFont(smallFont);
			bG2D.drawString("WP", wpSelX + 10, wpSelY + 20);
			bG2D.setFont(stringFont);

			// scale 12 button on nd
			bG2D.setColor(button);
			bG2D.fillRect(rngPX, rngPY, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("R +", rngPX + 3, rngPY + 20);
			// scale 30 button on nd
			bG2D.setColor(button);
			bG2D.fillRect(rngMX, rngMY, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("R -", rngMX + 3, rngMY + 20);

			// flight director button
			bG2D.setColor(button);
			bG2D.fillRect(fdBX, fdBY, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("FD", fdBX + 7, fdBY + 20);
			if (fdOn) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(fdBX + 1, fdBY + 1, 27, 27);
			}

			// auto pilot button apOn
			bG2D.setColor(button);
			bG2D.fillRect(apBX, apBY, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("AP", apBX + 8, apBY + 20);
			if (apOn) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(apBX + 1, apBY + 1, 27, 27);
			}

			// velocity vector button vvOn
			bG2D.setColor(button);
			bG2D.fillRect(vvBX, vvBY, 30, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("V V", vvBX + 5, vvBY + 20);
			if (vvOn) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(vvBX + 1, vvBY + 1, 27, 27);
			}

			// spoiler button
			bG2D.setColor(button);
			bG2D.fillRect(spX, spY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Spoilers", spX + 8, spY + 20);
			if (spoilersOn) {
				bG2D.setColor(Color.red);
				bG2D.drawRect(spX + 1, spY + 1, 62, 27);
				bG2D.drawRect(spX + 3, spY + 3, 58, 23);
				bG2D.drawString("SPOILERS", pfdX + 10, pfdY + 20);
			}

			// heading select button
			bG2D.setColor(button);
			bG2D.fillRect(hdgBX, hdgBY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("HDG", hdgBX + 20, hdgBY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(hdgBX, hdgBY + 30, 65, 22);
			// if(!llzActive){
			bG2D.setColor(yellowy);
			bG2D.drawString(MouseWheelHDG, hdgBX + 22, hdgBY + 45);
			// }

			if (hdgOn) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(hdgBX + 1, hdgBY + 1, 62, 27);
				bG2D.drawRect(hdgBX + 3, hdgBY + 3, 58, 23);
				bG2D.drawString("HDG", pfdX + 225, pfdY - 20);
			}
			if (timeDiff < adjustTime && changeParameter.equals("HDG")) {
				bG2D.setColor(yellowy);
				bG2D.drawRect(hdgBX + 1, hdgBY + 31, 62, 19);
			}

			// LLZ button
			bG2D.setColor(button);
			bG2D.fillRect(LLZX, LLZY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("LLZ", LLZX + 20, LLZY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(LLZX, LLZY + 30, 65, 22);
			if (llzActive) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(LLZX + 1, LLZY + 1, 62, 27);
				bG2D.drawRect(LLZX + 3, LLZY + 3, 58, 23);
				bG2D.drawString("LLZ", pfdX + 225, pfdY - 20);
			}
			if (llzArmed) {
				bG2D.setColor(Color.cyan);
				bG2D.drawString("Armed", LLZX + 13, LLZY + 45);
				bG2D.drawString("LLZ", pfdX + 235, pfdY - 5);
			}

			// approach guidance button
			bG2D.setColor(button);
			bG2D.fillRect(ApprX, ApprY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("GS", ApprX + 25, ApprY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(ApprX, ApprY + 30, 65, 22);
			if (glideSlope) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(ApprX + 1, ApprY + 1, 62, 27);
				bG2D.drawRect(ApprX + 3, ApprY + 3, 58, 23);
			}
			if (apprArmed) {
				bG2D.setColor(Color.cyan);
				bG2D.drawString("Armed", ApprX + 13, ApprY + 45);
				bG2D.drawString("GS", pfdXC - 5, pfdY - 5);
			}

			// bG2D.setColor(Color.black);
			// bG2D.drawString("GPR " + GPR, ApprX+70, ApprY+15);
			// bG2D.drawString("CGP " + CGP, ApprX+70, ApprY+30);
			// bG2D.drawString("GPE " + GPE, ApprX+70, ApprY+45);
			// bG2D.drawString("FPR " + FPR, ApprX+70, ApprY+60);

			// attitude mode button
			bG2D.setColor(button);
			bG2D.fillRect(attHX, attHY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("ATT MODE", attHX + 3, attHY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(attHX, attHY + 30, 65, 22);
			bG2D.setColor(yellowy);
			bG2D.drawString("Mode " + autoPilot.ATTHoldMode, attHX + 10, attHY + 45);

			// flame out button
			bG2D.setColor(Color.darkGray);
			bG2D.fillRect(fmOX, fmOY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Flameout", fmOX + 5, fmOY + 20);
			if (glideOn) {
				bG2D.setColor(Color.red);
				bG2D.drawRect(fmOX + 1, fmOY + 1, 62, 27);
				bG2D.drawRect(fmOX + 3, fmOY + 3, 58, 23);
			}

			// Speed button
			bG2D.setColor(button);
			bG2D.fillRect(sButtX, sButtY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("SPEED", sButtX + 12, sButtY + 20);
			// mouseWheelSpeed
			bG2D.setColor(Color.black);
			bG2D.fillRect(sButtX, sButtY + 30, 65, 22);
			bG2D.setColor(yellowy);
			bG2D.drawString(MouseWheelSpeed + " kts", sButtX + 13, sButtY + 45);

			// ATT HOLD button
			bG2D.setColor(button);
			bG2D.fillRect(ATTHButtX, ATTHButtY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("ATT HOLD", ATTHButtX + 4, ATTHButtY + 20);
			// mouseWheelAttitude
			bG2D.setColor(Color.black);
			bG2D.fillRect(ATTHButtX, ATTHButtY + 30, 65, 22);
			bG2D.setColor(yellowy);
			if (!glideOn)
				bG2D.drawString(MouseWheelAttitude + "°", ATTHButtX + 15, ATTHButtY + 45);
			if (glideOn)
				bG2D.drawString("GLIDE", ATTHButtX + 15, ATTHButtY + 45);

			// roll mode button
			bG2D.setColor(button);
			bG2D.fillRect(rollMX, rollMY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("Roll MODE", rollMX + 3, rollMY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(rollMX, rollMY + 30, 65, 22);
			bG2D.setColor(yellowy);
			bG2D.drawString(sim2.RollMode, rollMX + 1, rollMY + 45);

			// altitude hold button
			bG2D.setColor(button);
			bG2D.fillRect(ALTButtX, ALTButtY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("ALT", ALTButtX + 20, ALTButtY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(ALTButtX, ALTButtY + 30, 65, 22);
			bG2D.setColor(yellowy);
			if (altArmed) {
				bG2D.setColor(Color.cyan);
				bG2D.drawString("ALT", pfdXC + 20, pfdY - 5);
				bG2D.drawString(MouseWheelAltitude, pfdX + pfdWidth + 2, pfdY + pfdWidth + 15);
			}
			bG2D.drawString(MouseWheelAltitude + " ft", ALTButtX + 10, ALTButtY + 45);

			// altitude mode button
			bG2D.setColor(button);
			bG2D.fillRect(altBX, altBY, 65, 30);
			bG2D.setColor(Color.white);
			bG2D.drawString("ALT MODE", altBX + 4, altBY + 20);
			bG2D.setColor(Color.black);
			bG2D.fillRect(altBX, altBY + 30, 65, 22);
			bG2D.setColor(yellowy);
			bG2D.drawString("Mode " + autoPilot.ALTHoldMode, altBX + 10, altBY + 45);

			// PFD heading indicator
			bG2D.setColor(Color.black);
			bG2D.fillRect(250, 311 + 50, 301, 30);
			bG2D.setColor(Color.green);
			bG2D.drawLine(260, 335 + 50, 540, 335 + 50);
			bG2D.drawLine(400, 335 + 50, 400, 340 + 50);
			bG2D.drawLine(400 - pixelOffset, 327 + 50, 400 - pixelOffset, 335 + 50);// 10
																					// degree
																					// mark
			bG2D.drawString(H1, 390 - pixelOffset, 325 + 50);
			bG2D.drawLine(400 - pixelOffset - 30, 331 + 50, 400 - pixelOffset - 30, 335 + 50);// 5
																								// degree
																								// mark
			bG2D.drawLine(400 - pixelOffset - 90, 331 + 50, 400 - pixelOffset - 90, 335 + 50);// 5
																								// degree
																								// mark
			bG2D.drawLine(400 - pixelOffset - 60, 327 + 50, 400 - pixelOffset - 60, 335 + 50);// 10
																								// degree
																								// mark
			bG2D.drawString(H2, 390 - pixelOffset - 60, 325 + 50);
			bG2D.drawLine(400 - pixelOffset + 30, 331 + 50, 400 - pixelOffset + 30, 335 + 50);// 5
																								// degree
																								// mark
			bG2D.drawLine(400 - pixelOffset + 60, 327 + 50, 400 - pixelOffset + 60, 335 + 50);// 10
																								// degree
																								// mark
			bG2D.drawString(H4, 390 - pixelOffset + 60, 325 + 50);
			if (400 - pixelOffset - 120 > pfdX) {
				bG2D.drawLine(400 - pixelOffset - 120, 327 + 50, 400 - pixelOffset - 120, 335 + 50);// 10
																									// degree
																									// mark
				bG2D.drawString(H3, 390 - pixelOffset - 120, 325 + 50);
			}
			bG2D.drawLine(400 - pixelOffset + 90, 331 + 50, 400 - pixelOffset + 90, 335 + 50);// 5
																								// degree
																								// mark
			bG2D.drawLine(400 - pixelOffset + 120, 327 + 50, 400 - pixelOffset + 120, 335 + 50);// 10
																								// degree
																								// mark
			bG2D.drawLine(400 - pixelOffset + 150, 331 + 50, 400 - pixelOffset + 150, 335 + 50);// 5
																								// degree
																								// mark
			bG2D.drawString(H5, 390 - pixelOffset + 120, 325 + 50);
			if (400 - pixelOffset + 180 < pfdX + pfdWidth) {
				bG2D.drawLine(400 - pixelOffset + 180, 327 + 50, 400 - pixelOffset + 180, 335 + 50);// 10
																									// degree
																									// mark
				bG2D.drawString(H6, 390 - pixelOffset + 180, 325 + 50);
			}

			bG2D.setColor(Color.black);
			bG2D.fillRect(pfdX - 35, pfdY + pfdWidth, 45, 31); // bottom
			bG2D.fillRect(pfdX + pfdWidth - 10, pfdY + pfdWidth, 55, 31);
			bG2D.drawLine(pfdX - 35, pfdY, pfdX - 1, pfdY);
			bG2D.setColor(Color.white);
			bG2D.drawLine(pfdX - 35, pfdY + 1, pfdX - 1, pfdY + 1); // pfdAirspeed
																	// top
			bG2D.drawLine(pfdX - 35, pfdY + pfdWidth, pfdX - 1, pfdY + pfdWidth); // pfdAirspeed
																					// bottom
			bG2D.setColor(Color.yellow);
			bG2D.drawLine(pfdX + pfdWidth, pfdY + 1, pfdX + pfdWidth + 40, pfdY + 1); // pfdaltitude
																						// top
			bG2D.drawLine(pfdX + pfdWidth, pfdY + pfdWidth, pfdX + pfdWidth + 40, pfdY + pfdWidth); // pfdaltitude
																									// bottom
			bG2D.setColor(Color.black);
			bG2D.drawLine(pfdX + pfdWidth, pfdY, pfdX + pfdWidth + 40, pfdY);

			if (timeDiff > adjustTime) {
				changeParameter = " ";
			}

			if (apOn) {
				bG2D.setColor(Color.white);
				bG2D.drawString("AP", pfdXC + 130, pfdY - 20);
			}

			// autopilot speed - continued
			if (speedOn) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(sButtX + 1, sButtY + 1, 62, 27);
				bG2D.drawRect(sButtX + 3, sButtY + 3, 58, 23);
				bG2D.drawString("SPEED", pfdX + 40, pfdY - 20);
				bG2D.setColor(Color.white);
				bG2D.drawString("A/THR", pfdXC + 160, pfdY - 20);
			}
			if (timeDiff < adjustTime && changeParameter.equals("Speed")) {
				bG2D.setColor(yellowy);
				bG2D.drawRect(sButtX + 1, sButtY + 31, 62, 19);
			}

			// ATT HOLD - continued ATTHButt
			if (attitudeOn) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(ATTHButtX + 1, ATTHButtY + 1, 62, 27);
				bG2D.drawRect(ATTHButtX + 3, ATTHButtY + 3, 58, 23);
			}
			if (timeDiff < adjustTime && changeParameter.equals("ATT HOLD")) {
				bG2D.setColor(yellowy);
				bG2D.drawRect(ATTHButtX + 1, ATTHButtY + 31, 62, 19);
			}

			// altitude - continued
			if (altitudeOn && (apOn || fdOn)) {
				bG2D.setColor(Color.green);
				bG2D.drawRect(ALTButtX + 1, ALTButtY + 1, 62, 27);
				bG2D.drawRect(ALTButtX + 3, ALTButtY + 3, 58, 23);
			}
			if (timeDiff < adjustTime && changeParameter.equals("ALT")) {
				bG2D.setColor(yellowy);
				bG2D.drawRect(ALTButtX + 1, ALTButtY + 31, 62, 19);
			}
			bG2D.setColor(Color.green);
			bG2D.drawString(verticalMode, pfdXC - 12, pfdY - 20);
			if (altArmed) {
				bG2D.setColor(Color.cyan);
				bG2D.drawString(MouseWheelAltitude, pfdX + pfdWidth + 2, pfdY + pfdWidth + 15);
			}

			// balance
			bG2D.setColor(Color.black);
			bG2D.drawString("Balance", balX + 25, balY - 3);
			bG2D.fillRect(balX, balY, 100, 100);
			bG2D.setColor(Color.white);
			if (sim2.balCor < 0)
				bG2D.drawLine(balX + 50, balY + 50, balX + 50 - (int) (40 * Math.cos(sim2.balance)),
						balY + 50 - (int) (40 * Math.sin(sim2.balance)));
			if (sim2.balCor >= 0)
				bG2D.drawLine(balX + 50, balY + 50, balX + 50 + (int) (40 * Math.cos(sim2.balance)),
						balY + 50 - (int) (40 * Math.sin(sim2.balance)));
			bG2D.drawLine(balX + 50, balY + 98, balX + 50, balY + 93);

		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			bufferGraphics.dispose();
		}
	}

	private void drawVelocityVector(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawLine(vVX - 4, vVY - 10, vVX - 4, vVY + 80);
		bG2D.drawString("Velocity", vVX, vVY);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.VelX, vVX + 10, vVY + 20);
		bG2D.drawString(sim2.VelY, vVX + 10, vVY + 50);
		bG2D.drawString(sim2.VelZ, vVX + 10, vVY + 80);
		bG2D.setColor(Color.black);
		bG2D.drawString("X", vVX, vVY + 20);
		bG2D.drawString("Y", vVX, vVY + 50);
		bG2D.drawString("Z", vVX, vVY + 80);
		// bG2D.drawString(autoPilot.ClimbRateReq, vVX+70,vVY+80);

	}

	private void drawThrustVector(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawLine(thVX - 4, thVY - 10, thVX - 4, thVY + 80);
		bG2D.drawString("Thrust", thVX, thVY);
		bG2D.drawString("X", thVX, thVY + 20);
		bG2D.drawString("Y", thVX, thVY + 50);
		bG2D.drawString("Z", thVX, thVY + 80);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.ThrustX, thVX + 10, thVY + 20);
		bG2D.drawString(sim2.ThrustY, thVX + 10, thVY + 50);
		bG2D.drawString(sim2.ThrustZ, thVX + 10, thVY + 80);

	}

	private void drawSideSlip(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawString("SideSlip: ", 500, 600);
		bG2D.drawString("AoA", 500, 620);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.SideSlipDegreesString, 550, 600);
		bG2D.drawString(sim2.AOADegrees, 550, 620);

	}

	private void drawAircraftMatrix(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawLine(attMX - 4, attMY - 10, attMX - 4, attMY + 80);
		bG2D.drawString("Attitude Matrix", attMX, attMY);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.A11, attMX, attMY + 20);
		bG2D.drawString(sim2.A12, attMX + 40, attMY + 20);
		bG2D.drawString(sim2.A13, attMX + 80, attMY + 20);
		bG2D.drawString(sim2.A21, attMX, attMY + 50);
		bG2D.drawString(sim2.A22, attMX + 40, attMY + 50);
		bG2D.drawString(sim2.A23, attMX + 80, attMY + 50);
		bG2D.drawString(sim2.A31, attMX, attMY + 80);
		bG2D.drawString(sim2.A32, attMX + 40, attMY + 80);
		bG2D.drawString(sim2.A33, attMX + 80, attMY + 80);

	}

	private void drawBodyRates(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawLine(bRX - 44, bRY - 10, bRX - 44, bRY + 80);
		bG2D.drawString("Body rates", bRX - 40, bRY);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.BodyRatePitch, bRX, bRY + 20);
		bG2D.drawString(sim2.BodyRateRoll, bRX, bRY + 50);
		bG2D.drawString(sim2.BodyRateYaw, bRX, bRY + 80);
		bG2D.setColor(Color.black);
		bG2D.drawString("Pitch", bRX - 40, bRY + 20);
		bG2D.drawString(" Roll", bRX - 40, bRY + 50);
		bG2D.drawString(" Yaw", bRX - 40, bRY + 80);

	}

	private void drawGearPanel(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.fillRect(gearX, gearY, 100, 50);
		bG2D.setColor(Color.lightGray);
		bG2D.drawString("Wheels", gearX + 44, gearY + 12);
		bG2D.setColor(Color.black);
		// bG2D.drawString("Up:" + gearUp, gearX+110,gearY+15);
		// bG2D.drawString("Tx:" + gearTx, gearX+110,gearY+30);
		// bG2D.drawString("Dn:" + gearDn, gearX+110,gearY+45);
		// gear up button
		bG2D.setColor(Color.lightGray);
		bG2D.fillRect(gearX + 5, gearY + 5, 30, 16);
		bG2D.setColor(Color.black);
		bG2D.drawString("UP", gearX + 10, gearY + 18);
		// gear down button
		bG2D.setColor(Color.lightGray);
		bG2D.fillRect(gearX + 5, gearY + 29, 30, 16);
		bG2D.setColor(Color.black);
		bG2D.drawString("DN", gearX + 10, gearY + 42);
		if (gearDn && !gearTx) {
			bG2D.setColor(Color.green);
			bG2D.drawString("DN", gearX + 60, gearY + 25); // center
			bG2D.drawString("DN", gearX + 49, gearY + 40); // left
			bG2D.drawString("DN", gearX + 71, gearY + 40); // right
		}
		if (gearTx && gearDiff < gearTime) {
			bG2D.setColor(Color.yellow);
			bG2D.fillOval(gearX + 62, gearY + 17, 9, 9); // center
			bG2D.fillOval(gearX + 52, gearY + 30, 9, 9); // left
			bG2D.fillOval(gearX + 72, gearY + 30, 9, 9); // right
		}
		if (gearUp && !gearTx) {
			bG2D.setColor(Color.gray);
			bG2D.drawString("UP", gearX + 60, gearY + 25); // center
			bG2D.drawString("UP", gearX + 49, gearY + 40); // left
			bG2D.drawString("UP", gearX + 71, gearY + 40); // right
		}
		if ((gearUp || gearTx) && sim2.positionZ > -1500.0 / 3.2808 && mouseWheelSpeed < 150) {
			bG2D.setColor(Color.red);
			bG2D.setFont(largeBold);
			bG2D.drawString("TOO LOW - GEAR", pfdX + 155, pfdY + 17);
			bG2D.setFont(stringFont);
		}

	}

	private void drawPosition(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawLine(posX - 4, posY - 10, posX - 4, posY + 80);
		bG2D.drawString("Position", posX, posY);
		bG2D.drawString("X", posX, posY + 20);
		bG2D.drawString("Y", posX, posY + 50);
		bG2D.drawString("Z", posX, posY + 80);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.PositionX, posX + 10, posY + 20);
		bG2D.drawString(sim2.PositionY, posX + 10, posY + 50);
		bG2D.drawString(sim2.PositionZ, posX + 10, posY + 80);

	}

	private void drawForces(Graphics2D bG2D) {
		bG2D.setColor(Color.black);
		bG2D.drawString("forceX =", 20, 480);
		bG2D.drawString("forceY  =", 20, 520);
		bG2D.drawString("forceZ  =", 20, 560);
		bG2D.setColor(Color.blue);
		bG2D.drawString(sim2.ForceX, 20, 495);
		bG2D.drawString(sim2.ForceY, 20, 535);
		bG2D.drawString(sim2.ForceZ, 20, 575);
		bG2D.drawString(sim2.ThrustX, 90, 495);
		bG2D.drawString(sim2.ThrustY, 90, 535);
		bG2D.drawString(sim2.ThrustZ, 90, 575);
		bG2D.drawString(sim2.LiftwtX, 150, 495);
		bG2D.drawString(sim2.LiftwtY, 150, 535);
		bG2D.drawString(sim2.LiftwtZ, 150, 575);
		bG2D.drawString(sim2.DragwtX, 210, 495);
		bG2D.drawString(sim2.DragwtY, 210, 535);
		bG2D.drawString(sim2.DragwtZ, 210, 575);
		bG2D.drawString(sim2.BodyDX, 280, 495);
		bG2D.drawString(sim2.BodyDY, 280, 535);
		bG2D.drawString(sim2.BodyDZ, 280, 575);
		bG2D.drawString(sim2.SideLiftForceX, 350, 495);
		bG2D.drawString(sim2.SideLiftForceY, 350, 535);
		bG2D.drawString(sim2.SideLiftForceZ, 350, 575);
		bG2D.drawString(sim2.GravityZ, 425, 575);
		bG2D.setColor(Color.black);
		bG2D.drawString("thrustX +", 90, 480);
		bG2D.drawString("thrustY +", 90, 520);
		bG2D.drawString("thrustZ +", 90, 560);
		bG2D.drawString("liftwtX +", 150, 480);
		bG2D.drawString("liftwtY +", 150, 520);
		bG2D.drawString("liftwtZ +", 150, 560);
		bG2D.drawString("dragwtX +", 210, 480);
		bG2D.drawString("dragwtY +", 210, 520);
		bG2D.drawString("dragwtZ +", 210, 560);
		bG2D.drawString("bodyDX +", 280, 480);
		bG2D.drawString("bodyDY +", 280, 520);
		bG2D.drawString("bodyDZ +", 280, 560);
		bG2D.drawString("SideLiftForceX +", 350, 480);
		bG2D.drawString("SideLiftForceY +", 350, 520);
		bG2D.drawString("SideLiftForceZ +", 350, 560);
		bG2D.drawString("g*m", 445, 560);

	}

	public void DrawBackbufferToScreen() {
		bufferStrategy.show();
		Toolkit.getDefaultToolkit().sync();
	}

	public void mouseWheelMoved(MouseWheelEvent e) {
		if (changeParameter.equals("Speed")) {
			if (mouseWheelSpeed < 401 && mouseWheelSpeed > 99) {
				mouseWheelSpeed = mouseWheelSpeed - e.getWheelRotation();
				if (mouseWheelSpeed > 400)
					mouseWheelSpeed = 400;
				if (mouseWheelSpeed < 100)
					mouseWheelSpeed = 100;
				MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
			}
			timeThen = System.currentTimeMillis();
		}
		if (changeParameter.equals("ATT HOLD")) {
			mouseWheelAttitude = mouseWheelAttitude + (0.5 * e.getWheelRotation());
			mouseWheelAttitude = ((int) (mouseWheelAttitude * 2)) / 2.0;
			MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
			timeThen = System.currentTimeMillis();
			if (MouseWheelAttitude.length() > 5)
				MouseWheelAttitude = MouseWheelAttitude.substring(0, 5);
		}
		if (changeParameter.equals("ALT")) {
			if (mouseWheelAltitude >= 500) {
				mouseWheelAltitude = mouseWheelAltitude - (500 * e.getWheelRotation());
				if (mouseWheelAltitude < 500)
					mouseWheelAltitude = 500;
			}
			MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
			timeThen = System.currentTimeMillis();
			if (MouseWheelAttitude.length() > 5)
				MouseWheelAttitude = MouseWheelAttitude.substring(0, 5);
		}
		if (changeParameter.equals("HDG")) {
			mouseWheelHDG = (int) mouseWheelHDG - (e.getWheelRotation());
			if (mouseWheelHDG < 0) {
				mouseWheelHDG = 360 + mouseWheelHDG;
			}
			if (mouseWheelHDG > 359) {
				mouseWheelHDG = 360 - mouseWheelHDG;
			}
			MouseWheelHDG = String.valueOf(mouseWheelHDG);
			if (MouseWheelHDG.length() > 3)
				MouseWheelHDG = MouseWheelHDG.substring(0, 3);
			if (mouseWheelHDG < 10) {
				MouseWheelHDG = MouseWheelHDG.substring(0, 2);
				MouseWheelHDG = "000".substring(MouseWheelHDG.length()) + MouseWheelHDG;
			}
			if (mouseWheelHDG < 100) {
				MouseWheelHDG = MouseWheelHDG.substring(0, 2);
				MouseWheelHDG = "000".substring(MouseWheelHDG.length()) + MouseWheelHDG;
			}
			timeThen = System.currentTimeMillis();
		}
	}

	@Override
	public void keyPressed(KeyEvent e) {
		if (!keysDown.contains(e.getKeyCode()))
			keysDown.add(new Integer(e.getKeyCode()));
		// System.out.println(e.getKeyCode());
		// this.requestFocus();
		moveControls();
	}

	@Override
	public void keyReleased(KeyEvent e) {
		keysDown.remove(new Integer(e.getKeyCode()));
	}

	@Override
	public void keyTyped(KeyEvent e) {
	}

	public void mousePressed(MouseEvent e) {

	}

	public void mouseReleased(MouseEvent e) {

	}

	public void mouseEntered(MouseEvent e) {

	}

	public void mouseExited(MouseEvent e) {

	}

	public void mouseClicked(MouseEvent e) {
		// Gear up
		if (e.getX() >= gearX + 5 && e.getX() <= gearX + 35 && e.getY() >= gearY + 5 && e.getY() <= gearY + 21) {
			if (!gearUp) {
				gearTx = true;
				gearUp = true;
				gearDn = false;
				gearThen = System.currentTimeMillis();
				// Random rand = new Random();
				// int randomNum = rand.nextInt((1000 - 0) + 1);
				// noseGTime = (long)randomNum;

				// rightTime

				// leftGTime
			}
		}
		// Gear down
		if (e.getX() >= gearX + 5 && e.getX() <= gearX + 35 && e.getY() >= gearY + 29 && e.getY() <= gearY + 45) {
			if (!gearDn) {
				gearTx = true;
				gearUp = false;
				gearDn = true;
				gearThen = System.currentTimeMillis();
			}
		}

		// reset button
		if (e.getX() >= resetX && e.getX() <= resetX + 55 && e.getY() >= resetY && e.getY() <= resetY + 30) {
			reset();
			return;
		}
		// reset to northern landing
		if (e.getX() >= resLNX && e.getX() <= resLNX + 55 && e.getY() >= resLNY && e.getY() <= resLNY + 30) {
			landingNorthReset();
			return;
		}
		// reset to 10nm final northern landing
		if (e.getX() >= res10X && e.getX() <= res10X + 55 && e.getY() >= res10Y && e.getY() <= res10Y + 30) {
			landingNorth10nmReset();
			return;
		}
		// reset to southern landing
		if (e.getX() >= resLSX && e.getX() <= resLSX + 55 && e.getY() >= resLSY && e.getY() <= resLSY + 30) {
			landingSouthReset();
			return;
		}
		// reset to Takeoff runway 36
		if (e.getX() >= resTOX && e.getX() <= resTOX + 55 && e.getY() >= resTOY && e.getY() <= resTOY + 30) {
			if (sim2.velX > 0 || sim2.velY > 0)
				sim2.resetCrash();
			// takeoffNorthReset();
			return;
		}
		// reset to glide
		if (e.getX() >= resGX && e.getX() <= resGX + 55 && e.getY() >= resGY && e.getY() <= resGY + 30) {
			glideReset();
			return;
		}
		// stop button
		if (e.getX() >= stopX && e.getX() <= stopX + 55 && e.getY() >= stopY && e.getY() <= stopY + 30) {
			sim2.stopHere();
			return;
		}
		// pfdOn button
		if (e.getX() >= pfdOnX && e.getX() <= pfdOnX + 55 && e.getY() >= pfdOnY && e.getY() <= pfdOnY + 30) {
			if (pfdOn) {
				pfdOn = false;
				return;
			} else {
				pfdOn = true;
				return;
			}
		}
		// Boy/Girl button
		if (e.getX() >= boyX && e.getX() <= boyX + 55 && e.getY() >= boyY && e.getY() <= boyY + 30) {
			if (!boy) {
				sky = new Color(255, 220, 255); // sky pink
				boy = true;
				return;
			} else {
				sky = new Color(202, 225, 255);
				boy = false;
				return;
			}
		}

		// Data button
		if (e.getX() >= dataX && e.getX() <= dataX + 55 && e.getY() >= dataY && e.getY() <= dataY + 30) {
			if (drawData) {
				drawData = false;
			} else {
				drawData = true;
			}
		}

		// NAV button on nd
		if (e.getX() >= nav1X && e.getX() <= nav1X + 30 && e.getY() >= nav1Y && e.getY() <= nav1Y + 30) {
			if (!nav1On) {
				nav1On = true;
			} else {
				nav1On = false;
			}
		}

		// direct to VOR 1 button
		if (e.getX() >= drt1X && e.getX() <= drt1X + 30 && e.getY() >= drt1Y && e.getY() <= drt1Y + 30) {
			setMouseWheelHDG();
		}

		// direct to VOR 1 button
		if (e.getX() >= wpSelX && e.getX() <= wpSelX + 30 && e.getY() >= wpSelY && e.getY() <= wpSelY + 30) {
			if (ToPoint.equals("RWY36")) {
				ToPoint = "RWY18";
				return;
			}
			ToPoint = "RWY36";
		}

		// scale R+ button on nd
		if (e.getX() >= rngPX && e.getX() <= rngPX + 30 && e.getY() >= rngPY && e.getY() <= rngPY + 30) {
			if (ndScale == 12.0) {
				ndScale = 30.0;
				NDScale = "30";
			}
			if (ndScale == 6.0) {
				ndScale = 12.0;
				NDScale = "12";
			}
		}
		// scale R- button on nd
		if (e.getX() >= rngMX && e.getX() <= rngMX + 30 && e.getY() >= rngMY && e.getY() <= rngMY + 30) {
			if (ndScale == 12.0) {
				ndScale = 6.0;
				NDScale = "6";
			}
			if (ndScale == 30.0) {
				ndScale = 12.0;
				NDScale = "12";
			}
		}

		// flight director on/off button
		if (e.getX() >= fdBX && e.getX() <= fdBX + 30 && e.getY() >= fdBY && e.getY() <= fdBY + 30) {
			if (fdOn) {
				fdOn = false;
				if (!apOn) {
					attitudeOn = false;
					altitudeOn = false;
					verticalMode = " ";
					hdgOn = false;
				}

			} else {
				fdOn = true;
				if (!altitudeOn && !glideSlope) {
					attitudeOn = true;
					verticalMode = "ATT HOLD";
				}
				if (!hdgOn && !llzActive) {
					hdgOn = true;
				}
			}
		}

		// autopilot on/off button
		if (e.getX() >= apBX && e.getX() <= apBX + 30 && e.getY() >= apBY && e.getY() <= apBY + 30) {
			if (apOn) {
				apOn = false;
				if (!fdOn) {
					attitudeOn = false;
					altitudeOn = false;
					verticalMode = " ";
					hdgOn = false;
				}
			} else {

				if (sim2.iasKnots >= 100) {
					apOn = true;
					if (!llzActive) {
						hdgOn = true;
					}
					fdOn = true;
					if (!altitudeOn && !glideSlope) {
						attitudeOn = true;
						verticalMode = "ATT HOLD";
					}
				}
			}
		}

		// velocity vector button
		if (e.getX() >= vvBX && e.getX() <= vvBX + 30 && e.getY() >= vvBY && e.getY() <= vvBY + 30) {
			if (vvOn) {
				vvOn = false;
				return;
			}
			vvOn = true;
		}

		// Spoilers button
		if (e.getX() >= spX && e.getX() <= spX + 65 && e.getY() >= spY && e.getY() <= spY + 30) {
			if (spoilersOn) {
				spoilersOn = false;
			} else {
				spoilersOn = true;
			}
		}

		// flame out button
		if (e.getX() >= fmOX && e.getX() <= fmOX + 65 && e.getY() >= fmOY && e.getY() <= fmOY + 30) {
			glideOn = true;
			apOn = true;
			hdgOn = true;
			fdOn = true;
			speedOn = false;
			altitudeOn = false;
			attitudeOn = true;
			verticalMode = "GLIDE 200";
			setMouseWheelHDG();
		}

		// autopilot speed button
		if (e.getX() >= sButtX && e.getX() <= sButtX + 65 && e.getY() >= sButtY && e.getY() <= sButtY + 30) {
			glideOn = false;
			autoPilot.resetCumError();
			if (speedOn) {
				speedOn = false;
			} else {
				speedOn = true;
			}
		}
		if (e.getX() >= sButtX && e.getX() <= sButtX + 65 && e.getY() >= sButtY + 30 && e.getY() <= sButtY + 52) {
			timeThen = System.currentTimeMillis();
			changeParameter = "Speed";
		}

		// attitude hold mode
		if (e.getX() >= attHX && e.getX() <= attHX + 65 && e.getY() >= attHY && e.getY() <= attHY + 52) {
			autoPilot.attHoldModeChange();
		}

		// APPR button glide path
		if (e.getX() >= ApprX && e.getX() <= ApprX + 65 && e.getY() >= ApprY && e.getY() <= ApprY + 52) {
			if (apprArmed) {
				apprArmed = false;
				return;
			}
			apprArmed = true;
			glideSlope = false;
			// attitudeOn = false;
			// altitudeOn = false;
			// verticalMode = "APPR*";
		}

		// ATT HOLD
		if (e.getX() >= ATTHButtX && e.getX() <= ATTHButtX + 65 && e.getY() >= ATTHButtY
				&& e.getY() <= ATTHButtY + 30) {
			glideOn = false;
			if (attitudeOn) {
				attitudeOn = false;
				verticalMode = " ";
				if (!hdgOn) {
					fdOn = false;
				}
				if (!hdgOn && !altitudeOn) {
					apOn = false;
				}
			} else {
				fdOn = true;
				attitudeOn = true;
				altitudeOn = false;
				altArmed = false;
				glideSlope = false;
				verticalMode = "ATT HOLD";

				// if (mouseWheelAltitude/-3.2808 > sim2.positionZ){
				// altArmed = true;
				// }
			}
		}
		if (e.getX() >= ATTHButtX && e.getX() <= ATTHButtX + 65 && e.getY() >= ATTHButtY + 30
				&& e.getY() <= ATTHButtY + 52) {
			timeThen = System.currentTimeMillis();
			changeParameter = "ATT HOLD";
		}

		// altitude button
		if (e.getX() >= ALTButtX && e.getX() <= ALTButtX + 65 && e.getY() >= ALTButtY && e.getY() <= ALTButtY + 30) {
			if (altitudeOn) {
				altitudeOn = false;
				verticalMode = " ";
				if (!hdgOn) {
					fdOn = false;
				}
				if (!hdgOn && !attitudeOn) {
					apOn = false;
				}
			} else if (!altArmed && attitudeOn) {
				altArmed = true;
			} else {
				fdOn = true;
				altitudeOn = true;
				altArmed = false;
				attitudeOn = false;
				verticalMode = "ALT";
				if (glideSlope) { // TOGA go around
					glideSlope = false;
					mouseWheelSpeed = 180;
					MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
				}
			}
		}
		if (e.getX() >= ALTButtX && e.getX() <= ALTButtX + 65 && e.getY() >= ALTButtY + 30
				&& e.getY() <= ALTButtY + 52) {
			timeThen = System.currentTimeMillis();
			changeParameter = "ALT";
		}

		// heading select button
		if (e.getX() >= hdgBX && e.getX() <= hdgBX + 65 && e.getY() >= hdgBY && e.getY() <= hdgBY + 30) {
			if (hdgOn) {
				hdgOn = false;
				if (!altitudeOn && !attitudeOn) {
					fdOn = false;
				}
			} else {
				if (llzActive) {
					mouseWheelHDG = (int) mouseWheelHeading;
				}

				hdgOn = true;
				llzArmed = false;
				llzActive = false;
				if (!attitudeOn && !altitudeOn && !glideSlope) {
					apOn = false;
				}
			}

			if (sim2.RollMode.equals("1"))
				sim2.rollModeSelect();
		}
		if (e.getX() >= hdgBX && e.getX() <= hdgBX + 65 && e.getY() >= hdgBY + 30 && e.getY() <= hdgBY + 52) {
			timeThen = System.currentTimeMillis();
			changeParameter = "HDG";
		}

		// LLZ button
		if (e.getX() >= LLZX && e.getX() <= LLZX + 65 && e.getY() >= LLZY && e.getY() <= LLZY + 30) {
			if (llzArmed) {
				llzArmed = false;
				return;
			}
			if (llzActive) {
				llzActive = false;
				return;
			}
			llzArmed = true;
		}

		// altitude mode button
		if (e.getX() >= altBX && e.getX() <= altBX + 65 && e.getY() >= altBY && e.getY() <= altBY + 52) {
			autoPilot.altHoldModeChange();
		}

		// roll mode button
		if (e.getX() >= rollMX && e.getX() <= rollMX + 65 && e.getY() >= rollMY && e.getY() <= rollMY + 52) {
			sim2.rollModeSelect();
		}
		//
		// rotate font button
		if (e.getX() >= fontX && e.getX() <= fontX + 55 && e.getY() >= fontY && e.getY() <= fontY + 30) {
			if (rotateFont) {
				rotateFont = false;
				return;
			}
			rotateFont = true;
			return;
		}

		// projection visuals button
		if (e.getX() >= visX && e.getX() <= visX + 55 && e.getY() >= visY && e.getY() <= visY + 30) {
			if (runwayOn) {
				runwayOn = false;
				return;
			}
			runwayOn = true;
			return;
		}
	}

	public void velocityVectorCalc() {
		beta = sim2.betaQ;
		gamma = sim2.gammaQ;
		velVecX = PDx + (int) (((beta + sim2.velocityVector) * pitchFactor) * Math.sin(gamma));
		velVecY = PDy + (int) (((beta + sim2.velocityVector) * pitchFactor) * Math.cos(gamma));

		vvLXin = velVecX - (8 * Math.cos(gamma));
		vvLYin = velVecY + (8 * Math.sin(gamma));
		vvLXou = velVecX - (13 * Math.cos(gamma));
		vvLYou = velVecY + (13 * Math.sin(gamma));

		vvRXin = velVecX + (8 * Math.cos(gamma));
		vvRYin = velVecY - (8 * Math.sin(gamma));
		vvRXou = velVecX + (13 * Math.cos(gamma));
		vvRYou = velVecY - (13 * Math.sin(gamma));

		vvTXin = velVecX - (8 * Math.sin(gamma));
		vvTYin = velVecY - (8 * Math.cos(gamma));
		vvTXou = velVecX - (13 * Math.sin(gamma));
		vvTYou = velVecY - (13 * Math.cos(gamma));
	}

	public void reset() {
		affineTransform.rotate(0, 0, 0);
		ailPos = 0;
		ailPosReq = 0;
		elePos = -1.4;
		elePosReq = -1.4;// up elevator
		rudPos = 0;
		rudPosReq = 0;
		apOn = true;
		hdgOn = true;
		fdOn = true;
		nav1On = true;
		gearDn = false;
		gearUp = true;
		gearTx = false;
		mouseWheelSpeed = 200;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAttitude = 1.5;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelAltitude = 10000;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
		mouseWheelHDG = 0;
		MouseWheelHDG = "000";
		speedOn = true;
		attitudeOn = true;
		altitudeOn = true;
		glideSlope = false;
		verticalMode = "ALT";
		throttlePosition = 30;
		sim2.reset();
	}

	public static void landingNorthReset() {
		toPointX = RWY36[0];
		toPointY = RWY36[1];
		ToPoint = "RWY36";
		ailPos = 0;
		ailPosReq = 0;
		elePos = -1.8;
		elePosReq = -1.8;// up elevator
		rudPos = 0;
		rudPosReq = 0;
		apOn = true;
		hdgOn = true;
		fdOn = false;
		nav1On = true;
		gearDn = true;
		gearUp = false;
		gearTx = false;
		mouseWheelSpeed = 140;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAttitude = 0.5;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelAltitude = 500;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
		mouseWheelHDG = 0;
		MouseWheelHDG = "000";
		speedOn = true;
		attitudeOn = true;
		altitudeOn = false;
		glideSlope = false;
		llzActive = false;
		llzArmed = false;
		apprArmed = false;
		verticalMode = "ATT HOLD"; // "ALT";
		throttlePosition = 30;
		sim2.resetNorthLanding();

	}

	public static void landingNorth10nmReset() {
		toPointX = RWY36[0];
		toPointY = RWY36[1];
		ToPoint = "RWY36";
		ailPos = 0;
		ailPosReq = 0;
		elePos = -1.8;
		elePosReq = -1.8;// up elevator
		rudPos = 0;
		rudPosReq = 0;
		apOn = true;
		hdgOn = true;
		fdOn = true;
		nav1On = true;
		llzArmed = true;
		llzActive = false;
		gearDn = false;
		gearUp = true;
		gearTx = false;
		mouseWheelSpeed = 180;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAttitude = 0.5;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelAltitude = 2000;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
		mouseWheelHDG = 330;
		MouseWheelHDG = "330";
		speedOn = true;
		attitudeOn = false;
		altitudeOn = true;
		glideSlope = false;
		apprArmed = true;
		verticalMode = "ALT"; // "ALT";
		throttlePosition = 30;
		sim2.resetNorth10nmLanding();
	}

	public static void takeoffNorthReset() {
		toPointX = RWY36[0];
		toPointY = RWY36[1];
		ToPoint = "RWY36";
		ailPos = 0;
		ailPosReq = 0;
		elePos = -1.8;
		elePosReq = -1.8;// up elevator
		rudPos = 0;
		rudPosReq = 0;
		apOn = false;
		hdgOn = false;
		fdOn = false;
		nav1On = true;
		gearDn = true;
		gearUp = false;
		gearTx = false;
		mouseWheelSpeed = 180;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAttitude = 0.5;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelAltitude = 2000;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
		mouseWheelHDG = 0;
		MouseWheelHDG = "000";
		speedOn = false;
		attitudeOn = false;
		altitudeOn = false;
		glideSlope = false;
		verticalMode = "ATT HOLD"; // "ALT";
		throttlePosition = 0;
		Thread.currentThread();

		sim2.resetNorthTakeoff();
	}

	public void landingSouthReset() {
		toPointX = RWY18[0];
		toPointY = RWY18[1];
		ToPoint = "RWY18";
		ailPos = 0;
		ailPosReq = 0;
		elePos = -1.8;
		elePosReq = -1.8;// up elevator
		rudPos = 0;
		rudPosReq = 0;
		apOn = true;
		hdgOn = true;
		fdOn = false;
		nav1On = true;
		gearDn = true;
		gearUp = false;
		gearTx = false;
		mouseWheelSpeed = 140;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAttitude = 0.5;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelAltitude = 2000;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
		mouseWheelHDG = 180;
		MouseWheelHDG = "180";
		speedOn = true;
		attitudeOn = true;
		altitudeOn = false;
		glideSlope = false;
		llzActive = false;
		llzArmed = false;
		apprArmed = false;
		verticalMode = "ATT HOLD"; // "ALT";
		throttlePosition = 30;
		sim2.resetSouthLanding();
	}

	public void glideReset() {
		toPointX = RWY36[0];
		toPointY = RWY36[1];
		ToPoint = "RWY36";
		ailPos = 0;
		ailPosReq = 0;
		elePos = -1.4;
		elePosReq = -1.4;// up elevator
		rudPos = 0;
		rudPosReq = 0;
		apOn = true;
		hdgOn = true;
		fdOn = true;
		nav1On = true;
		gearDn = false;
		gearUp = true;
		gearTx = false;
		mouseWheelSpeed = 200;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAttitude = -10.0;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelAltitude = 2000;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
		setMouseWheelHDG();
		ndScale = 30.0;
		NDScale = "30";
		speedOn = false;
		attitudeOn = true;
		altitudeOn = false;
		glideSlope = false;
		verticalMode = "GLIDE 200"; // "ALT";
		throttlePosition = 0;
		glideOn = true;
		sim2.resetGlide();
	}

	public void moveControls() {
		if (keysDown.contains(new Integer(KeyEvent.VK_LEFT)) && ailPosReq > -50) // 37
		{
			ailPosReq -= 0.5;
			apOn = false;
		}
		if (keysDown.contains(new Integer(KeyEvent.VK_RIGHT)) && ailPosReq < 50) // 39
		{
			ailPosReq += 0.5;
			apOn = false;
		}
		if (keysDown.contains(new Integer(KeyEvent.VK_UP)) && elePosReq < 50) // 39
		{// attitudeOn = false; altitudeOn = false; verticalMode = " ";
			elePosReq += 0.05;
			apOn = false;
		}
		if (keysDown.contains(new Integer(KeyEvent.VK_DOWN)) && elePosReq < 50) // 39
		{// attitudeOn = false; altitudeOn = false; verticalMode = " ";
			elePosReq -= 0.05;
			apOn = false;
		}
		if (keysDown.contains(97) && rudPosReq > -50) {
			rudPosReq -= 0.5;
			apOn = false;
		}
		if (keysDown.contains(99) && rudPosReq < 50) {
			rudPosReq += 0.5;
			apOn = false;
		}
		if (keysDown.contains(100) && ailPosReq > -50) // 4
		{
			ailPosReq -= 2;
			apOn = false;
		}
		if (keysDown.contains(101)) // 5
		{
			ailPosReq = 0;
			rudPosReq = 0;
			apOn = false;
		}
		if (keysDown.contains(102) && ailPosReq < 50) // 6
		{
			ailPosReq += 2;
			apOn = false;
		}
		if (keysDown.contains(98) && elePosReq > -50) // 2
		{// attitudeOn = false; altitudeOn = false;
			// verticalMode = " ";
			elePosReq -= 0.25;
			apOn = false;
		}
		if (keysDown.contains(104) && elePosReq < 50) // 8
		{// attitudeOn = false; altitudeOn = false; verticalMode = " ";
			elePosReq += 0.25;
			apOn = false;
		}
		if (keysDown.contains(107)) {// attitudeOn = false; altitudeOn = false;
										// verticalMode = " ";
			elePosReq = 0;
		} // +
		if (keysDown.contains(33) && throttlePosition < 100) // Page Up
		{
			speedOn = false;
			throttlePosition += 2;
		}
		if (keysDown.contains(34) && throttlePosition > 0) // Page Down
		{
			speedOn = false;
			throttlePosition -= 2;
		}
		if (keysDown.contains(155)) // Insert Key
		{
			speedOn = false;
			throttlePosition = 100;
		}
		if (keysDown.contains(127)) // Delete Key
		{
			speedOn = false;
			throttlePosition = 0;
		}

		if (keysDown.contains(96)) // the 0 on the number keyboard
		{
			System.out.print('\f');
		} // this will clear the text on System.out.print...
	}

	public void controlPositions() {
		if (elePos < elePosReq - 0.01)
			elePos = elePos + 0.05;
		if (elePos > elePosReq + 0.01)
			elePos = elePos - 0.05;
		if (elePos < -1.59 && elePos > -1.61) {
			elePos = -1.6;
		}
		if (elePos < 0.01 && elePos > -0.01) {
			elePos = 0.0;
		}

		// if (ailPos < ailPosReq) ailPos = ailPos + 0.05;
		// if (ailPos > ailPosReq) ailPos = ailPos - 0.05;
		ailPos = ailPosReq;

		if (rudPos < rudPosReq)
			rudPos = rudPos + 0.5;
		if (rudPos > rudPosReq)
			rudPos = rudPos - 0.5;
		if (actThrust < throttlePosition)
			actThrust = actThrust + 0.5;
		if (actThrust > throttlePosition)
			actThrust = actThrust - 1.0;
	}

	/**
	 * Calculates where the black below horizon attitudes will appear on the
	 * PFD. 10 degree marks
	 */
	public void attitudesBelow(int i) {
		hXp = PDx + (int) (((Beta * pitchFactor) + (((Math.PI / 18) * i) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) + (((Math.PI / 18) * i) * pitchFactor)) * Math.cos(Gamma));
		P1xa = hXp - 15 * Math.cos(Gamma);
		P1ya = hYp + 15 * Math.sin(Gamma);
		P2xa = hXp + 15 * Math.cos(Gamma);
		P2ya = hYp - 15 * Math.sin(Gamma);
		inX1a = insideX(P1xa);
		inY1a = insideY(P1ya);
		inX2a = insideX(P2xa);
		inY2a = insideY(P2ya);

		if (Gamma > (-1 * Math.PI / 180) && Gamma < Math.PI / 180)
			P2ya = P1ya; // This makes the attitude line appear flat.

		inX2b = insideXm((int) P2xa);
		inY2b = insideYm((int) P2ya);

		hXp = PDx + (int) (((Beta * pitchFactor) + ((Math.PI / 2) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) + ((Math.PI / 2) * pitchFactor)) * Math.cos(Gamma));
		inX1c = insideXm(hXp);
		inY1c = insideYm(hYp);
	}

	/**
	 * Calculates where the black below horizon attitudes will appear on the
	 * PFD. 5 degree marks
	 */
	public void attitudesBelow5(int i) {
		hXp = PDx + (int) (((Beta * pitchFactor) + (((Math.PI / 18) * (i - 0.5)) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) + (((Math.PI / 18) * (i - 0.5)) * pitchFactor)) * Math.cos(Gamma));
		P1xa = hXp - 8 * Math.cos(Gamma);
		P1ya = hYp + 8 * Math.sin(Gamma);
		P2xa = hXp + 8 * Math.cos(Gamma);
		P2ya = hYp - 8 * Math.sin(Gamma);
		inX1a = insideX(P1xa);
		inY1a = insideY(P1ya);
		inX2a = insideX(P2xa);
		inY2a = insideY(P2ya);

		if (Gamma > (-1 * Math.PI / 180) && Gamma < Math.PI / 180)
			P2ya = P1ya; // This makes the attitude line appear flat.

		inX2b = insideXm((int) P2xa);
		inY2b = insideYm((int) P2ya);

		hXp = PDx + (int) (((Beta * pitchFactor) + ((Math.PI / 2) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) + ((Math.PI / 2) * pitchFactor)) * Math.cos(Gamma));
		inX1c = insideXm(hXp);
		inY1c = insideYm(hYp);
	}

	public void attitudesAbove(int i) {
		hXp = PDx + (int) (((Beta * pitchFactor) - (((Math.PI / 18) * i) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) - (((Math.PI / 18) * i) * pitchFactor)) * Math.cos(Gamma));
		P1xa = hXp - 15 * Math.cos(Gamma);
		P1ya = hYp + 15 * Math.sin(Gamma);
		P2xa = hXp + 15 * Math.cos(Gamma);
		P2ya = hYp - 15 * Math.sin(Gamma);
		inX1a = insideX(P1xa);
		inY1a = insideY(P1ya);
		inX2a = insideX(P2xa);
		inY2a = insideY(P2ya);

		if (Gamma > (-1 * Math.PI / 180) && Gamma < Math.PI / 180)
			P2ya = P1ya; // This makes the attitude line appear flat.

		inX2b = insideXm((int) P2xa);
		inY2b = insideYm((int) P2ya);

		hXp = PDx + (int) (((Beta * pitchFactor) - ((Math.PI / 2) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) - ((Math.PI / 2) * pitchFactor)) * Math.cos(Gamma));
		inX1c = insideXm(hXp);
		inY1c = insideYm(hYp);
	}

	public void attitudesAbove5(int i) {
		hXp = PDx + (int) (((Beta * pitchFactor) - (((Math.PI / 18) * (i - 0.5)) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) - (((Math.PI / 18) * (i - 0.5)) * pitchFactor)) * Math.cos(Gamma));
		P1xa = hXp - 8 * Math.cos(Gamma);
		P1ya = hYp + 8 * Math.sin(Gamma);
		P2xa = hXp + 8 * Math.cos(Gamma);
		P2ya = hYp - 8 * Math.sin(Gamma);
		inX1a = insideX(P1xa);
		inY1a = insideY(P1ya);
		inX2a = insideX(P2xa);
		inY2a = insideY(P2ya);

		if (Gamma > (-1 * Math.PI / 180) && Gamma < Math.PI / 180)
			P2ya = P1ya; // This makes the attitude line appear flat.

		inX2b = insideXm((int) P2xa);
		inY2b = insideYm((int) P2ya);

		hXp = PDx + (int) (((Beta * pitchFactor) - ((Math.PI / 2) * pitchFactor)) * Math.sin(Gamma));
		hYp = PDy + (int) (((Beta * pitchFactor) - ((Math.PI / 2) * pitchFactor)) * Math.cos(Gamma));
		inX1c = insideXm(hXp);
		inY1c = insideYm(hYp);
	}

	/**
	 * These four 'inside' methods decide if the attitude lines and digits will
	 * be drawn on the PFD.
	 */
	public boolean insideX(double xPoint) {
		if (xPoint >= pfdX && xPoint <= pfdX + 300)
			return true;
		return false;
	}

	public boolean insideY(double yPoint) {
		if (yPoint >= pfdY && yPoint <= pfdY + 300)
			return true;
		return false;
	}

	public void actionPerformed(ActionEvent act) {
		repaint();
	}

	public void hdgIndicatorAndND() {
		hdg = sim2.alphaQ;
		// hdg = Alpha;
		heading = (int) (hdg * 180 / Math.PI);
		if (heading < 0)
			heading = 360 + heading;
		Heading = Integer.toString(heading);
		Heading = "000".substring(Heading.length()) + Heading;

		exactHeading = hdg * 180 / Math.PI; // a double. Could be negative.
		if (exactHeading < 0)
			exactHeading = 360 + exactHeading;
		H1int = ((int) exactHeading / 10) * 10;
		hdgDiff = exactHeading - H1int;
		pixelOffset = (int) (hdgDiff * 6.0);
		H1 = Integer.toString(H1int);
		H1 = "000".substring(H1.length()) + H1;
		H2int = H1int - 10;
		if (H2int < 0)
			H2int = 360 + H2int;
		H2 = Integer.toString(H2int);
		H2 = "000".substring(H2.length()) + H2;
		H3int = H1int - 20;
		if (H3int < 0)
			H3int = 360 + H3int;
		H3 = Integer.toString(H3int);
		H3 = "000".substring(H3.length()) + H3;
		H4int = H1int + 10;
		if (H2int >= 360)
			H4int = H4int - 360;
		H4 = Integer.toString(H4int);
		H4 = "000".substring(H4.length()) + H4;
		H5int = H1int + 20;
		if (H5int >= 360)
			H5int = H5int - 360;
		H5 = Integer.toString(H5int);
		H5 = "000".substring(H5.length()) + H5;
		H6int = H1int + 30;
		if (H6int >= 360)
			H6int = H6int - 360;
		H6 = Integer.toString(H6int);
		H6 = "000".substring(H6.length()) + H6;

	}

	public void PFD() { // attitude, angle of bank
		// pitch = beta, roll = gamma
		// Center of Horizon reference the Center of the PFD
		beta = sim2.betaQ;
		gamma = sim2.gammaQ;

		hXp00 = PDx + (int) ((beta * pitchFactor) * Math.sin(gamma));
		hYp00 = PDy + (int) ((beta * pitchFactor) * Math.cos(gamma));
		// Horizon Line
		hX1 = hXp00 - (int) (212.14 * Math.cos(gamma));
		hY1 = hYp00 + (int) (212.14 * Math.sin(gamma));
		hX2 = hXp00 + (int) (212.14 * Math.cos(gamma));
		hY2 = hYp00 - (int) (212.14 * Math.sin(gamma));
		Beta = beta;
		Gamma = gamma;
		AoB = gamma * conversion;
		attitude = beta * conversion;

		PFDdefinePoints();
		polygon();
	}

	public boolean insideXm(int xPoint) {
		if (xPoint >= pfdX + 20 && xPoint <= pfdX + 280)
			return true;
		return false;
	}

	public boolean insideYm(int yPoint) {
		if (yPoint >= pfdY + 20 && yPoint <= pfdY + 280)
			return true;
		return false;
	}

	public boolean insideXm4(int xPoint) { // used for the green dot on the
											// horizon line
		if (xPoint >= pfdX + 4 && xPoint <= pfdX + 296)
			return true;
		return false;
	}

	public boolean insideYm4(int yPoint) { // used for the green dot on the
											// horizon line
		if (yPoint >= pfdY + 4 && yPoint <= pfdY + 296)
			return true;
		return false;
	}

	/**
	 * Calculate where the horizon line intercepts the side of the PFD. Defines
	 * it as P1x, P1y.
	 */
	public void intersectP1() {
		P1x = (((x1 * y2 - y1 * x2) * (x3 - x4)) - ((x1 - x2) * (x3 * y4 - y3 * x4)))
				/ (((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)));
		P1y = (((x1 * y2 - y1 * x2) * (y3 - y4)) - ((y1 - y2) * (x3 * y4 - y3 * x4)))
				/ (((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)));
	}

	/**
	 * Calculate where the horizon line intercepts the other side of the PFD.
	 * Defines it as P2x, P2y.
	 */
	public void intersectP2() {
		P2x = (((x1 * y2 - y1 * x2) * (x3 - x4)) - ((x1 - x2) * (x3 * y4 - y3 * x4)))
				/ (((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)));
		P2y = (((x1 * y2 - y1 * x2) * (y3 - y4)) - ((y1 - y2) * (x3 * y4 - y3 * x4)))
				/ (((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)));
	}

	public void intersectPoints(int px1, int py1, int px2, int py2, int px3, int py3, int px4, int py4) {
		newX = (((px1 * py2 - py1 * px2) * (px3 - px4)) - ((px1 - px2) * (px3 * py4 - py3 * px4)))
				/ (((px1 - px2) * (py3 - py4)) - ((py1 - py2) * (px3 - px4)));
		newY = (((px1 * py2 - py1 * px2) * (py3 - py4)) - ((py1 - py2) * (px3 * py4 - py3 * px4)))
				/ (((px1 - px2) * (py3 - py4)) - ((py1 - py2) * (px3 - px4)));
	}

	public boolean insidePFD(int xPoint, int yPoint) {
		if (xPoint >= pfdX && xPoint <= pfdX + pfdWidth) {
			if (yPoint >= pfdY && yPoint <= pfdY + pfdWidth)
				return true;
		}
		return false;
	}

	public void PFDdefinePoints() {
		if (hX1 == pfdX && hY1 == pfdY && hX2 == (pfdX + pfdWidth) && hY2 == (pfdY + pfdWidth)) {
			P1x = pfdX;
			P1y = pfdY;
			P2x = pfdX + 300;
			P2y = pfdY + 300;
			// UI.println("PFDdefinePoints 1");
			return; // 45 left turn
		}
		if (hX1 == pfdX && hY1 == pfdY + 300 && hX2 == (pfdX + pfdWidth) && hY2 == (pfdY + pfdWidth)) {
			P1x = pfdX;
			P1y = pfdY + 300;
			P2x = pfdX + 300;
			P2y = pfdY;
			// UI.println("PFDdefinePoints 2");
			return; // 45 right turn
		}
		if (hX1 == pfdX + 300 && hY1 == pfdY + 300 && hX2 == pfdX && hY2 == pfdY) {
			P1x = pfdX + 300;
			P1y = pfdY + 300;
			P2x = pfdX;
			P2y = pfdY;
			// UI.println("PFDdefinePoints 3");
			return; // 135 right turn
		}
		if (hX1 == pfdX + 300 && hY1 == pfdY && hX2 == pfdX && hY2 == pfdY + pfdWidth) {
			P1x = pfdX + 300;
			P1y = pfdY;
			P2x = pfdX;
			P2y = pfdY + 300;
			// UI.println("PFDdefinePoints 4");
			return; // 135 left turn
		}

		if (hX1 < pfdX) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX;
			y2 = (pfdY + 300);
			x3 = hX1;
			y3 = hY1;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}
		if (hX1 > (pfdX + 300)) {
			x1 = pfdX + 300;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = hX1;
			y3 = hY1;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}
		if (hY1 < pfdY) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = pfdY;
			x3 = hX1;
			y3 = hY1;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}
		if (hY1 > pfdY + 300) {
			x1 = pfdX;
			y1 = pfdY + 300;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = hX1;
			y3 = hY1;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}

		if (P1x < pfdX) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}
		if (P1x > (pfdX + 300)) {
			x1 = pfdX + 300;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}
		if (P1y < pfdY) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = pfdY;
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}
		if (P1y > pfdY + 300) {
			x1 = pfdX;
			y1 = pfdY + 300;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP1();
		}

		if (hX2 < pfdX) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP2();
		}
		if (hX2 > (pfdX + 300)) {
			x1 = pfdX + 300;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP2();
		}
		if (hY2 < pfdY) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = pfdY;
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP2();
		}
		if (hY2 > pfdY + 300) {
			x1 = pfdX;
			y1 = pfdY + 300;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = hX2;
			y4 = hY2;
			intersectP2();
		}

		if (P2x < pfdX) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = P2x;
			y4 = P2y;
			intersectP2();
		}
		if (P2x > (pfdX + 300)) {
			x1 = pfdX + 300;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = P2x;
			y4 = P2y;
			intersectP2();
		}
		if (P2y < pfdY) {
			x1 = pfdX;
			y1 = pfdY;
			x2 = pfdX + 300;
			y2 = pfdY;
			x3 = P1x;
			y3 = P1y;
			x4 = P2x;
			y4 = P2y;
			intersectP2();
		}
		if (P2y > pfdY + 300) {
			x1 = pfdX;
			y1 = pfdY + 300;
			x2 = pfdX + 300;
			y2 = (pfdY + 300);
			x3 = P1x;
			y3 = P1y;
			x4 = P2x;
			y4 = P2y;
			intersectP2();
		}
	}

	/**
	 * Draws the 3, 4 or 5 cornered polygon that indicates ground on the PFD.
	 * PDF = Primary Flight Display I had a few problems with this method.
	 */
	public void polygon() {
		if (P1x == pfdX && P1y == pfdY && P2x == (pfdX + pfdWidth) && P2y == (pfdY + pfdWidth)) {
			PDXg[0] = pfdX;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX + 300;

			PDYg[0] = pfdY;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY + 300;
			npoints = 3;
			// 45 degree left turn
			PDXs[0] = pfdX;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX + 300;
			PDYs[0] = pfdY;
			PDYs[1] = pfdY;
			PDYs[2] = pfdY + 300;
			spoints = 3;
			return;
		}

		if (P1x == pfdX && P1y == pfdY + 300 && P2x == pfdX + pfdWidth && P2y == pfdY) {
			PDXg[0] = pfdX;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX + 300;

			PDYg[0] = pfdY + 300;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY;
			npoints = 3;
			// 45 degree right turn
			PDXs[0] = pfdX;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX + 300;
			PDYs[0] = pfdY + 300;
			PDYs[1] = pfdY;
			PDYs[2] = pfdY;
			spoints = 3;
			// UI.println("polygon() 45 degree right turn");
			return;
		}

		if (P1x == pfdX + 300 && P1y == pfdY + 300 && P2x == pfdX && P2y == pfdY) {
			PDXg[0] = pfdX + 300;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX;

			PDYg[0] = pfdY + 300;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY;
			npoints = 3;
			// 135 degree right turn
			PDXs[0] = pfdX + 300;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX;
			PDYs[0] = pfdY + 300;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY;
			spoints = 3;
			return;
		}

		if (P1x == pfdX + 300 && P1y == pfdY && P2x == pfdX && P2y == pfdY + pfdWidth) {
			PDXg[0] = pfdX + 300;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX;

			PDYg[0] = pfdY;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY + 300;
			npoints = 3;
			// 135 degree left turn
			PDXs[0] = pfdX + 300;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX;
			PDYs[0] = pfdY;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY + 300;
			spoints = 3;
			return;
		}

		if (P1x > pfdX - 0.001 && P1x < pfdX + 0.001 && P2x > pfdX + 299.999 && P1x < pfdX + 300.001) {
			PDXg[0] = pfdX;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX + 300;
			PDXg[3] = pfdX + 300;
			PDYg[0] = (int) P1y;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY + 300;
			PDYg[3] = (int) P2y;
			npoints = 4;
			// wings approx level, nose near horizon
			PDXs[0] = pfdX;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX + 300;
			PDXs[3] = pfdX + 300;
			PDYs[0] = pfdY;
			PDYs[1] = (int) P1y;
			PDYs[2] = (int) P2y;
			PDYs[3] = pfdY;
			spoints = 4;
			return;
		}

		if (P1x > pfdX - 0.001 && P1x < pfdX + 0.001 && P2y > pfdY - 0.001 && P2y < pfdY + 0.001) {
			PDXg[0] = pfdX;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX + 300;
			PDXg[3] = pfdX + 300;
			PDXg[4] = (int) P2x;
			PDYg[0] = (int) P1y;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY + 300;
			PDYg[3] = pfdY;
			PDYg[4] = pfdY;
			npoints = 5;
			// right turn, nose low
			PDXs[0] = pfdX;
			PDXs[1] = pfdX;
			PDXs[2] = (int) P2x;
			PDYs[0] = pfdY;
			PDYs[1] = (int) P1y;
			PDYs[2] = pfdY;
			spoints = 3;
			return;
		}

		if (P1y > pfdY - 0.001 && P1y < pfdY + 0.001 && P2x > pfdX + 299.999 && P2x < pfdX + 300.001) {
			PDXg[0] = (int) P1x;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX;
			PDXg[3] = pfdX + 300;
			PDXg[4] = pfdX + 300;
			PDYg[0] = pfdY;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY + 300;
			PDYg[3] = pfdY + 300;
			PDYg[4] = (int) P2y;
			npoints = 5;
			// left turn, nose down
			PDXs[0] = (int) P1x;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX + 300;
			PDYs[0] = pfdY;
			PDYs[1] = (int) P2y;
			PDYs[2] = pfdY;
			spoints = 3;
			return;
		}

		if (P1y > pfdY + 299.999 && P1y < pfdY + 300.001 && P2x > pfdX + 299.999 && P2x < pfdX + 300.001) {
			PDXg[0] = (int) P1x;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX + 300;
			PDYg[0] = pfdY + 300;
			PDYg[1] = pfdY + 300;
			PDYg[2] = (int) P2y;
			npoints = 3;
			// nose high, right turn
			PDXs[0] = (int) P1x;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX;
			PDXs[3] = pfdX + 300;
			PDXs[4] = pfdX + 300;
			PDYs[0] = pfdY + 300;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY;
			PDYs[3] = pfdY;
			PDYs[4] = (int) P2y;
			spoints = 5;
			return;
		}

		if (P1x > pfdX - 0.001 && P1x < pfdX + 0.001 && P2y > pfdY + 299.999 && P2y < pfdY + 300.001) {
			PDXg[0] = pfdX;
			PDXg[1] = pfdX;
			PDXg[2] = (int) P2x;
			PDYg[0] = (int) P1y;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY + 300;
			npoints = 3;
			// nose high, left turn
			PDXs[0] = pfdX;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX + 300;
			PDXs[3] = pfdX + 300;
			PDXs[4] = (int) P2x;
			PDYs[0] = (int) P1y;
			PDYs[1] = pfdY;
			PDYs[2] = pfdY;
			PDYs[3] = pfdY + 300;
			PDYs[4] = pfdY + 300;
			spoints = 5;
			return;
		}

		if (P1y > pfdY + 299.999 && P1y < pfdY + 300.001 && P2y > pfdY - 0.001 && P2y < pfdY + 0.001) {
			PDXg[0] = (int) P1x;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX + 300;
			PDXg[3] = (int) P2x;
			PDYg[0] = pfdY + 300;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY;
			PDYg[3] = pfdY;
			npoints = 4;
			// 90 degree right hand turn, nose on horizon
			PDXs[0] = (int) P1x;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX;
			PDXs[3] = (int) P2x;
			PDYs[0] = pfdY + 300;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY;
			PDYs[3] = pfdY;
			spoints = 4;
			return;
		}

		if (P1x > pfdX + 299.999 && P1x < pfdX + 300.001 && P2y > pfdY - 0.001 && P2y < pfdY + 0.001) {
			PDXg[0] = pfdX + 300;
			PDXg[1] = pfdX + 300;
			PDXg[2] = (int) P2x;
			PDYg[0] = (int) P1y;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY;
			npoints = 3;
			// nose high, right turn, inverted
			PDXs[0] = pfdX + 300;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX;
			PDXs[3] = pfdX;
			PDXs[4] = (int) P2x;
			PDYs[0] = (int) P1y;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY + 300;
			PDYs[3] = pfdY;
			PDYs[4] = pfdY;
			spoints = 5;
			return;
		}

		if (P1y > pfdY + 299.999 && P1y < pfdY + 300.001 && P2x > pfdX - 0.001 && P2x < pfdX + 0.001) {
			PDXg[0] = (int) P1x;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX + 300;
			PDXg[3] = pfdX;
			PDXg[4] = pfdX;

			PDYg[0] = pfdY + 300;
			PDYg[1] = pfdY + 300;
			PDYg[2] = pfdY;
			PDYg[3] = pfdY;
			PDYg[4] = (int) P2y;
			npoints = 5;
			// left turn, nose low, almost inverted
			PDXs[0] = (int) P1x;
			PDXs[1] = pfdX;
			PDXs[2] = pfdX;
			PDYs[0] = pfdY + 300;
			PDYs[1] = (int) P2y;
			PDYs[2] = pfdY + 300;
			spoints = 3;
			return;
		}

		if (P1x > pfdX + 299.999 && P1x < pfdX + 300.001 && P2x > pfdX - 0.001 && P2x < pfdX + 0.001) {
			PDXg[0] = pfdX + 300;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX;
			PDXg[3] = pfdX;

			PDYg[0] = (int) P1y;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY;
			PDYg[3] = (int) P2y;
			npoints = 4;
			// inverted
			PDXs[0] = pfdX + 300;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX;
			PDXs[3] = pfdX;
			PDYs[0] = (int) P1y;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY + 300;
			PDYs[3] = (int) P2y;
			spoints = 4;
			return;
		}

		if (P1y > pfdY - 0.001 && P1y < pfdY + 0.001 && P2x > pfdX - 0.001 && P2x < pfdX + 0.001) {
			PDXg[0] = (int) P1x;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX;
			PDYg[0] = pfdY;
			PDYg[1] = pfdY;
			PDYg[2] = (int) P2y;
			npoints = 3;
			// nose high, left turn, inverted
			PDXs[0] = (int) P1x;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX + 300;
			PDXs[3] = pfdX;
			PDXs[4] = pfdX;
			PDYs[0] = pfdY;
			PDYs[1] = pfdY;
			PDYs[2] = pfdY + 300;
			PDYs[3] = pfdY + 300;
			PDYs[4] = (int) P2y;
			spoints = 5;
			return;
		}

		if (P1x > pfdX + 299.999 && P1x < pfdX + 300.001 && P2y > pfdY + 299.999 && P2y < pfdY + 300.001) {
			PDXg[0] = pfdX + 300;
			PDXg[1] = pfdX + 300;
			PDXg[2] = pfdX;
			PDXg[3] = pfdX;
			PDXg[4] = (int) P2x;

			PDYg[0] = (int) P1y;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY;
			PDYg[3] = pfdY + 300;
			PDYg[4] = pfdY + 300;
			npoints = 5;
			// nose low, almost inverted, roll left
			PDXs[0] = pfdX + 300;
			PDXs[1] = (int) P2x;
			PDXs[2] = pfdX + 300;
			PDYs[0] = (int) P1y;
			PDYs[1] = pfdY + 300;
			PDYs[2] = pfdY + 300;
			spoints = 3;
			return;
		}

		if (P1y > pfdY - 0.001 && P1y < pfdY + 0.001 && P2y > pfdY + 299.999 && P2y < pfdY + 300.001) {
			PDXg[0] = (int) P1x;
			PDXg[1] = pfdX;
			PDXg[2] = pfdX;
			PDXg[3] = (int) P2x;

			PDYg[0] = pfdY;
			PDYg[1] = pfdY;
			PDYg[2] = pfdY + 300;
			PDYg[3] = pfdY + 300;
			npoints = 4;
			// 90 degree left turn
			PDXs[0] = (int) P1x;
			PDXs[1] = pfdX + 300;
			PDXs[2] = pfdX + 300;
			PDXs[3] = (int) P2x;
			PDYs[0] = pfdY;
			PDYs[1] = pfdY;
			PDYs[2] = pfdY + 300;
			PDYs[3] = pfdY + 300;
			spoints = 4;
			return;
		}
	}

	public void fillGround() {
		PDXg[0] = pfdX;
		PDXg[1] = pfdX;
		PDXg[2] = pfdX + 300;
		PDXg[3] = pfdX + 300;

		PDYg[0] = pfdY;
		PDYg[1] = pfdY + 300;
		PDYg[2] = pfdY + 300;
		PDYg[3] = pfdY;
		npoints = 4;
	}

	public void fillSky() {
		PDXs[0] = pfdX;
		PDXs[1] = pfdX;
		PDXs[2] = pfdX + 300;
		PDXs[3] = pfdX + 300;

		PDYs[0] = pfdY;
		PDYs[1] = pfdY + 300;
		PDYs[2] = pfdY + 300;
		PDYs[3] = pfdY;
		spoints = 4;
	}

	public static void setTargetThrust(double target) {
		targetThrust = (int) target;
		throttlePosition = targetThrust;
	}

	public static void setElevatorPosition(double target) {
		if (elePosReq < 50 && elePosReq > -50)
			elePosReq = elePosReq + target;
	}

	public static void setAttitude(double target) {
		mouseWheelAttitude = mouseWheelAttitude + target;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		if (MouseWheelAttitude.length() > 5)
			MouseWheelAttitude = MouseWheelAttitude.substring(0, 5);
	}

	public static void bearingRange() {
		Xpos = sim2.positionX;
		Ypos = sim2.positionY;
		range = Math.sqrt(((Xpos - toPointX) * (Xpos - toPointX)) + ((Ypos - toPointY) * (Ypos - toPointY))) / 1852.0; // nm
																														// to
																														// meters
		bearing = Math.atan2((Ypos - toPointY), (Xpos - toPointX)) - Math.PI;
		Range = String.valueOf(range);
		if (range < 10 && Range.length() > 3)
			Range = Range.substring(0, 3);
		if (range >= 10 && range < 30 && Range.length() > 4) {
			Range = Range.substring(0, 4);
		}
		if (range >= 30 && range < 100 && Range.length() > 2) {
			Range = Range.substring(0, 2);
		}
		if (range > 100 && Range.length() > 3) {
			Range = Range.substring(0, 3);
		}
		Bearing = String.valueOf(bearing);
		if (Bearing.length() > 5)
			Bearing = Bearing.substring(0, 5);
		aX1 = ndXC + (30 * Math.sin(bearing - hdgND)); // head of needle
		aY1 = ndYC - (30 * Math.cos(bearing - hdgND));
		aX2 = ndXC + (110 * Math.sin(bearing - hdgND));
		aY2 = ndYC - (110 * Math.cos(bearing - hdgND));
		aXL = ndXC + (95 * Math.sin(bearing - hdgND - 0.05));
		aYL = ndYC - (95 * Math.cos(bearing - hdgND - 0.05));
		aXR = ndXC + (95 * Math.sin(bearing - hdgND + 0.05));
		aYR = ndYC - (95 * Math.cos(bearing - hdgND + 0.05));

		bX1 = ndXC - (30 * Math.sin(bearing - hdgND)); // tail of needle
		bY1 = ndYC + (30 * Math.cos(bearing - hdgND));
		bX2 = ndXC - (95 * Math.sin(bearing - hdgND));
		bY2 = ndYC + (95 * Math.cos(bearing - hdgND));
		bXL = ndXC - (110 * Math.sin(bearing - hdgND - 0.05));
		bYL = ndYC + (110 * Math.cos(bearing - hdgND - 0.05));
		bXR = ndXC - (110 * Math.sin(bearing - hdgND + 0.05));
		bYR = ndYC + (110 * Math.cos(bearing - hdgND + 0.05));

	}

	// radius of nd = 120 pixels
	private static void NDGraphics() {
		// toPoint
		Xpos = sim2.positionX;
		Ypos = sim2.positionY;

		if (ToPoint.equals("RWY36")) {
			toPointX = RWY36[0];
			toPointY = RWY36[1];
		}
		if (ToPoint.equals("RWY18")) {
			toPointX = RWY18[0];
			toPointY = RWY18[1];
		}

		range = Math.sqrt(((Xpos - toPointX) * (Xpos - toPointX)) + ((Ypos - toPointY) * (Ypos - toPointY))) / 1852.0; // nm
																														// to
																														// meters
		bearing = Math.atan2((Ypos - toPointY), (Xpos - toPointX)) - Math.PI;
		wPX = ndXC + (int) (120.0 * (range / ndScale) * Math.sin(bearing - hdgND));
		wPY = ndYC - (int) (120.0 * (range / ndScale) * Math.cos(bearing - hdgND));

		for (int i = 0; i < 5; i++) { // plot the runway points
			distE = Math.sqrt(((sim2.positionX - RWYX[i]) * (sim2.positionX - RWYX[i]))
					+ ((sim2.positionY - RNDY[i]) * (sim2.positionY - RNDY[i]))) / 1852.0; // nm
																							// to
																							// meters
			bear2 = Math.atan2((sim2.positionY - RNDY[i]), (sim2.positionX - RWYX[i])) - Math.PI;

			RWGX[i] = ndXC + (int) (120.0 * (distE / ndScale) * Math.sin(bear2 - hdgND));
			RWGY[i] = ndYC - (int) (120.0 * (distE / ndScale) * Math.cos(bear2 - hdgND));
		}

	}

	private static void PFDRunway() { // Runway on PFD
		for (int i = 0; i <= 3; i++) {
			Point3D point1 = new Point3D(RWYX[i], RWYY[i], 0);
			Point3D point2 = new Point3D(0, 0, 0);
			if (i < 3) {
				point2 = new Point3D(RWYX[i + 1], RWYY[i + 1], 0);
			} else {
				point2 = new Point3D(RWYX[0], RWYY[0], 0);
			}
			Plane3D plane = new Plane3D(sim2.positionX, sim2.positionY, sim2.positionZ, sim2.a11q, sim2.a21q,
					sim2.a31q);
			Point3D intercept = lineInterceptPlane(plane, point1, point2);

			// if intercept point lies within extremities of two points,
			// truncate line
			boolean pt1 = sideOfPlane(point1, plane);
			boolean pt2 = sideOfPlane(point2, plane);

			if (pt1 && pt2) {
				rwy[i] = new Line3D(point1, point2);
			}
			if (pt1 && !pt2) {
				rwy[i] = new Line3D(point1, intercept);
			}
			if (!pt1 && pt2) {
				rwy[i] = new Line3D(intercept, point2);
			}
			if (!pt1 && !pt2) {
				rwy[i] = null;
			}
		}
		Beta = sim2.betaQ;
		Gamma = sim2.gammaQ;
		for (int i = 0; i <= 3; i++) {
			if (rwy[i] != null) {
				range1 = Math.sqrt(((sim2.positionX - rwy[i].ptlx1) * (sim2.positionX - rwy[i].ptlx1))
						+ ((sim2.positionY - rwy[i].ptly1) * (sim2.positionY - rwy[i].ptly1))); // meters

				bear31 = Math.atan2((rwy[i].ptly1 - sim2.positionY), (rwy[i].ptlx1 - sim2.positionX));
				objAngle1 = Math.atan(-sim2.positionZ / range1); // angle of
																	// object
																	// below
																	// horizon
																	// for pfd
																	// runway

				hdgND = sim2.alphaQ;

				if (hdgND >= 0.0 && bear31 >= 0.0) {
					relB1 = bear31 - hdgND;
				}
				if (hdgND < 0.0 && bear31 < 0.0) {
					relB1 = bear31 - hdgND;
				}
				if (hdgND < 0.0 && bear31 >= 0.0) {
					if (bear31 > (hdgND + Math.PI)) {
						relB1 = bear31 - hdgND - (2.0 * Math.PI);
					}
					if (bear31 < (hdgND + Math.PI)) {
						relB1 = bear31 - hdgND;
					}
				}
				if (hdgND >= 0.0 && bear31 < 0.0) {
					if (bear31 > (hdgND - Math.PI)) {
						relB1 = bear31 - hdgND;
					}
					if (bear31 < (hdgND - Math.PI)) {
						relB1 = (2.0 * Math.PI) + bear31 - hdgND;
					}
				}

				cDX1 = (Beta + objAngle1) * pitchFactor * Math.sin(Gamma);
				cDY1 = (Beta + objAngle1) * pitchFactor * Math.cos(Gamma);

				cDX2 = PDx + cDX1 + (relB1 * pitchFactor * Math.cos(Gamma));
				cDY2 = PDy + cDY1 - (relB1 * pitchFactor * Math.sin(Gamma));

				// *******************************************************************
				range2 = Math.sqrt(((sim2.positionX - rwy[i].ptlx2) * (sim2.positionX - rwy[i].ptlx2))
						+ ((sim2.positionY - rwy[i].ptly2) * (sim2.positionY - rwy[i].ptly2))); // meters

				bear32 = Math.atan2((rwy[i].ptly2 - sim2.positionY), (rwy[i].ptlx2 - sim2.positionX));
				objAngle2 = Math.atan(-sim2.positionZ / range2); // angle of
																	// object
																	// below
																	// horizon
																	// for pfd
																	// runway

				if (hdgND >= 0.0 && bear32 >= 0.0) {
					relB2 = bear32 - hdgND;
				}
				if (hdgND < 0.0 && bear32 < 0.0) {
					relB2 = bear32 - hdgND;
				}
				if (hdgND < 0.0 && bear32 >= 0.0) {
					if (bear32 > (hdgND + Math.PI)) {
						relB2 = bear32 - hdgND - (2.0 * Math.PI);
					}
					if (bear32 < (hdgND + Math.PI)) {
						relB2 = bear32 - hdgND;
					}
				}
				if (hdgND >= 0.0 && bear32 < 0.0) {
					if (bear32 > (hdgND - Math.PI)) {
						relB2 = bear32 - hdgND;
					}
					if (bear32 < (hdgND - Math.PI)) {
						relB2 = (2.0 * Math.PI) + bear32 - hdgND;
					}
				}

				cDX3 = (Beta + objAngle2) * pitchFactor * Math.sin(Gamma);
				cDY3 = (Beta + objAngle2) * pitchFactor * Math.cos(Gamma);

				cDX4 = PDx + cDX3 + (relB2 * pitchFactor * Math.cos(Gamma));
				cDY4 = PDy + cDY3 - (relB2 * pitchFactor * Math.sin(Gamma));
				// RWGX[i+1] = (int)cDX2;
				// RWGY[i+1] = (int)cDY2;
				// *****************************************************************************
				drawLines[i] = new Line2D(cDX2, cDY2, cDX4, cDY4);

			}
			if (rwy[i] == null) {
				drawLines[i] = null;
			}
		}
	}

	private static void PFDRunwayCL() { // Runway centerline on PFD
		for (int i = 0; i <= 1; i++) {
			Point3D point1 = new Point3D(RCLX[i], RCLY[i], 0);
			Point3D point2 = new Point3D(0, 0, 0);
			if (i < 1) {
				point2 = new Point3D(RCLX[i + 1], RCLY[i + 1], 0);
			} else {
				point2 = new Point3D(RCLX[0], RCLY[0], 0);
			}
			Plane3D plane = new Plane3D(sim2.positionX + (5.0 * sim2.a11q), sim2.positionY + (5.0 * sim2.a21q),
					sim2.positionZ + (5.0 * sim2.a31q), sim2.a11q, sim2.a21q, sim2.a31q);
			Point3D intercept = lineInterceptPlane(plane, point1, point2);

			// if intercept point lies within extremities of two points,
			// truncate line
			boolean pt1 = sideOfPlane(point1, plane);
			boolean pt2 = sideOfPlane(point2, plane);

			if (pt1 && pt2) {
				rwy[i] = new Line3D(point1, point2);
			}
			if (pt1 && !pt2) {
				rwy[i] = new Line3D(point1, intercept);
			}
			if (!pt1 && pt2) {
				rwy[i] = new Line3D(intercept, point2);
			}
			if (!pt1 && !pt2) {
				rwy[i] = null;
			}
		}
		Beta = sim2.betaQ;
		Gamma = sim2.gammaQ;
		for (int i = 0; i <= 0; i++) {
			if (rwy[i] != null) {
				range1 = Math.sqrt(((sim2.positionX - rwy[i].ptlx1) * (sim2.positionX - rwy[i].ptlx1))
						+ ((sim2.positionY - rwy[i].ptly1) * (sim2.positionY - rwy[i].ptly1))); // meters

				bear31 = Math.atan2((rwy[i].ptly1 - sim2.positionY), (rwy[i].ptlx1 - sim2.positionX));
				objAngle1 = Math.atan(-sim2.positionZ / range1); // angle of
																	// object
																	// below
																	// horizon
																	// for pfd
																	// runway

				hdgND = sim2.alphaQ;

				if (hdgND >= 0.0 && bear31 >= 0.0) {
					relB1 = bear31 - hdgND;
				}
				if (hdgND < 0.0 && bear31 < 0.0) {
					relB1 = bear31 - hdgND;
				}
				if (hdgND < 0.0 && bear31 >= 0.0) {
					if (bear31 > (hdgND + Math.PI)) {
						relB1 = bear31 - hdgND - (2.0 * Math.PI);
					}
					if (bear31 < (hdgND + Math.PI)) {
						relB1 = bear31 - hdgND;
					}
				}
				if (hdgND >= 0.0 && bear31 < 0.0) {
					if (bear31 > (hdgND - Math.PI)) {
						relB1 = bear31 - hdgND;
					}
					if (bear31 < (hdgND - Math.PI)) {
						relB1 = (2.0 * Math.PI) + bear31 - hdgND;
					}
				}

				cDX1 = (Beta + objAngle1) * pitchFactor * Math.sin(Gamma);
				cDY1 = (Beta + objAngle1) * pitchFactor * Math.cos(Gamma);

				cDX2 = PDx + cDX1 + (relB1 * pitchFactor * Math.cos(Gamma));
				cDY2 = PDy + cDY1 - (relB1 * pitchFactor * Math.sin(Gamma));

				// *******************************************************************
				range2 = Math.sqrt(((sim2.positionX - rwy[i].ptlx2) * (sim2.positionX - rwy[i].ptlx2))
						+ ((sim2.positionY - rwy[i].ptly2) * (sim2.positionY - rwy[i].ptly2))); // meters

				bear32 = Math.atan2((rwy[i].ptly2 - sim2.positionY), (rwy[i].ptlx2 - sim2.positionX));
				objAngle2 = Math.atan(-sim2.positionZ / range2); // angle of
																	// object
																	// below
																	// horizon
																	// for pfd
																	// runway

				if (hdgND >= 0.0 && bear32 >= 0.0) {
					relB2 = bear32 - hdgND;
				}
				if (hdgND < 0.0 && bear32 < 0.0) {
					relB2 = bear32 - hdgND;
				}
				if (hdgND < 0.0 && bear32 >= 0.0) {
					if (bear32 > (hdgND + Math.PI)) {
						relB2 = bear32 - hdgND - (2.0 * Math.PI);
					}
					if (bear32 < (hdgND + Math.PI)) {
						relB2 = bear32 - hdgND;
					}
				}
				if (hdgND >= 0.0 && bear32 < 0.0) {
					if (bear32 > (hdgND - Math.PI)) {
						relB2 = bear32 - hdgND;
					}
					if (bear32 < (hdgND - Math.PI)) {
						relB2 = (2.0 * Math.PI) + bear32 - hdgND;
					}
				}

				cDX3 = (Beta + objAngle2) * pitchFactor * Math.sin(Gamma);
				cDY3 = (Beta + objAngle2) * pitchFactor * Math.cos(Gamma);

				cDX4 = PDx + cDX3 + (relB2 * pitchFactor * Math.cos(Gamma));
				cDY4 = PDy + cDY3 - (relB2 * pitchFactor * Math.sin(Gamma));
				// *****************************************************************************
				drawLines[i] = new Line2D(cDX2, cDY2, cDX4, cDY4);

			}
			if (rwy[i] == null) {
				drawLines[i] = null;
			}
		}
	}

	public static void testLineInterceptPlane() {
		Point3D point1 = new Point3D(3.0, -1.0, -4.0);
		Point3D point2 = new Point3D(9.0, -10.0, -1.0);
		Plane3D plane = new Plane3D(0.0, 0.0, -2.0, 2.0, 5.0, -3.0);
		// Point3D point1 = new Point3D(3.0,3.0,3.0);
		// Point3D point2 = new Point3D(9.0,9.0,9.0);
		// Plane3D plane = new Plane3D(1.0, 1.0, 0.0, 0.0, 0.0, -1.0);
		// Point3D point1 = new Point3D(3.0,7.0,3.0);
		// Point3D point2 = new Point3D(21.0,52.0,30.0);
		// Plane3D plane = new Plane3D(0.0, 0.0, 2.0, 1.0, 1.0, 1.0);
		Point3D intercept = lineInterceptPlane(plane, point1, point2);
		System.out.println("X: " + intercept.ptptx);
		System.out.println("Y: " + intercept.ptpty);
		System.out.println("Z: " + intercept.ptptz);
	}

	public static Point3D lineInterceptPlane(Plane3D pl, Point3D pt1, Point3D pt2) {
		double t = (pl.nva * (pl.ptpx0 - pt1.ptptx)) + (pl.nvb * (pl.ptpy0 - pt1.ptpty))
				+ (pl.nvc * (pl.ptpz0 - pt1.ptptz));
		t = t / ((pl.nva * (pt1.ptptx - pt2.ptptx)) + (pl.nvb * (pt1.ptpty - pt2.ptpty))
				+ (pl.nvc * (pt1.ptptz - pt2.ptptz)));
		double px = pt1.ptptx + (t * (pt1.ptptx - pt2.ptptx));
		double py = pt1.ptpty + (t * (pt1.ptpty - pt2.ptpty));
		double pz = pt1.ptptz + (t * (pt1.ptptz - pt2.ptptz));
		Point3D point = new Point3D(px, py, pz);
		return point;
	}

	public static void testSideOfPlane() {
		Point3D point = new Point3D(0.0, -10.0, 1.0);
		Plane3D plane = new Plane3D(4.0, 6.0, 0.0, 0.0, 0.0, 1.0);
		boolean positive = sideOfPlane(point, plane);
		System.out.println(positive);
	}

	public static boolean sideOfPlane(Point3D pt, Plane3D pl) {
		double D = (pl.nva * pt.ptptx) + (pl.nvb * pt.ptpty) + (pl.nvc * pt.ptptz) - (pl.nva * pl.ptpx0)
				- (pl.nvb * pl.ptpy0) - (pl.nvc * pl.ptpz0);
		// D = D/Math.sqrt((pl.nva * pl.nva)+(pl.nvb * pl.nvb)+(pl.nvb *
		// pl.nvb)); This can be omitted
		if (D > 0)
			return true;
		return false;
	}

	public void verticalSpeedIntercept() {
		vsIX = (((vsIX1 * vsIY2 - vsIY1 * vsIX2) * (vsIX3 - vsIX4))
				- ((vsIX1 - vsIX2) * (vsIX3 * vsIY4 - vsIY3 * vsIX4)))
				/ (((vsIX1 - vsIX2) * (vsIY3 - vsIY4)) - ((vsIY1 - vsIY2) * (vsIX3 - vsIX4)));
		vsIY = (((vsIX1 * vsIY2 - vsIY1 * vsIX2) * (vsIY3 - vsIY4))
				- ((vsIY1 - vsIY2) * (vsIX3 * vsIY4 - vsIY3 * vsIX4)))
				/ (((vsIX1 - vsIX2) * (vsIY3 - vsIY4)) - ((vsIY1 - vsIY2) * (vsIX3 - vsIX4)));
	}

	public static void setMouseWheelAttitude(double mwA) {
		mouseWheelAttitude = mwA;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		if (MouseWheelAttitude.length() > 5)
			MouseWheelAttitude = MouseWheelAttitude.substring(0, 5);
		attitudeOn = false;
	}

	public static void setMouseWheelAttitudeString() {
		MouseWheelAttitude = "---";
	}

	public static void setMouseWheelHeading(double mwH) { // for localizer LLZ
															// only
		if (mwH < 0) {
			mwH = 360 + mwH;
		}
		mouseWheelHeading = mwH;
		MouseWheelHDG = String.valueOf(mwH);
		if (mwH < 10) {
			MouseWheelHDG = MouseWheelHDG.substring(0, 3);
			MouseWheelHDG = "00000".substring(MouseWheelHDG.length()) + MouseWheelHDG;
		}
		if (mwH < 100 && mwH >= 10) {
			MouseWheelHDG = MouseWheelHDG.substring(0, 4);
			MouseWheelHDG = "00000".substring(MouseWheelHDG.length()) + MouseWheelHDG;
		}
		if (MouseWheelHDG.length() > 5)
			MouseWheelHDG = MouseWheelHDG.substring(0, 5);
	}

	public static void setMouseWheelHDG() { // for direct to waypoint only
		if (ToPoint.equals("RWY36")) {
			toPointX = RWY36[0];
			toPointY = RWY36[1];
		}
		if (ToPoint.equals("RWY18")) {
			toPointX = RWY18[0];
			toPointY = RWY18[1];
		}
		fOHDG = Math.atan2(sim2.positionY - toPointY, sim2.positionX - toPointX);
		fOHDG = fOHDG;
		if (fOHDG < 0)
			fOHDG = (2 * Math.PI) + fOHDG;
		fOHDG = (fOHDG * 180 / Math.PI) - 180.0;
		if (fOHDG < 0)
			fOHDG = 360 + fOHDG;
		mouseWheelHDG = (int) fOHDG;
		MouseWheelHDG = String.valueOf(mouseWheelHDG);
		if (MouseWheelHDG.length() > 3)
			MouseWheelHDG = MouseWheelHDG.substring(0, 3);
		if (mouseWheelHDG < 10) {
			MouseWheelHDG = MouseWheelHDG.substring(0, 1);
			MouseWheelHDG = "000".substring(MouseWheelHDG.length()) + MouseWheelHDG;
		}
		if (mouseWheelHDG < 100) {
			MouseWheelHDG = MouseWheelHDG.substring(0, 2);
			MouseWheelHDG = "000".substring(MouseWheelHDG.length()) + MouseWheelHDG;
		}
	}

	public static void setAileronPosition(double ailReqPos) {
		ailPosReq = ailReqPos;
	}

	public static void ElevatorPositionSet(double eleReqPos) { // autopilot
																// attitude 3
		elePosReq = eleReqPos;
	}

	public static void setLandingOff() {
		// fdOn = false;
		glideSlope = false;
		apOn = false;
		attitudeOn = false;
		// glideSlope = false;
		verticalMode = " ";
		speedOn = false;
		hdgOn = false;
		ailPosReq = 0.0;
		elePosReq = 0.0;
		throttlePosition = 0;
		mouseWheelAttitude = 15.0;
		MouseWheelAttitude = String.valueOf(mouseWheelAttitude);
		mouseWheelSpeed = 180;
		MouseWheelSpeed = String.valueOf(mouseWheelSpeed);
		mouseWheelAltitude = 2000;
		MouseWheelAltitude = String.valueOf(mouseWheelAltitude);
	}

}
