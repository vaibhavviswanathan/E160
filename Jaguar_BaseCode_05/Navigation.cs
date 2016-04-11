using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double x_des, y_des, t_des;
        private double x_est_var, y_est_var, xy_est_covar;
        public double desiredX, desiredY, desiredT;
        double desiredX_prev, desiredY_prev, desiredT_prev;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        // TODO MAKE THESE ACTUAL VALUES
        double std_l = 1.2;
        double std_r = 1.2;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;
        public int numParticles_temp = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 1;
        private double w_tot = 1;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        // Motion Planner Variables 
        public double samplingCellSizeX, samplingCellSizeY;
        public int numOccupiedCells;
        public int[] occupiedCellsList;
        public int[] numNodesInCell;
        public Node[,] NodesInCells;
        public Node[] trajList, nodeList;
        public int trajSize, trajCurrentNode, numNodes;

        public bool stpBtnPressed;

        // ---- previous lab stuff

        DateTime previousTimeL;
        double measured_timestepL;
        DateTime previousTimeR;
        double measured_timestepR;

        // PWM error globals
        double errorLInt = 0;
        double errorRInt = 0;
        double wprevL = 0;
        double wprevR = 0;
        double signalL, signalR;

        double rotRateLest = 0;
        double rotRateRest = 0;


        bool within_tracking = false;

        // TrackTrajectory variables
        private int traj_i = 0;

        public class Node
        {
            public double x, y;
            public int lastNode;
            public int nodeIndex;

            public Node()
            {
                x = 0;
                y = 0;
                lastNode = 0;
                nodeIndex = 0;
            }

            public Node(double _x, double _y, int _nodeIndex, int _lastNode)
            {
                x = _x;
                y = _y;
                nodeIndex = _nodeIndex;
                lastNode = _lastNode;
            }

        }


        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();

            // Create particles
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            x_est_var = 0;
            y_est_var = 0;
            xy_est_covar = 0;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // MP variable setup
            occupiedCellsList = new int[numXCells * numYCells];
            numNodesInCell = new int[numXCells * numYCells];
            NodesInCells = new Node[numXCells * numYCells, 500];
            trajList = new Node[maxNumNodes];
            nodeList = new Node[maxNumNodes];
            numNodes = 0;
            trajList[0] = new Node(0, 0, 0, 0);
            trajSize = 0;


        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    //FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // ****************** Additional Student Code: Start ************

            // Students must set motorSignalL and motorSignalR. Make sure
            // they are set between 0 and maxPosOutput. A PID control is
            // suggested.


            double Kp_PWM, Ki_PWM, Kd_PWM;
            Kp_PWM = 12; // 8; //  2.25 * 16;
            Ki_PWM = 2; // 10 * 2;
            Kd_PWM = 0.1;
            double N = 30; // filter coefficient

            DateTime currentTime = DateTime.Now;
            double MTSL = (currentTime - previousTimeL).TotalMilliseconds / 1000;
            if (wheelDistanceL != 0 || MTSL > 0.5)
            {
                measured_timestepL = (currentTime - previousTimeL).TotalMilliseconds / 1000;
                previousTimeL = currentTime;
                rotRateLest = (wheelDistanceL / (2 * Math.PI * wheelRadius)) * pulsesPerRotation / measured_timestepL;
            }
            double MTSR = (currentTime - previousTimeR).TotalMilliseconds / 1000;
            if (wheelDistanceR != 0 || MTSR > 0.5)
            {
                measured_timestepR = (currentTime - previousTimeR).TotalMilliseconds / 1000;
                previousTimeR = currentTime;
                rotRateRest = (wheelDistanceR / (2 * Math.PI * wheelRadius)) * pulsesPerRotation / measured_timestepR;
            }
            double errorL = desiredRotRateL - rotRateLest;
            errorLInt += (wheelDistanceL != 0 || MTSL > 0.5) ? errorL * measured_timestepL : 0;
            double DerrorL = (wheelDistanceL != 0 || MTSL > 0.5) ? N * (errorL - wprevL) : 0;
            wprevL += (wheelDistanceL != 0 || MTSL > 0.5) ? DerrorL * measured_timestepL : 0;
            signalL = Kp_PWM * errorL + Ki_PWM * errorLInt + Kd_PWM * DerrorL;

            double errorR = desiredRotRateR - rotRateRest;
            errorRInt += (wheelDistanceR != 0 || MTSR > 0.5) ? errorR * measured_timestepR : 0;
            double DerrorR = (wheelDistanceR != 0 || MTSR > 0.5) ? N * (errorR - wprevR) : 0;
            wprevR += (wheelDistanceR != 0 || MTSR > 0.5) ? DerrorR * measured_timestepR : 0;
            signalR = Kp_PWM * errorR + Ki_PWM * errorRInt + Kd_PWM * DerrorR;


            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            motorSignalL = (short)((zeroOutput + desiredRotRateL * 100 + signalL) / (1.8519 / 1.8519));// (zeroOutput + u_L);
            motorSignalR = (short)((zeroOutput - desiredRotRateR * 100 - signalR) / (1.6317 / 1.8519));//(zeroOutput - u_R);

            // motorSignalL = (short) -desiredRotRateL;
            //motorSignalR = desiredRotRateR;

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            double satsignalL = motorSignalL - zeroOutput;
            double satsignalR = motorSignalR - zeroOutput;

            // errorLInt += measured_timestep / Ki_PWM * (satsignalL - motorSignalR);

            // ****************** Additional Student Code: End   ************
        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Calculate delta X and delta Y
            //double delta_x = desiredX - x;
            //double delta_y = desiredY - y;
            double delta_x = x_des - x;
            double delta_y = y_des - y;

            // Calculate state
            double pho = Math.Sqrt(Math.Pow(delta_x, 2) + Math.Pow(delta_y, 2));
            double alpha = -t + Math.Atan2(delta_y, delta_x); //check to be within -pi and pi
            double beta = -t - alpha + desiredT; //check to be within -pi and pi

            // Threshold errors

            pho = (Math.Abs(pho) < phoTrackingAccuracy) ? 0 : pho;
            alpha = (Math.Abs(alpha) < alphaTrackingAccuracy) ? 0 : alpha;
            beta = (Math.Abs(beta) < betaTrackingAccuracy) ? 0 : beta;

            // Make sure within -pi and pi
            alpha = (alpha < -Math.PI) ? alpha + 2 * Math.PI : ((alpha > Math.PI) ? alpha - 2 * Math.PI : alpha);
            beta = (beta < -Math.PI) ? beta + 2 * Math.PI : ((beta > Math.PI) ? beta - 2 * Math.PI : beta);

            // Check to see if point is behind robot
            bool behindRobot = (Math.Abs(alpha) > Math.PI / 2) ? true : false;

            // adjust alpha if point is behind robot
            alpha = (behindRobot) ? -t + Math.Atan2(-delta_y, -delta_x) : alpha;
            alpha = (alpha < -Math.PI) ? alpha + 2 * Math.PI : ((alpha > Math.PI) ? alpha - 2 * Math.PI : alpha);

            // calculate desired velocity 
            double desiredV = (behindRobot) ? -Kpho * pho : Kpho * pho;
            double desiredW = Kalpha * alpha + Kbeta * beta;

            // use different controller if within threshold
            if (Math.Sqrt(Math.Pow(delta_x, 2) + Math.Pow(delta_y, 2)) < phoTrackingAccuracy)
            {
                within_tracking = true;

            }
            if (within_tracking && desiredX == desiredX_prev && desiredY == desiredY_prev && desiredT == desiredT_prev)
            {
                beta = -t + desiredT;
                beta = (beta < -Math.PI) ? beta + 2 * Math.PI : ((beta > Math.PI) ? beta - 2 * Math.PI : beta);
                double KbetaNew = -6 * Kbeta;
                desiredW = KbetaNew * beta;
                desiredV = 0;
            }
            else
            {
                within_tracking = false;
            }


            // saturate desired velocity
            // double saturatedV1 = Math.Sign(desiredV) * Math.Min(Math.Abs(desiredV), 0.25);
            // double saturatedW1 = saturatedV1 > 0 ? desiredW * (saturatedV1 / desiredV) : desiredW;

            double L = 2 * robotRadius;
            // double saturatedW = Math.Sign(saturatedW1) * Math.Min(Math.Abs(saturatedW1) * L, 0.25);
            // double saturatedV = saturatedW > 0 ? saturatedV1 * (saturatedW / saturatedW1) : saturatedV1;
            double saturatedV = desiredV;
            double saturatedW = desiredW;


            // desired wheel velocities
            double desiredWheelVelL = -(L * saturatedW - saturatedV);
            double desiredWheelVelR = (L * saturatedW + saturatedV);
            //lower saturation
            double satWheelVelL = 0;
            double satWheelVelR = 0;

            // min rot rate if trying to move
            if (Math.Abs(desiredWheelVelL) > 0)
                satWheelVelL = (Math.Sign(desiredWheelVelL) * Math.Max((double)Math.Abs(desiredWheelVelL), 0.05));
            if (Math.Abs(desiredWheelVelR) > 0)
                satWheelVelR = (Math.Sign(desiredWheelVelR) * Math.Max((double)Math.Abs(desiredWheelVelR), 0.05));

            if (satWheelVelL != 0 && satWheelVelR != 0)
            {
                if (satWheelVelL / desiredRotRateL > satWheelVelR / desiredRotRateR)
                    satWheelVelR = (desiredWheelVelR * satWheelVelL / desiredWheelVelL);
                else
                    satWheelVelL = (desiredWheelVelL * satWheelVelR / desiredWheelVelR);
            }

            desiredWheelVelL = satWheelVelL;
            desiredWheelVelR = satWheelVelR;

            // upper saturation
            satWheelVelL = Math.Sign(desiredWheelVelL) * Math.Min(Math.Abs(desiredWheelVelL), 0.25);
            satWheelVelR = Math.Sign(desiredWheelVelR) * Math.Min(Math.Abs(desiredWheelVelR), 0.25);
            if (Math.Abs(satWheelVelL) / Math.Abs(desiredWheelVelL) < Math.Abs(satWheelVelR) / Math.Abs(desiredWheelVelR))
                satWheelVelR = desiredWheelVelR * Math.Abs(satWheelVelL) / Math.Abs(desiredWheelVelL);
            else
                satWheelVelL = desiredWheelVelL * Math.Abs(satWheelVelR) / Math.Abs(desiredWheelVelR);
            desiredRotRateL = (short)(satWheelVelL / wheelRadius * pulsesPerRotation / (2 * Math.PI));
            desiredRotRateR = (short)(satWheelVelR / wheelRadius * pulsesPerRotation / (2 * Math.PI));

            // desiredRotRateL = (short) (-pulsesPerRotation*2);
            // desiredRotRateR = (short) (-pulsesPerRotation*2);

            desiredX_prev = desiredX;
            desiredY_prev = desiredY;
            desiredT_prev = desiredT;



            // ****************** Additional Student Code: End   ************
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            double distToCurrentNode = Math.Sqrt(Math.Pow(x_est - trajList[trajCurrentNode].x, 2) + Math.Pow(y_est - trajList[trajCurrentNode].y, 2));
            if (distToCurrentNode < 0.3 && trajCurrentNode + 1 < trajSize)
            {
                trajCurrentNode++;
                x_des = trajList[trajCurrentNode].x;
                y_des = trajList[trajCurrentNode].y;
                t_des = 0;
            }

            FlyToSetPoint();
        }

        // This function houses the core motion planner. This function
        // will generate a new trajectory when called. Students must 
        // add their code here.

        private void PRMMotionPlanner()
        {
            // Initialize sampling grid cell variables for weighted
            // random selection of nodes to expand.
            samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
            samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
            numOccupiedCells = 0;
            for (int i = 0; i < numXCells * numYCells; i++)
                numNodesInCell[i] = 0;
            numNodes = 0;


            // ****************** Additional Student Code: Start ************

            // Put code here to expand the PRM until the goal node is reached,
            // or until a max number of iterations is reached.


            // Create and add the start Node
            Node firstNode = new Node(x_est, y_est, 0, 0);
            AddNode(firstNode);

            // Create the goal node
            Node goalNode = new Node(desiredX, desiredY, 0, 0);


            // Loop until path created
            bool pathFound = false;
            int maxIterations = maxNumNodes;
            int iterations = 0;
            Random randGenerator = new Random();


            

            while (iterations < maxIterations && !pathFound)
            {
                int randCellNumber = randGenerator.Next(0, numOccupiedCells);
                int randNodeNumber = randGenerator.Next(0, numNodesInCell[occupiedCellsList[randCellNumber]]);

                Node randExpansionNode = NodesInCells[occupiedCellsList[randCellNumber], randNodeNumber];
                
                
                // Compute random distance for expanded node
                double randDist = 5.0*randGenerator.NextDouble();
                double randOrientation = -Math.PI + 2*Math.PI * randGenerator.NextDouble();

                // Determine x and y position of expanded node
                double newX = randExpansionNode.x + randDist * Math.Cos(randOrientation);
                double newY = randExpansionNode.y + randDist * Math.Sin(randOrientation);

                Node newNode = new Node(newX, newY, numNodes, randExpansionNode.nodeIndex);

                // Check for collisions
                if (!map.CollisionFound(randExpansionNode, newNode, robotRadius))
                {
                    AddNode(newNode); //TODO create tolerance

                    // Check connection to goal
                    if (!map.CollisionFound(newNode, goalNode, robotRadius))
                    {
                        goalNode.nodeIndex = numNodes;
                        goalNode.lastNode = newNode.nodeIndex;
                        AddNode(goalNode);
                        pathFound = true;
                    }
                }

                // Increment number of iterations
                iterations++;
            }




            // Create the trajectory to follow
            BuildTraj(goalNode);

            
            // ****************** Additional Student Code: End   ************




        }




        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        int GetCellNumber(double x, double y)
        {
            int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
            return cell;
        }

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        void AddNode(Node n)
        {
            int cellNumber = GetCellNumber(n.x, n.y);
            if (numNodesInCell[cellNumber] < 1)
            {
                occupiedCellsList[numOccupiedCells] = cellNumber;
                numOccupiedCells++;
            }

            if (numNodesInCell[cellNumber] < 400)
            {
                NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
                numNodesInCell[cellNumber]++;

                // Add to nodelist
                nodeList[numNodes] = n;
                numNodes++;
            }
            return;
        }


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        void BuildTraj(Node goalNode)
        {
            Node[] tempList = new Node[maxNumNodes];
            for (int j = 0; j < maxNumNodes; j++)
                trajList[j] = new Node(0, 0, 0, 0);

            tempList[0] = goalNode;
            int i = 1;

            // Make backwards traj by looking at parent of every child node
            while (tempList[i - 1].nodeIndex != 0)
            {
                tempList[i] = nodeList[tempList[i - 1].lastNode];
                i++;
            }

            // Reverse trajectory order
            for (int j = 0; j < i; j++)
            {
                trajList[j] = tempList[i - j - 1];
            }

            // Set size of trajectory and initialize node counter
            trajSize = i;
            trajCurrentNode = 0;

            OptimizeTraj();

            return;
        }

        void OptimizeTraj(){
            Tuple<double, int[]> optTrajAuxOut = OptimizeTrajAux(0, new int[0]);
            int [] trajListInts = optTrajAuxOut.Item2;
            Node[] trajListTemp = new Node[trajListInts.Length];
            for (int i = 0; i < trajListInts.Length; i++)
            {
                trajListTemp[i] = trajList[trajListInts[i]];
            }
            trajList = trajListTemp;
            trajSize = trajList.Length-1;
            trajCurrentNode = 0;


            return;
        }

        public class Tuple<T, U>
        {
            public T Item1 { get; private set; }
            public U Item2 { get; private set; }

            public Tuple(T item1, U item2)
            {
                Item1 = item1;
                Item2 = item2;
            }
        }

        // This function returns the sum of squared distances from node to goal
        Tuple<double, int[]> OptimizeTrajAux(int trajNode, int[] trajNodeParents)
        {
            Node currentNode = trajList[trajNode];
            Node nextNode;
            int[] trajNodesFinal;
            // see if you can go straight to goal
            if (!map.CollisionFound(trajList[trajNode], trajList[trajSize-1], robotRadius))
            {
                nextNode = trajList[trajSize-1];
                // calculate squared distance to goal
                double dist = Math.Pow(nextNode.x - currentNode.x, 2) + Math.Pow(nextNode.y - currentNode.y, 2);
                // make final trajectory array
                trajNodesFinal = new int[trajNodeParents.Length + 2];
                for (int i = 0; i < trajNodeParents.Length; i++)
                    trajNodesFinal[i] = trajNodeParents[i];
                trajNodesFinal[trajNodesFinal.Length - 2] = trajNode;
                trajNodesFinal[trajNodesFinal.Length - 1] = trajSize-1;
                return new Tuple<double,int[]>(dist, trajNodesFinal);
            }
            // else check each node in trajList /parents and pick shortest
            double minDist = double.PositiveInfinity;
            int[] completedPath = new int[1];

            // make copy of parents node to feed to possible children
            int[] trajNodeParentsNew = new int[trajNodeParents.Length + 1];
            for(int i = 0; i<trajNodeParents.Length; i++)
                trajNodeParentsNew[i] = trajNodeParents[i];
            trajNodeParentsNew[trajNodeParentsNew.Length - 1] = trajNode;

            for (int i = 0; i < trajSize-1; i++)
            {
                // check if next node is one of the parents or self
                if (i == trajNode) continue;
                bool letsContinue = false;
                for (int j = 0; j < trajNodeParents.Length; j++)
                {
                    if (i == trajNodeParents[j]) letsContinue = true; ;
                }
                if (letsContinue) continue;
                nextNode = trajList[i];
                if(map.CollisionFound(trajList[trajNode], trajList[trajSize-1], robotRadius)) continue;
                // else compare i's dist to minDist
                Tuple<double, int[]> trialNodeOutput = OptimizeTrajAux(i, trajNodeParentsNew);
                double dist = trialNodeOutput.Item1;
                dist += Math.Pow(nextNode.x - currentNode.x, 2) + Math.Pow(nextNode.y - currentNode.y, 2); // add distance to i
                int[] nextPath = trialNodeOutput.Item2;
                if (dist < minDist)
                {
                    minDist = dist;
                    completedPath = nextPath;
                }

            }
            return new Tuple<double,int[]>(minDist,completedPath);
        }

 


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // above.

            // compute diffEncoderPulses
            double diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            double diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;

            // set previous encoder values
            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;


            // correct for rollover
            diffEncoderPulseL = diffEncoderPulseL < -encoderMax / 2 ?
                diffEncoderPulseL + encoderMax : diffEncoderPulseL;
            diffEncoderPulseR = diffEncoderPulseR < -encoderMax / 2 ?
                diffEncoderPulseR + encoderMax : diffEncoderPulseR;
            diffEncoderPulseL = diffEncoderPulseL > encoderMax / 2 ?
                diffEncoderPulseL - encoderMax : diffEncoderPulseL;
            diffEncoderPulseR = diffEncoderPulseR > encoderMax / 2 ?
                diffEncoderPulseR - encoderMax : diffEncoderPulseR;

            // reverse direction for right side
            diffEncoderPulseR = -diffEncoderPulseR;

            // compute distance travelled in timestep
            wheelDistanceR = (diffEncoderPulseR * 2 * Math.PI * wheelRadius) / pulsesPerRotation; //check wheel r
            wheelDistanceL = (diffEncoderPulseL * 2 * Math.PI * wheelRadius) / pulsesPerRotation; //check wheel r

            // compute angle and distance travelled
            distanceTravelled = (wheelDistanceR + wheelDistanceL) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius); //check robot radius



            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x = x + distanceTravelled * Math.Cos(t + angleTravelled / 2);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled / 2);
            t = t + angleTravelled;
            if (t > Math.PI)
                t = t - 2 * Math.PI;
            else if (t < -Math.PI)
                t = t + 2 * Math.PI;


            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab


            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF
            bool weShouldReSample = ((wheelDistanceL != 0) || (wheelDistanceR != 0)) && newLaserData;
            newLaserData = false; // reset newLaserData

            // propogate particles using odomotery
            for (int i = 0; i < numParticles; i++)
            {
                double rand_l = wheelDistanceL * (1 + std_l * RandomGaussian() / w_tot);
                double rand_r = wheelDistanceR * (1 + std_r * RandomGaussian() / w_tot);

                // compute angle and distance travelled
                double randDistance = (rand_r + rand_l) / 2;
                double randAngle = (rand_r - rand_l) / (2 * robotRadius);

                // add this to a particle 
                if (propagatedParticles[i].Equals(null))
                {
                    Console.Write(' ');
                }
                if (particles[i].Equals(null))
                {
                    Console.Write(' ');
                }
                propagatedParticles[i].x = particles[i].x + randDistance * Math.Cos(particles[i].t + randAngle / 2);
                propagatedParticles[i].y = particles[i].y + randDistance * Math.Sin(particles[i].t + randAngle / 2);
                propagatedParticles[i].t = particles[i].t + randAngle;

                // ensures that angle stays with -pi and pi
                if (propagatedParticles[i].t > Math.PI)
                    propagatedParticles[i].t = propagatedParticles[i].t - 2 * Math.PI;
                else if (propagatedParticles[i].t < -Math.PI)
                    propagatedParticles[i].t = propagatedParticles[i].t + 2 * Math.PI;



            }


            if (weShouldReSample)
            {




                for (int i = 0; i < numParticles; i++)
                {
                    CalculateWeight(i);
                }


                // resample particles

                int maxSamples = 10;
                int[] sampled_inds = new int[numParticles * maxSamples];

                // double w_tot = 0; // find w_tot
                for (int i = 0; i < numParticles; i++)
                {
                    if (propagatedParticles[i].w > w_tot)
                        w_tot = propagatedParticles[i].w;
                }

                int bufferLimit = 0;
                for (int i = 0; i < numParticles; i++) // sample particles
                {
                    double particlesToAdd_temp = (propagatedParticles[i].w / w_tot);
                    particlesToAdd_temp = (propagatedParticles[i].w / w_tot) * maxSamples;
                    int particlesToAdd = particlesToAdd_temp > 0 ? Math.Min((int)(particlesToAdd_temp) + 1, 10) : 0;
                    for (int j = bufferLimit; j < bufferLimit + particlesToAdd; j++)
                        sampled_inds[j] = i;
                    bufferLimit += particlesToAdd;
                }

                for (int i = 0; i < numParticles; i++) // randomly sample from resampled
                {
                    int j = random.Next(0, bufferLimit);
                    int p = sampled_inds[j];
                    particles[i].x = propagatedParticles[p].x;
                    particles[i].y = propagatedParticles[p].y;
                    particles[i].t = propagatedParticles[p].t;
                }
            }
            else
            {
                for (int i = 0; i < numParticles; i++) // randomly sample from resampled
                {
                    particles[i].x = propagatedParticles[i].x;
                    particles[i].y = propagatedParticles[i].y;
                    particles[i].t = propagatedParticles[i].t;
                }
            }

            // average all particle states
            x_est = 0; y_est = 0; t_est = 0;
            for (int i = 0; i < numParticles; i++)
            {
                x_est += particles[i].x / numParticles;
                y_est += particles[i].y / numParticles;
                t_est += particles[i].t / numParticles;
            }

            x_est_var = 0; y_est_var = 0; xy_est_covar = 0;
            for (int i = 0; i < numParticles; i++)
            {
                x_est_var += Math.Pow(particles[i].x - x_est, 2) / (numParticles - 1);
                y_est_var += Math.Pow(particles[i].y - y_est, 2) / (numParticles - 1);
                xy_est_covar += (particles[i].x - x_est) * (particles[i].y - y_est) / (numParticles - 1);
            }



            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {

            double weight = 0;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated weight. Feel free to use the
            // function map.GetClosestWallDistance from Map.cs.

            if (inReachableSpace(p) || true)
            {

                double sigma_laser_percent = 0.01; // ( 1%, from datasheet);
                double sigma_wall = 0.03; // cm

                // take 8 arcs of the laser pi/4 apart
                int numArcs = 5;
                for (int i = 0; i < numArcs; i++)
                {
                    int laseriter = (int)Math.Round(((double)i / numArcs) * (LaserData.Length));
                    double laserangle = laserAngles[laseriter];
                    double sensor_measurement = (double)(LaserData[laseriter]) / 1000;
                    double minDist = map.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t - Math.PI / 2 + laserangle);

                    double sigma_laser = Math.Max(0.01, sigma_laser_percent * sensor_measurement);
                    double sigma = 5 * Math.Sqrt(Math.Pow(sigma_laser, 2) + Math.Pow(sigma_wall, 2));

                    double prob = 1 / (Math.Sqrt(2 * Math.PI)) * Math.Exp(-Math.Pow(sensor_measurement - minDist, 2) / (2 * Math.Pow(sigma, 2)));
                    if (sensor_measurement == 6.0) prob /= 1000; // range at infinity is 6.0. getting 6.0 and 6.0 shouldnt give you a perfect match.
                    weight += prob;


                }

            }
            propagatedParticles[p].w = weight;

        }

        bool inReachableSpace(int p)
        {
            double px = propagatedParticles[p].x;
            double py = propagatedParticles[p].y;
            bool IRS = (py < 2.794) && (py > -2.74);
            IRS &= (px < -3.55 / 2) ? (py > 0) || (py < -2.74) : true;
            IRS &= (px > 3.55 / 2) ? (py > 0) || (py < -2.74) : true;
            return true;
        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {


            numParticles = numParticles_temp;
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            // Set particles in random locations and orientations within environment
            for (int i = 0; i < numParticles; i++)
            {

                // Either set the particles at known start position [0 0 0],  
                // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
                    SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
                    SetStartPos(i);
            }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
            // It might be helpful to use boundaries defined in the
            // Map.cs file (e.g. map.minX)

            particles[p].x = map.minX + (map.maxX - map.minX) * random.NextDouble();
            particles[p].y = map.minY + (map.maxY - map.minY) * random.NextDouble();
            particles[p].t = -Math.PI + 2 * Math.PI * random.NextDouble();




            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
