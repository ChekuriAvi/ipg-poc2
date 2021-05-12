/*
******************************************************************************
**  CarMaker - Version 9.1.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmd Line ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
** 	User_Traffic_Calc ()
**	User_VehicleControl_Calc ()
**	User_Brake_Calc ()           in Vhcl_Calc ()
**	User_Calc ()
**	User_Check_IsIdle ()
**
**	User_Out ()
**
**
** APO Communication:
**
**	User_ApoMsg_Eval ()
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(XENO)
# include <mio.h>
#endif

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include <rbs.h>

#include "IOVec.h"
#include "User.h"

/*von hier*/

#include "Log.h"
#include "Vehicle/Sensor_Line.h"
#include "Vehicle/Sensor_Road.h"
#include "EHorizon.h"
#include "Vehicle/Sensor_Radar.h"
#include "Vehicle/Sensor_Object.h"
#include "Vehicle/Sensor_ObjectByLane.h"
#include "Vehicle/Sensor_Camera.h"

/*bis hier*/

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */


int UserCalcCalledByAppTestRunCalc = 0;

/*von hier*/

double Deviation_distance_path; /*Deviation distance of preview point (which is in my case in middle of front axle) from path*/
double Deviation_angle_path; /*Deviation angle of preview point (which is in my case in middle of front axle) from path*/
double Path_curve_xy_ahead; /*curvature at preview point ahead*/
/*double vhcl_x, vhcl_y, vhcl_z;*/
double v_target, v_final; /*target speed and final speed*/
double curvature_factor, g, v_calc; /*curvature safety factor (the car should not go into curvatures with physical limits), gravity coefficient, calculated velocity for curvature */
double curvature_start, curvature_max, curvature_end; /*longitudinal coordinates of curvature geometry*/
/*bool curvature_passed; /*this boolean is for checking whether the vehicle has passed the curvature_max already*/
double cruising_speed; /*speed in m/s which the vehicle will try to reach if there are no limitations*/
double v_dif; /*delta of v_final and Vehicle.v; v_dif = v_final - Vehicle.v*/
double speed_limit_ahead/*, lane_id_ahead*/; /*information about the speed limit and the lane id ahead of vehicle*/
int lane_change; /*this int is to be used to start a lane change via DVA*/
double lane_offset, lane_change_time; /*double to trigger the vehicle for a lane offset due to a lane change command; time how long a lane change will take in s*/
int current_lane; /*internal id of current lane*/
double current_lane_type, left_lane_type, right_lane_type; /*type of current lane (lane along the route); type of left lane*/
int detected_obj, relevant_target, brake_for_traffic; /*number of objects detected by radar sensor; ID of the relevant target in the object list; brake when traffic ahead and overtaking is not possible*/
double distance_relevant_target, speed_relevant_target; /* distance to the relevant target; relative speed of the relevant target*/
bool overtaking, passed; /*overtaking: false if no overtaking maneuver in process, true if overtaking in process; passed is true if traffic is passed*/
/*double obstacle_probability;*/
double test, test_1, test_2; /*test variable*/
int test_int;
int radar_traffic_n_detected;
double rel_course_angle_traffic;
int detected_obj_left, detected_obj_right; /*if object is detected set to 1 otherwise to 0*/
double distance_obj_left, distance_obj_right, speed_obj_left, speed_obj_right;

tEHorizon* MyEHorizon = NULL;
double Preview_Pts[20][3];
/*array out;*/

/*bis hier*/

tUser	User;


/*von hier*/

/*PI-controller for gas / brake*/
double pid_ctrl(double e) {
    static double esum;
    /*static double ealt;*/
    double y;

    esum = esum + e;
    y = 0.21 * e + 0.0001 * esum /*+ Kd * (e � ealt) / Ta*/; /*0.51 und 0.0001*/
    /*ealt = e;*/
    /*Log("y: %.2f \n", y);*/

    return y;
}

/*PI-controller for steering*/
double pid_ctrl_steer(double e) {
    static double esum;
    /*static double ealt;*/
    double y;

    esum = esum + e;
    y = 2 * e + 0.00006 * esum /*+ Kd * (e � ealt) / Ta*/; /*2 and 0.00005*/
    /*ealt = e;*/
    /*Log("y: %.2f \n", y);*/

    return y;
}

/*bis hier*/


/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User, 0, sizeof(User));
    
    /*von hier*/
    g = 9.81;
    curvature_factor = 0.6;
    cruising_speed = 50; /*50 m/s = 180 km/h*/
    v_dif = 0;

    /*bis hier*/

    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	const tIOConfig *cf;
	const char *defio = IO_GetDefault();
	LogUsage(" -io %-12s Default I/O configuration (%s)\n", "default",
	    (defio!=NULL && strcmp(defio, "none")!=0) ? defio : "minimal I/O");
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("default" /* or "demoapp", "demorbs,demofr" etc. */);

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    return argv;
}



/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/

int
User_Init (void)
{
    return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    int i;

    /*von hier*/
    
    DDefInt(NULL, "Current_Lane", "", &current_lane, DVA_None);
    DDefDouble(NULL, "Current_Lane_Type", "", &current_lane_type, DVA_None);
    DDefDouble(NULL, "Left_Lane_Type", "", &left_lane_type, DVA_None);
    DDefDouble(NULL, "Right_Lane_Type", "", &right_lane_type, DVA_None);
    DDefDouble(NULL, "Deviation_Distance_Path", "m", &Deviation_distance_path, DVA_None);
    DDefDouble(NULL, "Deviation_Angle_Path", "rad", &Deviation_angle_path, DVA_None);
    DDefDouble(NULL, "Path_Curve_XY_Ahead", "1/m", &Path_curve_xy_ahead, DVA_None);
    DDefDouble(NULL, "v_Calc", "m/s", &v_calc, DVA_None);
    DDefDouble(NULL, "v_Target", "m/s", &v_target, DVA_None);
    DDefDouble(NULL, "v_Final", "m/s", &v_final, DVA_None);
    DDefDouble(NULL, "Speed_Limit_ahead", "m/s", &speed_limit_ahead, DVA_None);
    /*DDefDouble(NULL, "Lane_ID_Ahead", "m/s", &lane_id_ahead, DVA_DM);*/
    DDefInt(NULL, "Lane_Change", "", &lane_change, DVA_DM);
    DDefInt(NULL, "Detected_Obj", "", &detected_obj, DVA_None);
    DDefInt(NULL, "Relevant_Target", "", &relevant_target, DVA_None);
    /*DDefDouble(NULL, "Obstacle_Probability", "", &obstacle_probability, DVA_None);*/
    DDefDouble(NULL, "Distance_Relevant_Target", "m", &distance_relevant_target, DVA_None);
    DDefDouble(NULL, "Speed_Relevant_Target", "m/s", &speed_relevant_target, DVA_None);
    DDefDouble(NULL, "Test", "", &test, DVA_None);
    DDefDouble(NULL, "Test_1", "", &test_1, DVA_None);
    DDefDouble(NULL, "Test_2", "", &test_2, DVA_None);
    DDefInt(NULL, "Radar_Traffic_n_Detected", "", &radar_traffic_n_detected, DVA_None);
    DDefInt(NULL, "Test_Int", "", &test_int, DVA_None);
    /*DDefDouble(NULL, "v_Long_Radar", "", &v_long_radar, DVA_None);*/
    DDefDouble(NULL, "Rel_Course_Angle_Traffic", "rad", &rel_course_angle_traffic, DVA_None);
    DDefInt(NULL, "Detected_Obj_Left", "", &detected_obj_left, DVA_None);
    DDefInt(NULL, "Detected_Obj_Right", "", &detected_obj_right, DVA_None);

    /*bis hier*/

    for (i=0; i<N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefDouble (NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
#if !defined(LABCAR)
    RBS_DeclQuants();
#endif
}


/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;

    return rv;
}



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;

    for (i=0; i<N_USEROUTPUT; i++)
	User.Out[i] = 0.0;

    /*von hier*/

    Path_curve_xy_ahead = 0;
    curvature_start = -1;
    curvature_end = -1;
    curvature_max = -1;
    /*curvature_passed = false;*/
    lane_change = 0;
    lane_change_time = 2.0;
    lane_offset = 0;
    current_lane = 0;
    brake_for_traffic = 0;
    overtaking = false;
    passed = false;
    relevant_target = -1;

    /*bis hier*/
    
    if (IO_None)
	return rv;

#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
#endif

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)
{

    /*von hier*/

    tEHorizonCfgIF CfgIF;

    CfgIF.Type = EHorizonType_RoutePath;
    CfgIF.rObjId = Env.Route.ObjId;
    CfgIF.ehMask = EHOT_MIN | EHOT_GRADE | EHOT_SPEEDLIM;
    CfgIF.nREmax = 5;
    CfgIF.ds = 10;
    CfgIF.maxrange = 200;

    MyEHorizon = EHorizon_New(&CfgIF, "My EHorizon");

    /*bis hier*/

    return 0;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{
    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{
    return 0;
}



/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{

    /*von hier*/

    EHorizon_Delete(MyEHorizon);
    MyEHorizon = NULL;

    /*bis hier*/

    return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    if (SimCore.State != SCState_Simulate)
	return;

    /*von hier*/

    /*using eHorizon for detecting speed limits ahead of the vehicle's current position*/
    tEHorizonIF IF;
    if (SimCore.State != SCState_Simulate)
        return;
    IF.PreviewDist = 100;
    IF.s = Car.sRoad + IF.PreviewDist; /*marker position at preview Dist from vehicle*/
    IF.nDynPath = 0;
    
    EHorizon_Eval(MyEHorizon, &IF);

    if (IF.nSL > 0) { /*in case the marker of eHorizon already passed the end of route, do not look for new speed limits, but keep the lst one nstead*/
        speed_limit_ahead = IF.SpeedLimit->val;
        if (speed_limit_ahead == -1) { /*no speed limit*/
            speed_limit_ahead = cruising_speed;
        }
    }



      
    /*current_lane = RoadSensor[0].Act.tMidLane; /*RoadSensor1 hat die ID=0*/
    Deviation_distance_path = RoadSensor[0].Path.Deviation.Dist;
    Deviation_angle_path = RoadSensor[0].Path.Deviation.Ang;
    
    /*
    v = Sqroot(� * r * g)
        �    Reibungszahl
        r    Radius der Kreisbahn(Kr�mmungsradius der Kurve)
        g    Fallbeschleunigung
    */
    
    /*0.0001 is my minimum for a curvature*/
   
    if (DrivMan.SpeedLimit == -1) { /*if there is no speed limit, use CruiseSpeed instead*/
        DrivMan.SpeedLimit = cruising_speed;
    }

    
    /*check whether there is a curvature ahead detected, if so caclulate curvature parameters and v_calc
    * check whether vehicle is in a curvature right now, if so calculate v_calc
    */

    if ((curvature_start == -1) && (curvature_end == -1)) {/*no curvature detected*/
        
        v_calc = -1; /*v_calc is "deactivated*/

        if (((speed_limit_ahead == -1) || (speed_limit_ahead > cruising_speed)) && (DrivMan.SpeedLimit > cruising_speed)) { /*no speed limit or speed limit is higher than cruising speed; vehicle uses Cruising speed instead*/
            v_target = cruising_speed;
        }
        else { /*there is a speed limit which is lower than cruising speed. Stay with speed limit*/
            if (DrivMan.SpeedLimit >= speed_limit_ahead) {
                v_target = speed_limit_ahead;
            }
            else {
                v_target = DrivMan.SpeedLimit;
            }
        }

        v_final = v_target;

    }

    if ((fabs(RoadSensor[1].Path.CurveXY) >= 0.0001) && /*curvature ahead detected*/
        ((sqrt(Car.Tire[FR].muRoad * fabs(1 / RoadSensor[1].Path.CurveXY) * g) * curvature_factor) < v_target)) { /*curvature required slower speed than target speed -> significant curvature*/

        if (Path_curve_xy_ahead < fabs(RoadSensor[1].Path.CurveXY)) { /*the curvature is getting bigger*/
            Path_curve_xy_ahead = fabs(RoadSensor[1].Path.CurveXY); /*biggest curvature*/
            curvature_max = RoadSensor[1].sRoad; /*position of biggest curvature*/

            if (curvature_start == -1) { /*for detected start of curvature; not in curvature yet*/
                curvature_start = RoadSensor[1].sRoad; /*starting position of curvature ahead*/
            } /*else not important*/

        } /*else not important*/

        curvature_end = RoadSensor[1].sRoad; /*as long as the curvature requires lower speed, curvature_end is recorded*/
        v_calc = sqrt(Car.Tire[FR].muRoad * fabs(1 / Path_curve_xy_ahead) * g) * curvature_factor; /*calculated speed according to the biggest curvature*/

    }

    /*if (fabs(RoadSensor[0].Path.CurveXY) >= (Path_curve_xy_ahead - 0.0001)) { /*check whether Vehicle has passed the maximum curvature*/
        /*curvature_passed = true;
    }
    else {
        curvature_passed = false;
    }*/

}   



/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    /*von hier*/
    /*IPGDriver only controlls the clutch, the rest is taken from the self-manipulated VehicleControl values (see in User_VehicleControl_Calc)*/
    DrivMan.Brake = VehicleControl.Brake;
    DrivMan.BrakePark = VehicleControl.BrakePark;
    /*DrivMan.Clutch = 0;*/
    DrivMan.Gas = VehicleControl.Gas;

    /*bis hier*/

    return 0;
}


/*
** User_VehicleControl_Calc ()
**
** called
** - in RT context
** - after VehicleControl_Calc()
*/

int
User_VehicleControl_Calc(double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
        return 0;

    /*von hier*/

    radar_traffic_n_detected = 0;
    /*test_1 = 0;*/
    /*v_long_radar = 0;*/
    rel_course_angle_traffic = 0;
    /*test = 0;*/
    detected_obj = 0;
    distance_relevant_target = -1;
    speed_relevant_target = -1;
    detected_obj_left = 0;
    distance_obj_left = -1;
    speed_obj_left = -1;
    detected_obj_right = 0;
    distance_obj_right = -1;
    speed_obj_right = -1;
    if (RadarSensor[0].GlobalInf[0].nObj != 0) { /*if radar detects objects ahead*/
        for (int i = 0; i < RadarSensor[0].MaxNumObj; i++) { /*search through complete list*/
            if (RadarSensor[0].ObjList[i].ObjId >= 0) { /*if there is a traffic object (vehicles)*/
                radar_traffic_n_detected = radar_traffic_n_detected + 1;
                
                /*for left lane*/
                if ((RadarSensor[0].ObjList[i].DistY >= (RoadSensor[1].Act.Width / 2)) && (RadarSensor[0].ObjList[i].DistY <= (RoadSensor[1].Act.Width * 1.5)) && (fabs(RadarSensor[0].ObjList[i].RelCourseAngle) <= 0.05) && ((RadarSensor[0].ObjList[i].DynProp >= 2))) {
                    detected_obj_left = 1;
                    distance_obj_left = RadarSensor[0].ObjList[i].Dist;
                    speed_obj_left = RadarSensor[0].ObjList[i].VrelX + Vehicle.v;
                }
                /*for right lane*/
                if ((RadarSensor[0].ObjList[i].DistY <= -(RoadSensor[1].Act.Width / 2)) && (RadarSensor[0].ObjList[i].DistY >= -(RoadSensor[1].Act.Width * 1.5)) && (fabs(RadarSensor[0].ObjList[i].RelCourseAngle) <= 0.05) && ((RadarSensor[0].ObjList[i].DynProp >= 2))) { /*works only for moving objects*/
                    detected_obj_right = 1;
                    distance_obj_right = RadarSensor[0].ObjList[i].Dist;
                    speed_obj_right = RadarSensor[0].ObjList[i].VrelX + Vehicle.v;
                    /*test = detected_obj_right;*/
                }
                /*for current lane*/
                if (fabs(RadarSensor[0].ObjList[i].DistY) <= (RoadSensor[1].Act.Width / 2) && (fabs(RadarSensor[0].ObjList[i].RelCourseAngle) <= 0.05) && ((RadarSensor[0].ObjList[i].DynProp >= 2))) {
                    detected_obj = 1;
                    speed_relevant_target = RadarSensor[0].ObjList[i].VrelX + Vehicle.v;
                    distance_relevant_target = RadarSensor[0].ObjList[i].Dist;
                    /*test_1 = RadarSensor[0].ObjList[i].ObjId;*/
                    if (overtaking == false && relevant_target == -1 && lane_change == 0) { /*do not consider traffic during lane-changes*/
                        relevant_target = RadarSensor[0].ObjList[i].ObjId; /*global Object ID of traffic to overtake*/
                        /*test = 1;
                        test_1 = relevant_target;*/
                    }
                }
            }
        }
    }
    if (RadarSensor[1].GlobalInf[0].nObj != 0) { /*if radar detects objects behind vehicle*/
        for (int i = 0; i < RadarSensor[1].MaxNumObj; i++) { /*search through complete list*/
            if (RadarSensor[1].ObjList[i].ObjId >= 0) { /*if there is a traffic object (vehicles)*/
                if ((RadarSensor[1].ObjList[i].DistY <= -(RoadSensor[0].Act.Width / 2)) && (RadarSensor[1].ObjList[i].DistY >= -(RoadSensor[0].Act.Width * 1.5)) && ((fabs(RadarSensor[1].ObjList[i].RelCourseAngle) - 3.1415) <= 0.05) && ((RadarSensor[1].ObjList[i].DynProp >= 2))) {
                    detected_obj_left = 1;
                    distance_obj_left = RadarSensor[1].ObjList[i].Dist;
                    speed_obj_left = RadarSensor[1].ObjList[i].VrelX + Vehicle.v; /*this is not the real traffic speed*/
                }
                /*if ((RadarSensor[1].ObjList[i].DistY >= (RoadSensor[0].Act.Width / 2)) && (RadarSensor[1].ObjList[i].DistY <= (RoadSensor[0].Act.Width * 1.5)) && (fabs(RadarSensor[1].ObjList[i].RelCourseAngle) - 3.1415 <= 0.05) && ((RadarSensor[1].ObjList[i].DynProp >= 2))) { /*works only for moving objects*/
                    /*detected_obj_right = 1;
                    distance_obj_right = RadarSensor[1].ObjList[i].Dist;
                    speed_obj_right = RadarSensor[1].ObjList[i].VrelX + Vehicle.v; /*this is not the real traffic speed*/
                /*}*/
            }
        }
    }


    if (overtaking == false) { /*if not already in overtaking procedure*/
        if (detected_obj > 0) { /*objects in current lane ahead detected and is slower than our desired speed*/
            if (((detected_obj_left == 0) || ((distance_obj_left >= Vehicle.v) && (speed_obj_left >= Vehicle.v))) && (speed_relevant_target < (v_final - 2)) && (left_lane_type == 0) && (relevant_target != -1)) {
                lane_change = 1;
                overtaking = true;
                
            }
            else { /*overtaking not possible, left lane is not free*/
                brake_for_traffic = 1;
            }
        }
        else { /* no object detected in current lane*/
            brake_for_traffic = 0;
        }
    }
    else {
        for (int i = 0; i < RadarSensor[1].MaxNumObj; i++) { /*search through complete list*/
            if (relevant_target == RadarSensor[1].ObjList[i].ObjId) {/*vehicle just passed traffic*/
                passed = true;
            }
        }
        if ((passed == true) && (right_lane_type == 0) && (((detected_obj_right == 0) || ((distance_obj_right >= Vehicle.v) && (speed_obj_right >= Vehicle.v))))) {
            lane_change = -1;
            relevant_target = -1;
            /*test = 0;*/
            test_1 = 0;
            passed = false;
            overtaking = false;

        }
        if (detected_obj > 0) {
            brake_for_traffic = 1;
        }
        else {
            brake_for_traffic = 0;
        }
    }




    /*detected_obj = ObjByLane[0].Lane[1]->nObjF; /*0 if no relevant target is detected, > 0 if relevant target is detected*/
    /*distance_relevant_target = ObjByLane[0].Lane[1]->ObjFront[0].sMin; /*distance between relevant target's reference point and sensor origin*/
    /*speed_relevant_target = ObjByLane[0].Lane[1]->ObjFront[0].VelLong; /*relative velocity of relevant target compared to ego vehicle*/
 

    /*if (overtaking == false) { /*if not already in overtaking procedure*/
        /*if (detected_obj > 0) { /*objects in current lane ahead detected and is slower than our desired speed*/
            /*relevant_target = ObjByLane[0].Lane[1]->ObjFront[0].ObjID; /*global Object ID of traffic to overtake*/
            /*if (((ObjByLane[0].Lane[0][0].nObjF == 0) /*no objects on left lane in front of vehicle*/
                /*|| ((ObjByLane[0].Lane[0][0].ObjFront[0].sMin >= Vehicle.v /*could be also a fix number like 50 [m]*//*) && (ObjByLane[0].Lane[0][0].ObjFront[0].VelLong >= Vehicle.v))) /*or enough distance (the higher the velocity the more distance is required) and velocity of traffic is higher than vehicle's speed or equal*/
                /*&& (speed_relevant_target < (v_final - 2)) /*traffic in front is slower than it is allowed to be on that lane*/
                /*&& (left_lane_type == 0) /*and left lane is driveable*/
                /*&& ((ObjByLane[0].Lane[0][0].nObjR == 0) /*and no objects on left lane behind the vehicle*/
                    /*|| ((ObjByLane[0].Lane[0][0].ObjRear[0].sMin <= -Vehicle.v /*could be also a fix number like 50 [m]*//*) && (ObjByLane[0].Lane[0][0].ObjRear[0].VelLong <= Vehicle.v)))) { /*or enough distance (the higher the velocity the more distance is required) and velocity of traffic is not higher than vehicle's speed or equal*/

                /*lane_change = 1;
                overtaking = true;
            }
            else { /*overtaking not possible, left lane is not free*/
                /*brake_for_traffic = 1;
            }

        }
        else { /* no object detected in current lane*/
            /*brake_for_traffic = 0;
        }
    }
    else {

        if (relevant_target == ObjByLane[0].Lane[2]->ObjRear[0].ObjID) {/*vehicle just passed traffic*/
            /*passed = true;
        }

        if (passed == true /*vehicle passed the traffic which it had to overtake*/
            /*&& ((ObjByLane[0].Lane[2][0].nObjF == 0) /*no objects on right lane in front of vehicle*/
            /*|| ((ObjByLane[0].Lane[2][0].ObjFront[0].sMin >= Vehicle.v /*could be also a fix number like 50 [m]*//*) && (ObjByLane[0].Lane[2][0].ObjFront[0].VelLong >= Vehicle.v))) /*or enough distance (the higher the velocity the more distance is required) and velocity of traffic is higher than vehicle's speed or equal*/
            /*&& (right_lane_type == 0) /*and right lane is driveable*/
            /*&& ((ObjByLane[0].Lane[2][0].nObjR == 0) /*and no objects on right lane behind the vehicle*/
            /*|| ((ObjByLane[0].Lane[2][0].ObjRear[0].sMin <= -Vehicle.v /*could be also a fix number like 50 [m]*//*) && (ObjByLane[0].Lane[2][0].ObjRear[0].VelLong <= Vehicle.v)))) { /*or enough distance (the higher the velocity the more distance is required) and velocity of traffic is not higher than vehicle's speed or equal*/

            /*lane_change = -1;
            overtaking = false;
            passed = false;
        }
        if (detected_obj > 0) {
            brake_for_traffic = 1;
        }

    }*/
  

    if (RoadSensor[0].Lanes.left->tMidLane >= 0) { /*that means that left lanes are in direction of route*/
        current_lane_type = RoadSensor[0].Lanes.left[RoadSensor[0].Lanes.laneIdx - current_lane].type; /*type of current lane (along the route)*/

        if ((RoadSensor[0].Lanes.laneIdx - current_lane) != 0) { /*if lane is not L0*/
            left_lane_type = RoadSensor[0].Lanes.left[(RoadSensor[0].Lanes.laneIdx - current_lane) - 1].type;
            right_lane_type = RoadSensor[0].Lanes.left[(RoadSensor[0].Lanes.laneIdx - current_lane) + 1].type;
        }
        else { /*if lane is L0*/
            left_lane_type = RoadSensor[0].Lanes.right[RoadSensor[0].Lanes.laneIdx - current_lane].type;
            right_lane_type = RoadSensor[0].Lanes.left[(RoadSensor[0].Lanes.laneIdx - current_lane) + 1].type;
        }

    }
    else { /*that means that left lanes are opposite direction of route*/
        current_lane_type = RoadSensor[0].Lanes.right[RoadSensor[0].Lanes.laneIdx - current_lane].type;

        if (RoadSensor[0].Lanes.laneIdx != 0) { /*if lane is not R0*/
            left_lane_type = RoadSensor[0].Lanes.right[(RoadSensor[0].Lanes.laneIdx - current_lane) - 1].type;
            right_lane_type = RoadSensor[0].Lanes.right[(RoadSensor[0].Lanes.laneIdx - current_lane) + 1].type;
        }
        else { /*if lane is R0*/
            left_lane_type = RoadSensor[0].Lanes.left[RoadSensor[0].Lanes.laneIdx - current_lane].type;
            right_lane_type = RoadSensor[0].Lanes.right[(RoadSensor[0].Lanes.laneIdx - current_lane) + 1].type;
        }
    }



    static int j;
    if (lane_change == 1) { /*lane change to the left, only works for one lane_change, for more changes the width of lanes and the type of them is not considered*/
        
        if (left_lane_type == 0) {
            lane_offset = lane_offset + ((RoadSensor[0].Act.Width + RoadSensor[0].OnLeft.Width) / 2) / (lane_change_time * 1000); /*calculate the offset by using the lane widths*/
            j = j + 1;
        }

        if (j >= (lane_change_time * 1000)) {
            j = 0;
            lane_change = 0;


            if (RoadSensor[0].Lanes.left->tMidLane >= 0) {/*that means that left lanes are in direction of route*/
                if ((RoadSensor[0].Lanes.laneIdx - current_lane) != 0) { /*if lane is not L0*/
                    current_lane = current_lane + 1; /*increase current_lane by 1*/
                }
            }
            else {
                if ((RoadSensor[0].Lanes.laneIdx - current_lane) != 0) { /*if lane is not R0*/
                    current_lane = current_lane + 1; /*increase current_lane by 1*/
                }
            }


        }
        
    }
    else if (lane_change == -1) {/*lane change to the right, only works for one lane_change, for more changes the width of lanes and the type of them is not considered*/
        
        if (right_lane_type == 0) {
            lane_offset = lane_offset - ((RoadSensor[0].Act.Width + RoadSensor[0].OnRight.Width) / 2) / (lane_change_time * 1000); /*calculate the offset by using the lane widths*/
            j = j + 1;
        }
        
        if (j >= (lane_change_time * 1000)) {
            j = 0;
            lane_change = 0;


            if (RoadSensor[0].Lanes.left->tMidLane >= 0) { /*that means that left lanes are in direction of route*/
                if ((RoadSensor[0].Lanes.laneIdx - current_lane) != 0) { /*if lane is not L0*/
                    current_lane = current_lane - 1; /*decrease current_lane by 1*/
                }
            }
            else { /*that means that left lanes are opposite direction of route*/
                if ((RoadSensor[0].Lanes.laneIdx - current_lane) != 0) { /*if lane is not R0*/
                    current_lane = current_lane - 1; /*decrease current_lane by 1*/
                }
            }


        }
    }

    /*the lane width might change, so the lane_offset has to be checked and updated if necessary*/
    /*if ((lane_change == 0) && ((lane_offset >= fabs(0.05)) && ((lane_offset != ((RoadSensor[0].Act.Width + RoadSensor[0].OnLeft.Width) / 2)) || (lane_offset != ((RoadSensor[0].Act.Width + RoadSensor[0].OnRight.Width) / 2))))) {
        if (lane_offset > 0) {
            lane_offset = ((RoadSensor[0].Act.Width + RoadSensor[0].OnLeft.Width) / 2);
        }
        if (lane_offset < 0) {
            lane_offset = -((RoadSensor[0].Act.Width + RoadSensor[0].OnRight.Width) / 2);
        }
    }*/




    if ((Deviation_distance_path != lane_offset /*0*/) || (Deviation_angle_path != 0)) {
        VehicleControl.Steering.Ang = VehicleControl.Steering.Ang - pid_ctrl_steer( Deviation_angle_path) * 10 - pid_ctrl_steer(Deviation_distance_path - lane_offset); /*10 & 1 */
    }
  


    if (v_calc != -1) {
        v_final = v_calc;
    }
    else {
        v_final = v_target;
    }

    if (brake_for_traffic == 1) {
        if (speed_relevant_target <= v_final) { /*vehicle should not be faster than speed limit or calculated*/
            v_final = speed_relevant_target;
        }
        curvature_max = distance_relevant_target + Vehicle.sRoad;
    }

    if (Vehicle.v <= (v_final + 1)) { /*current velocity is lower than calculated. Deviation of 1 m/s allowed*/

        if (Vehicle.v <= (v_final - 2)) { /*full throttle until vehicle is just 2 m/s underneath calculated velocity*/
            
            VehicleControl.Gas = VehicleControl.Gas + 0.0002; /*vehicles Gas raises, until it reaches full throttle*/
            if (VehicleControl.Gas >= 1) {
                VehicleControl.Gas = 1;
            }
            
        }

        else { /*vehicle's velocity is close to calculated velocity. A PI-Constroller is used*/
            v_dif = v_final - Vehicle.v;
            VehicleControl.Gas = fabs(pid_ctrl(v_dif));
            if (VehicleControl.Gas > 1) {
                VehicleControl.Gas = 1;
            }
            
        }

        VehicleControl.Brake = 0.0;
    }

    else if (Vehicle.v > (v_final + 2)) { /*current velocity is higher than calculated. Deviation of 2 m/s allowed*/
        
        if (Vehicle.v > (v_final + 3)) { /*full brake until vehicle is just 3 m/s above calculated velocity*/
            
            VehicleControl.Brake = (Vehicle.v - v_final) / (curvature_max - Vehicle.sRoad);

            /*VehicleControl.Brake = VehicleControl.Brake + 0.0006; /*vehicles Brake raises, until it reaches full brake; faster than Gas*/
            if (VehicleControl.Brake >= 1) {
                VehicleControl.Brake = 1;
            }
        } else if (curvature_max != -1) { /*curvature ahead or in curvature already*/
                v_dif = Vehicle.v - v_final;
                VehicleControl.Brake = fabs(pid_ctrl(v_dif));
                if (VehicleControl.Brake > 1) {
                    VehicleControl.Brake = 1;
                }
            }
            else { /*no curvature ahead and not in curvature already*/
                v_dif = Vehicle.v - v_final;
                VehicleControl.Brake = fabs(pid_ctrl(v_dif));
                if (VehicleControl.Brake > 1) {
                    VehicleControl.Brake = 1;
                }
            }
        VehicleControl.Gas = 0.0;
        
    }

    if ((curvature_end > 0) && (Vehicle.sRoad >= curvature_end) /*&& (curvature_passed == true)*/) { /*reset curvature positions, after vehicle passes curvature_end position*/
        curvature_start = -1;
        curvature_max = -1;
        curvature_end = -1;
        Path_curve_xy_ahead = 0;
        /*curvature_passed = false;*/
    }


    /*bis hier*/

    return 0;
}



/*
** User_Brake_Calc ()
**
** called
** - in RT context
** - after Brake_Calc() in Vhcl_Calc()
*/

int
User_Brake_Calc (double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/

    return 0;
}



/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{
#if !defined(LABCAR)
    RBS_OutMap(CycleNo);
#endif

    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }

#endif
    return -1;
}



/*
** User_ApoMsg_Send ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - near the end of the main loop, in MainThread_FinishCycle()
** - pay attention to realtime condition
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just before exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{
}

