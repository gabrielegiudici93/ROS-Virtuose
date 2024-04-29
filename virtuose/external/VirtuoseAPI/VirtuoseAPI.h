/*
 * VirtuoseAPI.h - Virtuose Application Programming Interface
 *
 * Copyright (C) 2001-2020, HAPTION S.A.
 * Author: Pascal Louveau
 * Version number: 4.03
 * Revision date: 25/02/2020
 */


/**
* \authors Pascal LOUVEAU
* \authors Jerome PERRET
* 
* \brief This file contains the prototypes of the functions composing the virtuose API. 
* 
* \details The goal of this API is to allow the interfacing between a client application and a haptic device.\n 
*/

#if !defined(_VirtuoseAPI_h)
#define _VirtuoseAPI_h


#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
 
#include <stdio.h>


typedef void *VirtContext;
typedef void (*VirtPeriodicFunction)(VirtContext, void *);


typedef enum
{
	/* Default command type: no movement possible */
    COMMAND_TYPE_NONE = 0,
    /* Force/position control */
    COMMAND_TYPE_IMPEDANCE = 3,
    /* Position/force control with virtual kinematics */
    COMMAND_TYPE_VIRTMECH = 5,
	/* articular position control */
	COMMAND_TYPE_ARTICULAR = 6,
	/* articular force control */
	COMMAND_TYPE_ARTICULAR_IMPEDANCE = 7,
}
VirtCommandType;


typedef enum
{
    /* Default command type: no movement possible */
    GRIPPER_COMMAND_TYPE_NONE = 0,
    /* Position/position control */
    GRIPPER_COMMAND_TYPE_POSITION,
    /* Force/position control */
    GRIPPER_COMMAND_TYPE_IMPEDANCE,
}
VirtGripperCommandType;

  
typedef enum
{
    /* Indexing is allowed on translations and rotations */
    INDEXING_ALL = 0,
    /* Indexing is allowed on translations only */
    INDEXING_TRANS = 1,
    /* No indexing allowed, even without deadman  */
    INDEXING_NONE = 2,
    /* Indexing is allowed on translations and rotations, no force while indexing */
    INDEXING_ALL_FORCE_FEEDBACK_INHIBITION = 3,
    /* Indexing is allowed on translations only, no force while indexing */
    INDEXING_TRANS_FORCE_FEEDBACK_INHIBITION = 4,
    /* Indexing is allowed on rotations only, no force while indexing */
    INDEXING_ROT_FORCE_FEEDBACK_INHIBITION = 6,
    /* Indexing is allowed on rotations only */
    INDEXING_ROT = 7,
}
VirtIndexingType;
 
 
typedef enum 
{ 
    VM_TYPE_CartMotion = 0, 
    VM_TYPE_Spline, 
    VM_TYPE_Rx, 
    VM_TYPE_Tx,
    VM_TYPE_TxRx, 
    VM_TYPE_TxTyTz, 
    VM_TYPE_RxRyRz,
    VM_TYPE_TxTy,
    VM_TYPE_Crank,
} 
VirtVmType; 



/**
* virtActiveRotationSpeedControl : activate the speed control in rotation
* \param VC Virtuose context
* \param angle Angle of the cone (rad)
* \param speedFactor Scaling of the rotation speed
* \return Error code
*/
extern int virtActiveRotationSpeedControl(VirtContext VC, float angle, float speedFactor);

/**
* virtActiveSpeedControl : activate hybrid position/speed control in translation.
* \param VC Virtuose context
* \param radius Radius of the sphere (m)
* \param speedFactor Scaling of the speed
* \return Error code
*/
extern int virtActiveSpeedControl(VirtContext VC, float radius, float speedFactor);

extern int virtAddForce(VirtContext VC, float *force);

/**
* virtAPIVersion : read the Virtuose library version
* \param major pointer to an integer. It corresponds to the major index of the software version. 
* \param minor pointer to an integer. It corresponds to the minor index of the software version.
* \return Error code
*/
extern int virtAPIVersion(int *major, int *minor);


extern int virtAttachQSVO(VirtContext VC, float *Ks, float *Bs);
extern int virtAttachVO(VirtContext VC, float mass, float *mxmymz);
extern int virtAttachVOAvatar(VirtContext VC, float mass, float *mxmymz);

/**
* virtClose : close a connexion to the device
* \param VC Virtuose context
* \return Error code
*/
extern int virtClose(VirtContext VC);

#define virtConvertDisplToTransformationMatrix	virtConvertDeplToHomogeneMatrix
extern int virtConvertDeplToHomogeneMatrix(VirtContext VC, float* d, float* m);

#define virtConvertTransformationMatrixToDispl	virtConvertHomogeneMatrixToDepl
extern int virtConvertHomogeneMatrixToDepl(VirtContext VC, float* d, float* m);

extern int virtConvertRGBToGrayscale (VirtContext VC, float *rgb, float *gray);
extern int virtDeactiveRotationSpeedControl(VirtContext VC);
extern int virtDeactiveSpeedControl(VirtContext VC);
extern int virtDetachVO(VirtContext VC);
extern int virtDetachVOAvatar(VirtContext VC);
extern int virtDisableControlConnexion(VirtContext VC, int disable);
extern int virtDisplayHardwareStatus(VirtContext VC, FILE *fichier);
extern int virtEnableForceFeedback(VirtContext VC, int enable);
extern int virtForceShiftButton(VirtContext VC, int forceShiftButton);
extern int virtGetADC (VirtContext VC, int line, float* adc);
extern int virtGetAlarm(VirtContext VC, unsigned int *alarm);
extern int virtGetAnalogicInputs (VirtContext VC, float* inputs);
extern int virtGetArticularPosition(VirtContext VC, float *pos);
extern int virtGetArticularPositionOfAdditionalAxe(VirtContext VC, float *pos);
extern int virtGetArticularSpeed(VirtContext VC, float *speed);
extern int virtGetArticularSpeedOfAdditionalAxe(VirtContext VC, float *speed);
extern int virtGetAvatarPosition (VirtContext VC, float *pos);
extern int virtGetAxisOfRotation(VirtContext VC, float* axe);
extern int virtGetBaseFrame(VirtContext VC, float *base);
extern int virtGetButton(VirtContext VC, int button_number, int *state);
extern int virtGetCatchFrame(VirtContext VC, float *frame);
extern int virtGetCenterSphere(VirtContext VC, float* pos);
extern int virtGetCommandType(VirtContext VC, VirtCommandType *type);
extern int virtGetControlerVersion(VirtContext VC, int *major, int *minor);
extern int virtGetDeadMan(VirtContext VC, int *dead_man);
extern int virtGetDeviceID(VirtContext VC, int *device_type, int *serial_number);
extern int virtGetEmergencyStop(VirtContext VC, int *emergency_stop);
extern int virtGetError(VirtContext VC,int *error);
extern int virtGetErrorCode(VirtContext VC);
extern char *virtGetErrorMessage(int code);
extern int virtGetFailure(VirtContext VC, unsigned int *error);
extern int virtGetForce(VirtContext VC, float *force);
extern int virtGetForceFactor(VirtContext VC, float *force_factor);
extern int virtGetIndexingMode(VirtContext VC, VirtIndexingType *indexing_mode);
extern int virtGetMouseState(VirtContext VC, int* actif, int* clic_gauche, int* clic_droit);
extern int virtGetObservationFrame(VirtContext VC, float *obs);
extern int virtGetPhysicalPosition (VirtContext VC, float *pos);
extern int virtGetPhysicalSpeed(VirtContext VC, float *speed);
extern int virtGetPosition(VirtContext VC, float *pos);
extern int virtGetPowerOn(VirtContext VC, int *power);
extern int virtGetSpeed(VirtContext VC, float *speed);
extern int virtGetSpeedFactor(VirtContext VC, float *speed_factor);
extern int virtGetTimeLastUpdate (VirtContext VC, unsigned int *time);
extern int virtGetTimeoutValue(VirtContext VC,float *time_value);
extern int virtGetTimeStep(VirtContext VC, float* step);

#define virtGetDigitalInputs virtGetTorInputs
extern int virtGetTorInputs(VirtContext VC, unsigned int *tor_in);

extern int virtGetTrackball(VirtContext VC, int* x_move, int* y_move);
extern int virtGetTrackballButton(VirtContext VC, int* actif, int* btn_gauche, int* btn_milieu, int* btn_droit);
extern int virtIsInBounds(VirtContext VC, unsigned int *bounds);
extern int virtIsInShiftPosition(VirtContext VC, int* decalage);
extern int virtIsInSpeedControl(VirtContext VC, int *translation, int *rotation);
extern VirtContext virtOpen(const char *nom);
extern int virtOutputsSetting(VirtContext VC, unsigned int outputs);
extern int virtSaturateTorque (VirtContext VC, float forceThreshold, float momentThreshold);
extern int virtSetArticularForce(VirtContext VC, float *force);
extern int virtSetArticularForceOfAdditionalAxe(VirtContext VC, float *effort);
extern int virtSetArticularPosition(VirtContext VC, float *pos);
extern int virtSetArticularPositionOfAdditionalAxe(VirtContext VC, float *pos);
extern int virtSetArticularSpeed(VirtContext VC, float *speed);
extern int virtSetArticularSpeedOfAdditionalAxe(VirtContext VC, float *speed);
extern int virtSetBaseFrame(VirtContext VC, float *base);
extern int virtSetCatchFrame(VirtContext VC, float *frame);
extern int virtSetCommandType(VirtContext VC, VirtCommandType type);
extern int virtSetDebugFlags(VirtContext VC, unsigned short flag);
extern int virtSetForce(VirtContext VC, float *force);
extern int virtSetForceFactor(VirtContext VC, float force_factor);
extern int virtSetForceInSpeedControl(VirtContext VC, float force);
extern int virtSetGripperCommandType(VirtContext VC, VirtGripperCommandType type);
extern int virtSetIndexingMode(VirtContext VC, VirtIndexingType indexing_mode);
extern int virtSetObservationFrame(VirtContext VC, float *obs);
extern int virtSetObservationFrameSpeed(VirtContext VC, float *speed);
extern int virtSetOutputFile(VirtContext VC, char *name);
extern int virtSetPeriodicFunction(VirtContext VC, void (*fn)(VirtContext, void *), float *period, void *arg);
extern int virtSetPosition(VirtContext VC, float *pos);
extern int virtSetPowerOn(VirtContext VC, int power);
extern int virtSetPwmOutput(VirtContext VC, float *pwm);
extern int virtSetSpeed(VirtContext VC, float *speed);
extern int virtSetSpeedFactor(VirtContext VC, float speed_factor);
extern int virtSetTexture(VirtContext VC, float *position,float *intensite, int reinit);
extern int virtSetTextureForce(VirtContext VC, float *texture_force);
extern int virtSetTimeoutValue(VirtContext VC,float time_value);
extern int virtSetTimeStep(VirtContext VC, float step);
extern int virtSetTorqueInSpeedControl(VirtContext VC, float torque);
extern int virtStartLoop (VirtContext VC);
extern int virtStopLoop (VirtContext VC);
extern int virtTrajRecordStart (VirtContext VC);
extern int virtTrajRecordStop (VirtContext VC);
extern int virtTrajSetSamplingTimeStep (VirtContext VC, float timeStep, unsigned int *recordTime);
extern int virtVmActivate (VirtContext VC);
extern int virtVmDeactivate (VirtContext VC);
extern int virtVmDeleteSpline(VirtContext VC, char *file_name);
extern int virtVmGetBaseFrame (VirtContext VC, float *base);
extern int virtVmGetTrajSamples (VirtContext VC, float* samples);
extern int virtVmLoadSpline(VirtContext VC, char *file_name);
extern int virtVmSaveCurrentSpline(VirtContext VC, char *file_name);
extern int virtVmSetBaseFrame (VirtContext VC, float *base);
extern int virtVmSetBaseFrameToCurrentFrame (VirtContext VC);
extern int virtVmSetDefaultToCartesianPosition (VirtContext VC);
extern int virtVmSetDefaultToTransparentMode (VirtContext VC);
extern int virtVmSetRobotMode(VirtContext VC, int OnOff);
extern int virtVmSetType (VirtContext VC, VirtVmType type);
extern int virtVmStartTrajSampling (VirtContext VC, unsigned int nbSamples);
extern int virtVmWaitUpperBound(VirtContext VC);
extern int virtWaitPressButton(VirtContext VC, int button_number);
extern int virtGetNbAxes(VirtContext VC, int* nbAxes);
extern int virtSetFingerTipSpeed(VirtContext VC, float *speed);
extern int virtGetSimulationStiffness(VirtContext VC, float *stiffness);
extern int virtGetSimulationDamping(VirtContext VC, float *damping);


#define DEBUG_SERVO	0x0001
#define DEBUG_LOOP	0x0002


#define VIRT_E_NO_ERROR					0
#define VIRT_E_OUT_OF_MEMORY			1
#define VIRT_E_COMMUNICATION_FAILURE	2
#define VIRT_E_INVALID_CONTEXT			3
#define VIRT_E_FILE_NOT_FOUND			4
#define VIRT_E_WRONG_FORMAT				5
#define VIRT_E_TIME_OUT					6
#define VIRT_E_NOT_IMPLEMENTED			7
#define VIRT_E_VARIABLE_NOT_AVAILABLE	8
#define VIRT_E_INCORRECT_VALUE			9
#define VIRT_E_SYNTAX_ERROR				10
#define VIRT_E_HARDWARE_ERROR			11
#define VIRT_E_POSITION_DISCONTINUITY	12
#define VIRT_E_VIRTUOSE_DLL_NOT_FOUND	13
#define VIRT_E_PERIODIC_FUNCTION		14
#define VIRT_E_PERFORMANCE_COUNTER		15
#define VIRT_E_MAJOR_MINOR_VERSION		16
#define VIRT_E_WRONG_MODE				17
#define VIRT_E_MODE_NOT_SUPPORTED		18 
#define VIRT_E_CALL_TIME				19 
#define VIRT_E_INCOMPATIBLE_VERSION		20 
#define VIRT_E_INCORRECT_VM_TYPE		21 
#define VIRT_E_ALREADY_CALLED			22 
 
#define VIRT_BOUND_LEFT_AXE_1			0x0001 
#define VIRT_BOUND_RIGHT_AXE_1			0x0002 
#define VIRT_BOUND_SUP_AXE_2			0x0004 
#define VIRT_BOUND_INF_AXE_2			0x0008 
#define VIRT_BOUND_SUP_AXE_3			0x0010 
#define VIRT_BOUND_INF_AXE_3			0x0020 
#define VIRT_BOUND_RIGHT_AXE_4			0x0040 
#define VIRT_BOUND_LEFT_AXE_4			0x0080 
#define VIRT_BOUND_SUP_AXE_5			0x0100 
#define VIRT_BOUND_INF_AXE_5			0x0200 
#define VIRT_BOUND_LEFT_AXE_6			0x0400 
#define VIRT_BOUND_RIGHT_AXE_6			0x0800 
#define VIRT_BOUND_INF_AXE_2_3			0x1000 
#define VIRT_BOUND_SUP_AXE_2_3			0x2000 
 
 
#define	VIRT_ALARM_OVERHEAT				0x00004000 
#define VIRT_ALARM_SATURATE				0x00008000
#define VIRT_ALARM_CALLBACK_OVERRUN		0x00020000 
#define VIRT_ALARM_ERROR_POSITION		0x00040000 
#define VIRT_ALARM_LOW_BATTERY			0x00100000
 

#define VIRT_BREAKDOWN_MOTOR_1			0x0001
#define VIRT_BREAKDOWN_MOTOR_2			0x0002
#define VIRT_BREAKDOWN_MOTOR_3			0x0004
#define VIRT_BREAKDOWN_MOTOR_4			0x0008
#define VIRT_BREAKDOWN_MOTOR_5			0x0010
#define VIRT_BREAKDOWN_MOTOR_6			0x0020
#define VIRT_BREAKDOWN_MOTOR_7			0x0040
#define VIRT_BREAKDOWN_MOTOR_8			0x0080


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* _VirtuoseAPI_h */
