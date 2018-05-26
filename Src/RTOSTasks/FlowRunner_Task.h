/*
 * Flow_Runner_Task.h
 *
 *  Created on: Mar 9, 2017
 *      Author: Ted
 */

#ifndef RTOSTASKS_FLOWRUNNER_TASK_H_
#define RTOSTASKS_FLOWRUNNER_TASK_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

#define MAX_PARAM 6
#define MAX_COMMANDS 4500

#define NUMERIC(a) ((a) >= '0' && (a) <= '9')
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-' || (a) == '+')
#define NUMERIC_DECIMAL_SIGNED(a) (NUMERIC_SIGNED(a) || (a) == '.')

enum COMMAND_STATUS {
	CMD_STATUS_Queued,
	CMD_STATUS_Running,
	CMD_STATUS_Finished,
	CMD_STATUS_Invalid,                    //command fell into a default case so it is not valid
	CMD_STATUS_Timeout,
	CMD_STATUS_FullStroke,
	CMD_STATUS_Aborting,
	CMD_STATUS_Error
};

typedef struct{
	char letter;
	uint8_t number;
	int32_t parameters[MAX_PARAM];
	uint8_t param_count;
	enum COMMAND_STATUS cmd_status;
}flow_command_t;


void FlowRunner_TaskStart();
void FlowRunner_StartRun();
uint8_t FlowRunner_IsRunning();
void FlowRunner_AbortRun();
uint8_t FlowRunner_IsAborting();
uint8_t FlowRunner_Builder(uint8_t *pCOMMAND_TO_PROCESS);
uint16_t FlowRunner_ResetQueue();
uint8_t FlowRunner_BuildQueue(uint8_t *pCOMMAND_TO_PROCESS);
uint8_t FlowRunner_ProcessCommand(const uint8_t *pCOMMAND_TO_PROCESS, flow_command_t *processed_cmd);
void FlowRunner_Execute(flow_command_t *command_to_execute);
uint16_t FlowRunner_GetCurrentStep();
void FlowRunner_ResetAllHardwareToDefault();

//FLOW COMMAND SYNTAX
//command letter - char
//command number - uint8_t
//command parameter count - uint8_t
//command parameter array 6 elements - int32

//MAX COMMAND SIZE is 256 characters
//MAX NUMBER OF PARAMETERS is MAX_PARAM  = 6 currently
//FIRST CHAR MUST BE COMMAND LETTER
//SECOND CHAR MUST BE COMMAND NUMBER
//COMMAND MUST END WITH '*'
//Blank spaces are counted for parameter separators.
//Parameters must be of type int32.
//Could use the first 18 prime numbers to encode actuator numbers in a few parameters parameter and then factorize to get each actuator
// 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61
//A0 1 2 3 4 5 6*     - SET actuators 1,2,3,4,5,6 to LOW
//A1 1 2 5*		      - SET actuators 1,2,5 to HIGH
//A2*				  - SET all actuators (1 to 18) to LOW
//A3*				  - SET all actuators (1 to 18) to HIGH
//A4 p1 p2 p3 p4 p5 p6* - Set actuators p1, p2... to LOW and block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit - 2x2 = 4 sec.
//A5 p1 p2 p3 p4 p5 p6* - Set actuators p1, p2... to HIGH and block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit - 2x2 = 4 sec.
//A6*					- SET all actuators (1 to 18) to LOW and block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit - 2x2 = 4 sec.
//A7*					- SET all actuators (1 to 18) to HIGH and block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit - 2x2 = 4 sec.
//A8 p1 p2 p3 p4 p5 p6*	- "Lance" a blister. Extend (with block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit). Than Retract again block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit.
//D0 30000*		      - DELAY in ms
//C0 3500 3700 1*     - Calibrate all sensors between 3500 and 3700 raw. param3 (0 - VOLATILE, 1 - NON_VOLATILE)
//C1 lowVal highVal 1 sensorNum*   - Calibrate one sensorNum between 3500 and 3700 raw  (0 - VOLATILE, 1 - NON_VOLATILE)
//W0 p1*			    - wait for all prior commands to complete with a timeout
//W1 p1 p2 p3 p4 p5 p6* - wait for command at (p2 and p3 and p4 and p5 and p6) to complete with a timeout of p1
//W2 p1 p2 p3 p4 p5 p6* - wait for command at (p2 or p3 or p4 or p5 or p6) to complete with a timeout of p1
//H0 1 2 3 4*		  - Turn heater # off
//H1 7000*		      - Turn heater #1 on with a setpoint of 7000/100 = 70.00 C
//H2 7000*			  - Turn heater #2 on with a setpoint of 7000/100 = 70.00 C
//H3 7000*			  - Turn heater #3 on with a setpoint of 7000/100 = 70.00 C
//H4 7000*			  - Turn heater #4 on with a setpoint of 7000/100 = 70.00 C
//H5 minval maxval timeout* - Block until Heater 1's reading is between min and max value. Min/Max value are/ 100. timeout is in milliseconds
//H6 minval maxval timeout* - Block until Heater 2's reading is between min and max value. Min/Max value are/ 100. timeout is in milliseconds.
//H7 minval maxval timeout* - Block until Heater 3's reading is between min and max value. Min/Max value are/ 100. timeout is in milliseconds.
//H8 minval maxval timeout* - Block until Heater 4's reading is between min and max value. Min/Max value are/ 100. timeout is in milliseconds.
//H9 lowTemp highTemp lowDwell highDwell CycleNums useFan*  - PCR
//I0*				- Take an image
//I1*				- Take preTMB image
//I2*				- Take result Image
//T0 seconds*		- time remaining in seconds
//M0*				- Mixer off - No stir bar alignment. Will execute within microseconds.
//M1* 				- Mixer off - With stir bar alignment. Blocking - will take 1-3 seconds to align stir bar. Timeout is set at approx 8sec.
//M2 P1*			- Mixer off - With stir bar alignment P1 checks if a minimum number of revolutions have been completed. If not, an error is generated. Blocking - will take 1-3 seconds to align stir bar.
//M3 duty*			- Mixer On at duty cycle. Duty cycle values are 0 to 999. Will execute within microseconds.
//M4 p1 p2 p3*		- NOT implemented yet. Where P1 is run until x number of revolutions are achieved, p2 is timeout period, p3 is duty cycle. Non-Blocking. With stir bar alignment.
//M5 p1 p2 p3*		- NOT implemented yet. Where P1 is run for x amount of time, P2 is a minimum number of revolutions, P3 is duty cycle. Non-Blocking. With stir bar alignment.
//F0*				- Fan Off
//F1*				- Fan ON
//P1 kP kI kD* //set PID values for Heater 1. Heater Number = 0 to 3, kP /100, kI and kD are /1000, so 1/1000 would be 0.001
//P2 kP kI kD* //set PID values for Heater 1. Heater Number = 0 to 3, kP /100, kI and kD are /1000, so 1/1000 would be 0.001
//P3 kP kI kD* //set PID values for Heater 1. Heater Number = 0 to 3, kP /100, kI and kD are /1000, so 1/1000 would be 0.001
//P4 kP kI kD* //set PID values for Heater 1. Heater Number = 0 to 3, kP /100, kI and kD are /1000, so 1/1000 would be 0.001
//S0 Mnum runRate*	- Stepper Home a single motor *Mnum are 1,2,3,4,5,6,7,8 at runRate;
//S1 Mnum travelDistance runRate* - Stepper motor move a single motor at runRate
//S2 Mnum travelDistance runRate stopOnFD DeltaVoltage keepgoing*   STEPPER MOTOR MOVE (relative from current position
                                                                                // *runRate - steps per second
                                                                                // *travelDistance - steps. Can be down (+) or up (-).
                                                                                // *stopOnFD - stop on (fluid detector).
                                                                                // 1,2,3,4,5,6,7,8,9,10,11 - are the FDs
                                                                                // *DeltaVoltage - this is the amount of voltage change of the FD. This is a raw number - 0 to 4095 and can be + or -
																				// continue - (0 or 1), 0 will generate CMD_STATUS_Error if travelDistance is obtained, 1 will set command status to CMD_STATUS_FullStroke

//S3 Mnum travelDistance runRate stopOnFD1 stopOnFD2 deltaVoltage*				// runs until travelDistance is obtained or (stopOnFD1 and stopOnFD2) are triggered.
//S8 travelDist1 travelDist2 travelDist3 travelDist4 travelDist5 travelDist6    // drive motors 1 to 6 to individual travelDist at a specific step rate
//S9 runRate* Home all stepper motors at runRate

#endif /* RTOSTASKS_FLOWRUNNER_TASK_H_ */
