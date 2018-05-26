/*
 * Flow_Runner_Task.c
 *
 *  Created on: Mar 9, 2017
 *      Author: Ted
 */

#include <FlowRunner_Task.h>

#include "HEATER_CTRL_Task.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "Fan_Control.h"
#include "Mixer_Control.h"
#include "L6470.h"
#include "fluid_sensor.h"

#include "USB_Task.h"

//private variables
static osThreadId FlowRunner_TaskHandle;
static EventGroupHandle_t xFlowRunner_EventGroup;

static flow_command_t Flow_command_queue[MAX_COMMANDS];
static uint16_t Flow_commands_total = 0;
static uint16_t Flow_command_current = 0;

#define FRF_Enable (1<<0)    //bit 0          FRF = Flow Run Flag
#define FRF_Abort  (1<<1)    //bit 1

//private functions
static void FlowRunner(void const * argument);
static uint8_t FlowRunner_WaitForAbort(uint32_t timeToWait); //used for blocking the FlowRunner task with the option of aborting

static void FlowRunner_StopMixerWithStirBarAlignment(flow_command_t *command_to_execute); // helper function used to stop the mixing motor while aligning the stir bars
static void FlowRunner_WaitForSixStepperMovesToComplete(flow_command_t *command_to_execute); //helper function used to wait for stepper motors 0 to 5 to clear the busy flag
static enum COMMAND_STATUS FlowRunner_WaitForOneStepperMoveToComplete(uint8_t motorNum);  //helper function waits for 1 stepper motor to clear the busy flag

static void FlowRunner_ExtendActuatorsAndWaitForLimitSW(flow_command_t *command_to_execute);
static void FlowRunner_RetractActuatorsAndWaitForLimitSW(flow_command_t *command_to_execute);
static void FlowRunner_CheckIfAbortIsRequired(flow_command_t *command_to_execute);


static uint8_t FlowRunner_WaitForHeaterToBeInRange(uint8_t heatNum, float lowVal, float highVal, uint32_t timeout);

static void FlowRunner_SensorCalibration(uint8_t FS_num, flow_command_t *command_to_execute);

void FlowRunner_TaskStart()
{
	  /* definition and creation of USBControl_Task */
	  osThreadDef(FlowRunner_Task, FlowRunner, osPriorityNormal, 0, 256);
	  FlowRunner_TaskHandle = osThreadCreate(osThread(FlowRunner_Task), NULL);

	  /* Create the event group. */
	  xFlowRunner_EventGroup = xEventGroupCreate();
	  /*	Event group map
	   *	BIT_0 = enable flow run
	   *	BIT_1 = abort flow run
	   *	BIT_2 =
	   *	BIT_3 =
	   *	BIT_4 =
	   *	BIT_5 =
	   *	BIT_6 =
	   *	BIT_7 = reset
	   */
}

/*
 * Takes a command string, runs FlowRunner_ProcessCommand on it to convert it to flow_command_t.
 * If conversion is successful, it gets queued on the next command_queue slot.
 * Returns 0 for fail and 1 for success.
 */
uint8_t FlowRunner_BuildQueue(uint8_t *pCOMMAND_TO_PROCESS)
{
	if (MAX_COMMANDS <= Flow_commands_total) return 0;

	flow_command_t processed_cmd;
	if (FlowRunner_ProcessCommand(pCOMMAND_TO_PROCESS, &processed_cmd))
	{
		Flow_command_queue[Flow_commands_total] = processed_cmd;
		Flow_commands_total++;
		return 1;
	}

	return 0;
}

/*
 * Reset command_queue total to 0
 * Returns the number of commands prior to resetting
 */
uint16_t FlowRunner_ResetQueue()
{
	uint16_t temp = Flow_commands_total;
	Flow_commands_total = 0;
	return temp;
}


/*
 * Abort the current FlowRun. Does nothing if not currently running.
 */
void FlowRunner_AbortRun()
{
	if (FlowRunner_IsRunning())
	{
		xEventGroupSetBits(xFlowRunner_EventGroup, FRF_Abort);
	}
}

/*
 * Checks if Abort is flag is set or not.
 */
uint8_t FlowRunner_IsAborting()
{
	EventBits_t uxBits = xEventGroupGetBits(xFlowRunner_EventGroup);   //read the latest set of bits
	return (uxBits & FRF_Abort);
}

/*
 * Blocks the task with the option of aborting. timeToWait is in milliseconds
 */
static uint8_t FlowRunner_WaitForAbort(uint32_t timeToWait)
{
	EventBits_t xbits = xEventGroupWaitBits(xFlowRunner_EventGroup, FRF_Abort, pdFALSE, pdFALSE, pdMS_TO_TICKS(timeToWait));
	return (xbits & FRF_Abort);
}


/*
 * Start the FlowRunner
 */
void FlowRunner_StartRun()
{
	Flow_command_current = 0;
	xEventGroupClearBits(xFlowRunner_EventGroup, FRF_Abort);
	xEventGroupSetBits(xFlowRunner_EventGroup, FRF_Enable);
}

/*
 * Check if Flow is Running
 */
uint8_t FlowRunner_IsRunning()
{
	EventBits_t uxBits = xEventGroupGetBits(xFlowRunner_EventGroup);
	return (uxBits & FRF_Enable);
}

/*
 * This is the Flow Runner
 */
static void FlowRunner(void const * argument)
{
	  EventBits_t uxBits;

	  /* Infinite loop */
	  for(;;)
	  {
		  uxBits = xEventGroupWaitBits(xFlowRunner_EventGroup, FRF_Enable, pdFALSE, pdFALSE, portMAX_DELAY);  //Wait indefinitely for Bit_0 to become True.

		  if ( (uxBits & FRF_Enable) != 0 )  //if bit0 is not 0  (is set). Not really needed, but just in case
		  {
			  while (Flow_command_current < Flow_commands_total)
			  	  {
				  FlowRunner_Execute(&Flow_command_queue[Flow_command_current]);

				  uxBits = xEventGroupGetBits(xFlowRunner_EventGroup);   //read the latest set of bits
				  if (uxBits & FRF_Abort) break; //if the abort bit is set

				  Flow_command_current++;
			  	  }

			  if (uxBits & FRF_Abort)  //if the sequence was aborted, clear the abort flag
			  	  {
				  xEventGroupClearBits(xFlowRunner_EventGroup, FRF_Abort);

				  //do other things... run the reset flow?
				  FlowRunner_ResetAllHardwareToDefault();
			  	  }

			  xEventGroupClearBits(xFlowRunner_EventGroup, FRF_Enable);
		  }
	  }
}

/*
 * Executes a reset flow bringing all hardware to its default state
 */
void FlowRunner_ResetAllHardwareToDefault()
{
	HEATER_CTRL_HeaterOFF(0);
	HEATER_CTRL_HeaterOFF(1);
	HEATER_CTRL_HeaterOFF(2);
	HEATER_CTRL_HeaterOFF(3);

	Fan_Control_Off();

	Mixer_Control_MixerOff();

	int32_t list_of_actuators[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
    ACTUATOR_RETRACT(list_of_actuators, 18);

    //bring stepper motors back home
    uint32_t runRate			= 1000;

    for (uint8_t i = 0; i < 6; i++)
    	{	//if the home flag is not set, move the motor toward home - enough steps to get there
    	//if (!L6470DrvMtx[i].Command->CheckStatusRegisterFlag(i, SW_F_ID))
    		L6470DrvMtx[i].Command->SetParam(i, L6470_MAX_SPEED_ID, Step_s_2_MaxSpeed(runRate));
    		L6470DrvMtx[i].Command->Move(i, L6470_DIR_FWD_ID, 120000);
    	}

    flow_command_t command_to_execute;
    //wait for all 6 motor moves to complete
    FlowRunner_WaitForSixStepperMovesToComplete(&command_to_execute);
}


/*
 * Returns the current step being executed
 */
uint16_t FlowRunner_GetCurrentStep()
{
	return Flow_command_current;
}


/*
 *Takes a command string and converts it into a flow_command_t.
 */
uint8_t FlowRunner_ProcessCommand(const uint8_t *pCOMMAND_TO_PROCESS, flow_command_t *processed_cmd)
{
	//check if pointer is null or empty
	if ((pCOMMAND_TO_PROCESS == NULL) || (pCOMMAND_TO_PROCESS[0] == '\0')) return 0;

	uint8_t command_length = strlen((char *)pCOMMAND_TO_PROCESS);

	//if it is not 3 chars long or doesn't end with '*' or the second char is not a number
	if ((command_length < 3) || (pCOMMAND_TO_PROCESS[command_length - 1] != '*')|| (!NUMERIC(pCOMMAND_TO_PROCESS[1])) ) return 0;

	processed_cmd->letter = *pCOMMAND_TO_PROCESS++;	//get command letter
	processed_cmd->number = *pCOMMAND_TO_PROCESS++ - '0';	//get command number
	processed_cmd->param_count = 0;  //set to 0. Doesn't work without this even though everything is supposed to initialize to 0..

	if (command_length > 4) //it has parameters, so grab them
	{
		char *one_cmd;
		one_cmd = strtok((char *)pCOMMAND_TO_PROCESS, " *"); //get the first token
		while (one_cmd != NULL)
		{
			processed_cmd->parameters[processed_cmd->param_count] = strtol(one_cmd, NULL, 10);
			one_cmd = strtok(NULL, " *"); //get the next token
			processed_cmd->param_count++;
		}
	}

	processed_cmd->cmd_status = CMD_STATUS_Queued;
	return 1;
}




/*
 * Takes a SEQUENCE_COMMAND struct and executes the command
 */
void FlowRunner_Execute(flow_command_t *command_to_execute)
{
	command_to_execute->cmd_status = CMD_STATUS_Running;
	switch (command_to_execute->letter)
	{
		case 'A':		//ACTUATOR
		{
			switch (command_to_execute->number)
				{
					case 0:
						{
							ACTUATOR_RETRACT(command_to_execute->parameters, command_to_execute->param_count);
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

					case 1:
						{
							ACTUATOR_EXTEND(command_to_execute->parameters, command_to_execute->param_count);
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}
					case 2:
						{
							 int32_t list_of_actuators[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
							 ACTUATOR_RETRACT(list_of_actuators, 18);
							 command_to_execute->cmd_status = CMD_STATUS_Finished;
							 break;
						}
					case 3:
						{
							int32_t list_of_actuators[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
							ACTUATOR_EXTEND(list_of_actuators, 18);
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}
					case 4: //Set actuators p1, p2... to Retracted and block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit - 2x2 = 4 sec.
						{
							FlowRunner_RetractActuatorsAndWaitForLimitSW(command_to_execute);
							break;
						}
					case 5: //Set actuators p1, p2... to Extend and block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit - 2x2 = 4 sec.
						{
							FlowRunner_ExtendActuatorsAndWaitForLimitSW(command_to_execute);
							break;
						}
					case 6:
						{
							int32_t list_of_actuators[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
							ACTUATOR_RETRACT(list_of_actuators, 18);
							uint8_t i;
							for (i = 0; i < 40; i++)	//after approximately 40 x 100 = 4000 milliseconds
								{
									if (Actuator_AreRetracted(list_of_actuators, 18)) break;
									//delay 100ms before rechecking
									if (FlowRunner_WaitForAbort(100)) return; //abort if the bit is set
								}

							//NOTE - should we address abort/reset?
							if (i == 39) command_to_execute->cmd_status = CMD_STATUS_Timeout;
							else command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}
					case 7:
						{
							int32_t list_of_actuators[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
							ACTUATOR_EXTEND(list_of_actuators, 18);
							uint8_t i;
							for (i = 0; i < 40; i++)	//after approximately 40 x 100 = 4000 milliseconds
								{
									if (Actuator_AreExtended(list_of_actuators, 18)) break;
									//delay 100ms before rechecking
									if (FlowRunner_WaitForAbort(100)) return; //abort if the bit is set
								}

							//NOTE - should we address abort/reset?
							if (i == 39) command_to_execute->cmd_status = CMD_STATUS_Timeout;
							else command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}
					case 8: //A8 p1 p2 p3 p4 p5 p6*	- "Lance" a blister. Extend (with block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit). Than Retract again block/wait for feedback to achieve that state with a timeout of 2x expected time to reach limit.
						{
							FlowRunner_ExtendActuatorsAndWaitForLimitSW(command_to_execute);

							//only if the Extend portion completed successfully, execute the retract portion.
							if (command_to_execute->cmd_status == CMD_STATUS_Finished) FlowRunner_RetractActuatorsAndWaitForLimitSW(command_to_execute);
							break;
						}

					default:
						command_to_execute->cmd_status = CMD_STATUS_Invalid;
				}

			break;
		}

		case 'D':	//DELAY
			{
				switch (command_to_execute->number)
				{
					case 0:
					{	//block for the desired time or until an abort is encountered
						xEventGroupWaitBits(xFlowRunner_EventGroup, FRF_Abort, pdFALSE, pdFALSE, pdMS_TO_TICKS(command_to_execute->parameters[0]));
						command_to_execute->cmd_status = CMD_STATUS_Finished;
						break;
					}
					default:
						command_to_execute->cmd_status = CMD_STATUS_Invalid;
				}

				break;
			}

		case 'C':	//CALIBRATE
			{
				switch (command_to_execute->number)
					{
						case 0:  //ALL SENSORS
						{
							for (uint8_t i = 0; i < BA_ADC_SENSOR_COUNT; i++) FlowRunner_SensorCalibration(i, command_to_execute);
							break;
						}  //end case 0
						case 1:  //ALL other cases (example Sensor 1)
						{
							FlowRunner_SensorCalibration(command_to_execute->parameters[3] - 1, command_to_execute);  //Sensors are 0 based.
						}
						default:
							command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}
				break;
			}

		case 'H':		//Heater commands
			{
				switch (command_to_execute->number)
					{
					case 0:  //Turn heater # off
						{
							for (uint8_t i = 0; i < command_to_execute->param_count; i++)  //loop through all available parameters
							HEATER_CTRL_HeaterOFF(command_to_execute->parameters[i] - 1);  //-1 since heater is zero based
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

					case 1:  //Turn heater #1,2,3,4 on with a setpoint of 7000/100 = 70.00 C
					case 2:
					case 3:
					case 4:
						{	//heater is 0 based so (-1)
							HEATER_CTRL_HeaterON(command_to_execute->number - 1, command_to_execute->parameters[0]/ 100.0f);
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

					case 5: //Block until Heater 1 reading is between min and max value.
					case 6: //Block until Heater 2 reading is between min and max value.
					case 7: //Block until Heater 3 reading is between min and max value.
					case 8: //Block until Heater 4 reading is between min and max value.
						{  //parameters - minval maxval timeout* Min/Max value are/ 100. timeout is in seconds.
							float minVal 	 = command_to_execute->parameters[0]/ 100.0f;
							float maxVal 	 = command_to_execute->parameters[1]/ 100.0f;
							uint32_t timeout = command_to_execute->parameters[2];
							uint8_t heatNum  = command_to_execute->number - 5;    // -5 because H5 relates to H1, but heater is zero based

							command_to_execute->cmd_status = FlowRunner_WaitForHeaterToBeInRange(heatNum, minVal, maxVal, timeout);
							break;
						}

					case 9: //H9 lowTemp highTemp lowDwell highDwell CycleNums useFan*  - PCR
						{
							float lowTemp        = command_to_execute->parameters[0]/ 100.0f;
							float highTemp       = command_to_execute->parameters[1]/ 100.0f;
							float diviation		 = 0.5f;  //use for InRange command. 0.5 = +/- 0.5 C
							uint32_t lowDwell    = command_to_execute->parameters[2];
							uint32_t highDwell   = command_to_execute->parameters[3];
							uint8_t	 totalCycles = command_to_execute->parameters[4];
							uint8_t  useFan      = command_to_execute->parameters[5];
							uint16_t timeout = 30000;										//time to wait to get in range before timing out

							//go to low temperature
							HEATER_CTRL_HeaterON(0, lowTemp);
							HEATER_CTRL_HeaterON(1, lowTemp);

							for(uint8_t cycleNum = 0; cycleNum < totalCycles; cycleNum++)
							{
								//wait for low temperature to be obtained
								if (FlowRunner_WaitForHeaterToBeInRange(1, (lowTemp - diviation), (lowTemp + diviation), timeout) == CMD_STATUS_Timeout)
								{
									command_to_execute->cmd_status = CMD_STATUS_Timeout;
									return;
								}
								//if usefan was true, turn fan off
								if (useFan) Fan_Control_Off();

								//wait for lowDwell
								if (FlowRunner_WaitForAbort(lowDwell)) return; //abort if the bit is set

								//go to high temperature
								HEATER_CTRL_HeaterON(0, highTemp);
								HEATER_CTRL_HeaterON(1, highTemp);

								//wait for high temperature to be obtained
								if (FlowRunner_WaitForHeaterToBeInRange(1, (highTemp - diviation), (highTemp + diviation), timeout) == CMD_STATUS_Timeout)
								{
									command_to_execute->cmd_status = CMD_STATUS_Timeout;
									return;
								}

								//wait for highDwell
								if (FlowRunner_WaitForAbort(highDwell)) return; //abort if the bit is set

								//if useFan is true, turn on fan and set heater control to low temperature
								if (useFan) Fan_Control_On();
								HEATER_CTRL_HeaterON(0, lowTemp);
								HEATER_CTRL_HeaterON(1, lowTemp);
							}

							//At the end of the last cycle, wait for low temp to be obtained, turn fan off and dwell again.

							//wait for low temperature to be obtained
							if (FlowRunner_WaitForHeaterToBeInRange(1, (lowTemp - diviation), (lowTemp + diviation), timeout) == CMD_STATUS_Timeout)
								{
								command_to_execute->cmd_status = CMD_STATUS_Timeout;
								return;
								}

							//if usefan was true, turn fan off
							if (useFan) Fan_Control_Off();

							//wait for lowDwell
							if (FlowRunner_WaitForAbort(lowDwell)) return; //abort if the bit is set

							HEATER_CTRL_HeaterOFF(0);
							HEATER_CTRL_HeaterOFF(1);

							command_to_execute->cmd_status = CMD_STATUS_Finished;

							break;
						}

					default:
						command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}

				break;
			}

		case 'I':
			{
				switch (command_to_execute->number)
					{
					case 0: //I0*				- Take an image
						USB_SendFromTask(100, BA_msgID_Command, "Take an image\r\n");
						break;
					case 1: //I1*				- Take preTMB image
						USB_SendFromTask(100, BA_msgID_Command, "Take preTMB image\r\n");
						break;
					case 2: //I2*				- Take result Image
						USB_SendFromTask(100, BA_msgID_Command, "Take Result image\r\n");
						break;
					default:
						command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}

				break;
			}


		case 'M':	//Mixer control
			{
				switch (command_to_execute->number)
					{
						case 0://M0*				- Mixer off - No stir bar alignment. Will execute within microseconds.
						{
							Mixer_Control_MixerOff();
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

						case 1://M1* 				- Mixer off - With stir bar alignment. Blocking - will take 1-3 seconds to align stir bar.
						{
							FlowRunner_StopMixerWithStirBarAlignment(command_to_execute);
							if (command_to_execute->cmd_status == CMD_STATUS_Running) command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

						case 2://M2 P1*			- Mixer off - With stir bar alignment P1 checks if a minimum number of revolutions have been completed. If not, an error is generated. Blocking - will take 1-3 seconds to align stir bar.
						{
							FlowRunner_StopMixerWithStirBarAlignment(command_to_execute);

							//check if minimum number of revolutions were completed and command is still in running state
							if ((Mixer_Control_GetRevs() >= command_to_execute->parameters[0]))
									{
								    if (command_to_execute->cmd_status == CMD_STATUS_Running) command_to_execute->cmd_status = CMD_STATUS_Finished;
									}
							else command_to_execute->cmd_status = CMD_STATUS_Error;

							break;
						}

						case 3://M3 p1*			- Mixer On at p1 duty cycle. Duty cycle values are 0 to 999. Will execute within microseconds.
						{
							Mixer_Control_MixerOn(command_to_execute->parameters[0]);
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

						case 4://M4 p1 p2 p3*		- Where P1 is run until x number of revolutions are achieved, p2 is timeout period, p3 is duty cycle. Non-Blocking. With stir bar alignment.
						{
							break;
						}

						case 5://M5 p1 p2 p3*		- Where P1 is run for x amount of time, P2 is a minimum number of revolutions, P3 is duty cycle. Non-Blocking. With stir bar alignment.
						{
							break;
						}


						default:
							command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}
				break;
			}

		case 'F':	//FAN
			{
				switch (command_to_execute->number)
					{
						case 0://Fan off
						{
							Fan_Control_Off();
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}
						case 1://Fan On
						{
							Fan_Control_On();
							command_to_execute->cmd_status = CMD_STATUS_Finished;
							break;
						}

						default:
							command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}
				break;
			}


		case 'P':	//PID and PCR
			{
				switch (command_to_execute->number)
					{
						case 1://kP kI kD* //set PID values for Heater 1. kP, kI and kD are /10000, so 1/10000 would be 0.0001
						case 2:
						case 3:
						case 4: //it is the same for 1,2,3 and 4 . heatNum is zero based so (-1)
							{
								HEATER_CTRL_SetPID(command_to_execute->number - 1, command_to_execute->parameters[0]/100.0f,
										command_to_execute->parameters[1]/1000.0f, command_to_execute->parameters[2]/1000.0f );
								command_to_execute->cmd_status = CMD_STATUS_Finished;
								break;
							}

						default:
							command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}
				break;
			}


		case 'S':	//STEPPER Control
			{
				switch (command_to_execute->number)
					{
						case 0://S0 Mnum runRate*	- Stepper Home a single motor *Mnum are 1,2,3,4,5,6,7,8 ;
							{
								uint8_t Mnum = command_to_execute->parameters[0] - 1; //motor num is zero based
								uint32_t runRate			= command_to_execute->parameters[1];

								L6470DrvMtx[Mnum].Command->SetParam(Mnum, L6470_MAX_SPEED_ID, Step_s_2_MaxSpeed(runRate));
								L6470DrvMtx[Mnum].Command->Move(Mnum, L6470_DIR_FWD_ID, 120000); //start the move

								//wait for command to complete
								command_to_execute->cmd_status = FlowRunner_WaitForOneStepperMoveToComplete(Mnum);
								break;
							}

						case 1://S1 Mnum travelDistance runRate* - Stepper motor move a single motor
							{
								uint8_t Mnum = command_to_execute->parameters[0] - 1; //motor num is zero based
								uint32_t travelDistance = abs(command_to_execute->parameters[1]);
								uint32_t runRate			= command_to_execute->parameters[2];
								uint8_t Direction = (command_to_execute->parameters[1] > 0) ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID;

								L6470DrvMtx[Mnum].Command->SetParam(Mnum, L6470_MAX_SPEED_ID, Step_s_2_MaxSpeed(runRate));
								L6470DrvMtx[Mnum].Command->Move(Mnum, Direction, travelDistance);  //start the move

								//wait for command to complete
								command_to_execute->cmd_status = FlowRunner_WaitForOneStepperMoveToComplete(Mnum);
								break;
							}

						case 2://S2 Mnum travelDistance runRate stopOnFD DeltaVoltage keepgoing*
							{
								uint8_t Mnum 			    = command_to_execute->parameters[0] - 1; //motor num is zero based
								uint32_t travelDistance     = abs(command_to_execute->parameters[1]);
								uint32_t runRate			= command_to_execute->parameters[2];
								uint8_t motorDirection 	    = (command_to_execute->parameters[1] > 0) ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID;
								uint8_t FDnum     	    	= command_to_execute->parameters[3] - 1;  //fdnum is zero based
								uint8_t keepgoing  			= command_to_execute->parameters[5];
								uint8_t pushingToSensor		= ((FDnum >= 0) && (FDnum <= 10));		  //if a valid sensor number is provided, pushingToSensor is true

								L6470DrvMtx[Mnum].Command->SetParam(Mnum, L6470_MAX_SPEED_ID, Step_s_2_MaxSpeed(runRate));
								uint32_t LowThreshold, HighThreshold;

								if (pushingToSensor)
								{
									uint32_t startValue = Fluid_Sensor_GetValue(FDnum);

									//if looking for a voltage increase
									if (command_to_execute->parameters[4] > 0)
									{
										LowThreshold = 0;

										if ((startValue + command_to_execute->parameters[4]) > 4095) HighThreshold = 4095;  //clamp it at 4095
										else HighThreshold = startValue + command_to_execute->parameters[4];
									}
									else  //if looking for a voltage decrease
									{
										HighThreshold = 4095;

										if ((startValue + command_to_execute->parameters[4]) < 0) LowThreshold = 0;  //clamp it to 0
										else LowThreshold  = startValue + command_to_execute->parameters[4];
									}

									//NOTE If LowThreshold = 0 and HighThreshold = 4095, the watch dog covers the entire range and a sensor trip can not be detected.
									if ((LowThreshold == 0) && (HighThreshold = 4095) && (keepgoing == 0))
									{
										command_to_execute->cmd_status = CMD_STATUS_Error;
										break;
									}

									//since ulTaskNotifyTake waits for the notification value to be grater than 0, clear it - just in case it is already grater than 0
									ulTaskNotifyTake(pdTRUE, 0);

									//configure the watch dog window. Anything outside of this window will be considered a sensor trip
									Fluid_Sensor_SetWatchDog(FDnum, LowThreshold, HighThreshold, FlowRunner_TaskHandle);
								}


								//start the move
								L6470DrvMtx[Mnum].Command->Move(Mnum, motorDirection, travelDistance);

								//block on watch dog signal while motor is still busy

								while(1)  //while the motor is busy with the move
								{
									//if the returned value is 0, the xTicksToWait expired prior to receiving a notification. If > 0, a notification was received
									if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)) > 0)
									{
										//no need to clear the watch dog - already cleared in the fluid_sensor.c
										L6470DrvMtx[Mnum].Command->HardStop(Mnum); //stop the motor
										command_to_execute->cmd_status = CMD_STATUS_Finished;
										break;
									}

									//if the motor is no longer busy - the move has completed
									if (L6470DrvMtx[Mnum].Command->CheckStatusRegisterFlag(Mnum, BUSY_ID))
									{
										if (pushingToSensor) Fluid_Sensor_ClearWatchDog();	//clear the watch dog - it is fine to do that
										if (keepgoing == 0) command_to_execute->cmd_status = CMD_STATUS_Error;  //pushingToSensor = false should be used with KeepGoing = true
										else command_to_execute->cmd_status = CMD_STATUS_FullStroke;

										break;
									}
								}

								break;
							}
						case 3: //S3 Mnum travelDistance runRate stopOnFD1 stopOnFD2 deltaVoltage*				// runs until travelDistance is obtained or (stopOnFD1 and stopOnFD2) are triggered.
							{



								break;
							}

						case 8://S8 travelDist1 travelDist2 travelDist3 travelDist4 travelDist5 travelDist6
							{
								for (uint8_t i = 0; i < 6; i++)
								{
									L6470DrvMtx[i].Command->SetParam(i, L6470_MAX_SPEED_ID, Step_s_2_MaxSpeed(1000));
									L6470DrvMtx[i].Command->Move(i, L6470_DIR_REV_ID, command_to_execute->parameters[i]);  //start all the moves
								}

								//wait for all 6 motor moves to complete
								FlowRunner_WaitForSixStepperMovesToComplete(command_to_execute);
								break;
							}
						case 9:  ////S9 runRate* Home all stepper motors at runRate
							{
								uint32_t runRate			= command_to_execute->parameters[0];

								for (uint8_t i = 0; i < 6; i++)
								{	//if the home flag is not set, move the motor toward home - enough steps to get there
									//if (!L6470DrvMtx[i].Command->CheckStatusRegisterFlag(i, SW_F_ID))
									L6470DrvMtx[i].Command->SetParam(i, L6470_MAX_SPEED_ID, Step_s_2_MaxSpeed(runRate));
									L6470DrvMtx[i].Command->Move(i, L6470_DIR_FWD_ID, 120000);
								}

								//wait for all 6 motor moves to complete
								FlowRunner_WaitForSixStepperMovesToComplete(command_to_execute);
								break;
							}

						default:
							command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}
				break;
			}

		case 'T':
			{
				switch (command_to_execute->number)
					{
					case 0: //T0*				- Time remaining in seconds
						{
						char timeRemStr[10];
						itoa(command_to_execute->parameters[0], timeRemStr, 10);

						USB_SendFromTask(100, BA_msgID_Command, "Time remaining: ");
						USB_SendFromTask(100, BA_msgID_Command, timeRemStr);
						break;
						}
					default:
						command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}

				break;
			}

		case 'W':	//Wait command
			{
				switch (command_to_execute->number)
					{
						case 0://W0 p1*			       - wait for all prior commands to complete with a timeout
						case 1://W1 p1 p2 p3 p4 p5 p6* - wait for command at (p2 and p3 and p4 and p5 and p6) to complete with a timeout of p1
						case 2://W2 p1 p2 p3 p4 p5 p6* - wait for command at (p2 or p3 or p4 or p5 or p6) to complete with a timeout of p1

						break;

						default:
							command_to_execute->cmd_status = CMD_STATUS_Invalid;
					}
				break;
			}

		default:
			command_to_execute->cmd_status = CMD_STATUS_Invalid;

	}


	//if the command didn't finish successfully, and it is not already aborting, check if abort is needed
	if ((command_to_execute->cmd_status != CMD_STATUS_Finished) && (!FlowRunner_IsAborting()))
	{
		FlowRunner_CheckIfAbortIsRequired(command_to_execute);
	}
}


static void FlowRunner_CheckIfAbortIsRequired(flow_command_t *command_to_execute)
{

	//max stroke
	if ((command_to_execute->letter == 'S') && (command_to_execute->number == 2) && (command_to_execute->cmd_status == CMD_STATUS_Error))
		{
		//FlowRunner_AbortRun();
		USB_SendFromTask(0, BA_msgID_Command,"Max stroke. Aborting...\r\n");
		}


}


/*
 * Waits for a specific heater to be within the specified range.
 * If the range is not obtained within the specified timeout, or if it is aborted, the returned value is CMD_STATUS_Timeout
 * If the range is obtained, the returned value is CMD_STATUS_Finished
 */
static uint8_t FlowRunner_WaitForHeaterToBeInRange(uint8_t heatNum, float lowVal, float highVal, uint32_t timeout)
{
	uint32_t totalWait = 0;  //in seconds

	while(timeout > totalWait)
	{
		if (HEATER_CTRL_HeaterIsInRange(heatNum, lowVal, highVal) == 1)	// -5 because H5 relates to H1, but heater is zero based
			{
				return CMD_STATUS_Finished;
			}

	//delay 100ms before rechecking
	if (FlowRunner_WaitForAbort(100)) return CMD_STATUS_Timeout; //abort if the bit is set

	totalWait += 100;
	}

	return CMD_STATUS_Timeout;
}


/*
 * Extends the actuators and blocks while monitoring the their limit switches to achieve commanded state.
 * Timeout is set at approximately 4s
 * Delay between checks is approximately 100ms.
 */
static void FlowRunner_RetractActuatorsAndWaitForLimitSW(flow_command_t *command_to_execute)
{
	ACTUATOR_RETRACT(command_to_execute->parameters, command_to_execute->param_count);

	//simplified in order to avoid having to deal with potential variable overflow...
	uint8_t i;
	for (i = 0; i < 40; i++)	//after approximately 40 x 100 = 4000 milli seconds
		{
			if (Actuator_AreRetracted(command_to_execute->parameters, command_to_execute->param_count))	break;

			//delay 100ms before rechecking
			if (FlowRunner_WaitForAbort(100)) return;  //abort if the bit is set
		}

	//NOTE - should we address abort/reset?
	if (i == 39) command_to_execute->cmd_status = CMD_STATUS_Timeout;
	else command_to_execute->cmd_status = CMD_STATUS_Finished;
}

/*
 * Extends the actuators and blocks while monitoring the their limit switches to achieve commanded state.
 * Timeout is set at approximately 4s
 * Delay between checks is approximately 100ms.
 */
static void FlowRunner_ExtendActuatorsAndWaitForLimitSW(flow_command_t *command_to_execute)
{
	ACTUATOR_EXTEND(command_to_execute->parameters, command_to_execute->param_count);

	//simplified in order to avoid having to deal with potential variable overflow... (if tick timestamp was used instead)
	uint8_t i;
	for (i = 0; i < 40; i++)	//after approximately 40 x 100 = 4000 milli seconds
		{
			if (Actuator_AreExtended(command_to_execute->parameters, command_to_execute->param_count))	break;

			//delay 100ms before rechecking
			if (FlowRunner_WaitForAbort(100)) return;  //abort if the bit is set
		}

	//NOTE - should we address abort/reset?
	if (i == 39) command_to_execute->cmd_status = CMD_STATUS_Timeout;
	else command_to_execute->cmd_status = CMD_STATUS_Finished;
}

/*
 * Blocks while monitoring stepper motor to stop reporting as busy.
 * Timeout is set at approximately 30s
 * Delay between checks is approximately 10ms.
 * motorNum is zero based
 * Returns 0 for timeout, 1 for complete, 2 for abort
 */
static enum COMMAND_STATUS FlowRunner_WaitForOneStepperMoveToComplete(uint8_t motorNum)
{
	uint16_t timeout = 0;

	while (1)
	{
		if (L6470DrvMtx[motorNum].Command->CheckStatusRegisterFlag(motorNum, BUSY_ID))
		{
			return CMD_STATUS_Finished;
		}
		else if (timeout > 3000)  //timeout set at approximately 30 seconds for now (3000 x 10ms delay). Stop the motor
		{
			L6470DrvMtx[motorNum].Command->HardStop(motorNum);
			return CMD_STATUS_Timeout;
		}
		else if (FlowRunner_WaitForAbort(10)) //10ms delay per loop. If an abort is called, stop the motor
		{
			L6470DrvMtx[motorNum].Command->HardStop(motorNum);
			return CMD_STATUS_Aborting;
		}

		timeout++;
	}
}

/*
 * Blocks while monitoring stepper motor 0 to 5 to stop reporting as busy.
 * Timeout is set at approximately 30s
 * Delay between checks is approximately 10ms.
 */
static void FlowRunner_WaitForSixStepperMovesToComplete(flow_command_t *command_to_execute)
{
	//wait for all 6 motor moves to complete
	uint8_t IsComplete[6] = {0,0,0,0,0,0};
	uint16_t timeout = 0;

	while (1)
	{
		IsComplete[0] = L6470DrvMtx[0].Command->CheckStatusRegisterFlag(0, BUSY_ID);
		IsComplete[1] = L6470DrvMtx[1].Command->CheckStatusRegisterFlag(1, BUSY_ID);
		IsComplete[2] = L6470DrvMtx[2].Command->CheckStatusRegisterFlag(2, BUSY_ID);
		IsComplete[3] = L6470DrvMtx[3].Command->CheckStatusRegisterFlag(3, BUSY_ID);
		IsComplete[4] = L6470DrvMtx[4].Command->CheckStatusRegisterFlag(4, BUSY_ID);
		IsComplete[5] = L6470DrvMtx[5].Command->CheckStatusRegisterFlag(5, BUSY_ID);

		if (IsComplete[0] && IsComplete[1] && IsComplete[2] && IsComplete[3] && IsComplete[4] && IsComplete[5])
		{
			command_to_execute->cmd_status = CMD_STATUS_Finished;

			//Check for home sensor being on!
			break;
		}
		else if (timeout > 3000)  //timeout set at approximately 30 seconds for now (3000 x 10ms delay). Stop the motors
		{
			L6470DrvMtx[0].Command->HardStop(0);
			L6470DrvMtx[1].Command->HardStop(1);
			L6470DrvMtx[2].Command->HardStop(2);
			L6470DrvMtx[3].Command->HardStop(3);
			L6470DrvMtx[4].Command->HardStop(4);
			L6470DrvMtx[5].Command->HardStop(5);
			command_to_execute->cmd_status = CMD_STATUS_Timeout;
			break;
		}
		else if (FlowRunner_WaitForAbort(10)) //10ms delay per loop. If an abort is called, stop the motors
		{
			L6470DrvMtx[0].Command->HardStop(0);
			L6470DrvMtx[1].Command->HardStop(1);
			L6470DrvMtx[2].Command->HardStop(2);
			L6470DrvMtx[3].Command->HardStop(3);
			L6470DrvMtx[4].Command->HardStop(4);
			L6470DrvMtx[5].Command->HardStop(5);
			break;
		}

		timeout++;
	}
}


/*
 * Slows the mixing motor to 15% duty cycle, waits 500ms and counts 2 revolutions.
 * At the instance of the detection of the second revolution, the motor is stopped.
 * In case the 2 revolutions are not detected, the command will timeout in approx. 8 seconds.
 */
static void FlowRunner_StopMixerWithStirBarAlignment(flow_command_t *command_to_execute)
{
	//set mixer duty to 150 and delay 500ms
	Mixer_Control_MixerOn(150);
	if (FlowRunner_WaitForAbort(500)) return;

	uint32_t endRev = Mixer_Control_GetRevs() + 2;  //Mixer_Control_GetRevs will most likely return 0 as the Mixer_Control_MixerOn sets the counter to 0
	uint16_t timeout = 0;

	while (endRev > Mixer_Control_GetRevs())
	{
		if (FlowRunner_WaitForAbort(10)) break;		//10ms delay. break out of the loop if aborting - passes through  Mixer_Control_MixerOff
		timeout++;

		if (timeout > 800) //timeout = approx 800x10 = 8000 ms
			{
			command_to_execute->cmd_status = CMD_STATUS_Timeout;
			break;
			}
	}
	Mixer_Control_MixerOff();
}


static void FlowRunner_SensorCalibration(uint8_t FS_num, flow_command_t *command_to_execute)
{
	uint16_t minVal = command_to_execute->parameters[0];
	uint16_t maxVal = command_to_execute->parameters[1];
	enum POT_MEM whichMem = command_to_execute->parameters[2]; //Volatile(0) or NoN-volatile(1)

	uint16_t dpotVal = FLUID_CAL_READPOT(FS_num, whichMem, SENSOR, 0);
	uint8_t Attempts = 0;

	//USB_SendPrivate("Calibrate #%u to %u - %u. Current POT = %u\r\n", FS_num, minVal, maxVal, dpotVal);
	//USB_SendPrivate("ADC before: %u\r\n",ADCBuffer[FS_num - 1] );

	if (dpotVal == 256)
		{
		command_to_execute->cmd_status = CMD_STATUS_Error;
		BA_ERROR_HANLDER(BA_ERROR_MCP4661_READ, FS_num);
		return;
		}

	while ((Fluid_Sensor_GetValue(FS_num) < minVal) || (Fluid_Sensor_GetValue(FS_num) > maxVal))
		{
		if (Fluid_Sensor_GetValue(FS_num) > maxVal)
			{
			if (dpotVal == 0)
				{
				command_to_execute->cmd_status = CMD_STATUS_Error;
				BA_ERROR_HANLDER(BA_ERROR_MCP4661_RANGE, FS_num);
				return;
				}

			dpotVal--;
			}
		else if (Fluid_Sensor_GetValue(FS_num) < minVal)
			{
			if (dpotVal == 255)
				{
				command_to_execute->cmd_status = CMD_STATUS_Error;
				BA_ERROR_HANLDER(BA_ERROR_MCP4661_RANGE, FS_num);
				return;
				}

			dpotVal++;
			}

			Attempts++;
			if (FLUID_CAL_WRITEPOT(FS_num, whichMem, SENSOR, 0, dpotVal) != HAL_OK)
	    		{
				command_to_execute->cmd_status = CMD_STATUS_Error;
				BA_ERROR_HANLDER(BA_ERROR_MCP4661_WRITE, FS_num);
				break;
	    		}

			if (Attempts == 255)
	    		{
				command_to_execute->cmd_status = CMD_STATUS_Error;
				BA_ERROR_HANLDER(BA_ERROR_MCP4661_RANGE, FS_num);
				break;
	    		}


			if (FlowRunner_WaitForAbort(3)) return; //abort if the bit is set
			//HAL_Delay(3);
		}

	if (command_to_execute->cmd_status != CMD_STATUS_Error) command_to_execute->cmd_status = CMD_STATUS_Finished;
	//USB_SendPrivate("ADC after: %u.POT = %u\r\n", ADCBuffer[FS_num - 1], dpotVal);
}
