#include <cstdio>
#include "kuka_iiwa_driver/kukaiiwa_client.hpp"

#include <cstring>
//******************************************************************************
iiwaClient::iiwaClient()
{	
	// current_control_mode = control_mode; // TODO: Another posibility is to create a funtion to set this mode.
	// current_session_state = iiwa_sessionS;
	// memcpy(cmd_jnt_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	for(unsigned i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
		cmd_jnt_pos[i] = 0.0;
	
	for(unsigned i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
		cmd_torques[i] = 0.0;
	
	for(unsigned i=0; i<CART_VECTOR_DIM; i++)
		cmd_wrench[i] = 0.0;
}

//******************************************************************************
iiwaClient::~iiwaClient()
{
}
      
//******************************************************************************
void iiwaClient::onStateChange(ESessionState oldState, ESessionState newState)
{
	LBRClient::onStateChange(oldState, newState);
	// react on state change events
	switch (newState)
	{
		case MONITORING_WAIT:
		{
			current_session_state = newState;
			break;
		}       
		case MONITORING_READY:
		{
			this->getContinousState();
			memcpy(cmd_jnt_pos, meas_jnt_pos, LBRState::NUMBER_OF_JOINTS * sizeof(double));
			current_session_state = newState;
			break;
		}
		case COMMANDING_WAIT:
		{
			this->getContinousState();
			memcpy(cmd_jnt_pos, meas_jnt_pos, LBRState::NUMBER_OF_JOINTS * sizeof(double));
			current_session_state = newState;
			break;
		}   
		case COMMANDING_ACTIVE:
		{
			current_session_state = newState;
			break;
		}
		default:
		{
			current_session_state = newState;
			break;
		}
	}
}

//******************************************************************************
void iiwaClient::monitor()
{
	LBRClient::monitor();

	/***************************************************************************/
	/*                                                                         */
	/*   Place user Client Code here                                           */
	/*                                                                         */
	/***************************************************************************/

}

//******************************************************************************
void iiwaClient::waitForCommand()
{
	// In waitForCommand(), the joint values have to be mirrored. Which is done, 
	// by calling the base method.

	/***************************************************************************/
	/*                                                                         */
	/*   Place user Client Code here                                           */
	/*                                                                         */
	/***************************************************************************/ 

	robotCommand().setJointPosition(cmd_jnt_pos);

}

//******************************************************************************
void iiwaClient::command()
{
	/***************************************************************************/
	/*                                                                         */
	/*   Place user Client Code here                                           */
	/*                                                                         */
	/***************************************************************************/   

	// In command(), the joint angle values have to be set. 
	//robotCommand().setJointPosition( newJointValues );
	// LBRClient::command();

	robotCommand().setJointPosition(cmd_jnt_pos);

}

void iiwaClient::getContinousState() {
	memcpy(meas_jnt_pos,this->robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
	memcpy(meas_torques,this->robotState().getMeasuredTorque(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
	memcpy(meas_ext_torques,this->robotState().getExternalTorque(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
}

void iiwaClient::getDiscreteState(){
	commanding_mode = this->robotState().getClientCommandMode();
	current_session_state = this->robotState().getSessionState();
	connection_quality = this->robotState().getConnectionQuality();
}
