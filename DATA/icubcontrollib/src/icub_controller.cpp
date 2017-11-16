////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017 Francesco Mannella
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////
#include "icub_controller.h"
#include <algorithm>
#include <iterator>

//-------------------------------------------------------------------------

const BodyPartTypesToBodyPartNames BodyDB::bodyPartTypesToBodyPartNames = {
    {BodyPartType::HEAD, "head"},
    {BodyPartType::LEFT_ARM, "left_arm"},
    {BodyPartType::RIGHT_ARM, "right_arm"},
    {BodyPartType::TORSO, "torso"},
    {BodyPartType::NONE, "none"} };

const JointIDsToJointNames BodyDB::jointIDsToJointNames = {
		{JointID::NECK_PITCH, "NECK_PITCH"},
		{JointID::NECK_ROLL, "NECK_ROLL"},
		{JointID::NECK_YAW, "NECK_YAW"},
		{JointID::EYES_TILT, "EYES_TILT"},
		{JointID::EYES_VERSION, "EYES_VERSION"},
		{JointID::EYES_VERGENCE, "EYES_VERGENCE"},
		{JointID::LEFT_SHOULDER_PITCH, "LEFT_SHOULDER_PITCH"},
		{JointID::LEFT_SHOULDER_ROLL, "LEFT_SHOULDER_ROLL"},
		{JointID::LEFT_SHOULDER_YAW, "LEFT_SHOULDER_YAW"},
		{JointID::LEFT_ELBOW, "LEFT_ELBOW"},
		{JointID::LEFT_WRIST_PROSUP, "LEFT_WRIST_PROSUP"},
		{JointID::LEFT_WRIST_PITCH, "LEFT_WRIST_PITCH"},
		{JointID::LEFT_WRIST_YAW, "LEFT_WRIST_YAW"},
		{JointID::LEFT_HAND_FINGER, "LEFT_HAND_FINGER"},
		{JointID::LEFT_THUMB_OPPOSE, "LEFT_THUMB_OPPOSE"},
		{JointID::LEFT_THUMB_PROXIMAL, "LEFT_THUMB_PROXIMAL"},
		{JointID::LEFT_THUMB_DISTAL, "LEFT_THUMB_DISTAL"},
		{JointID::LEFT_INDEX_PROXIMAL, "LEFT_INDEX_PROXIMAL"},
		{JointID::LEFT_INDEX_DISTAL, "LEFT_INDEX_DISTAL"},
		{JointID::LEFT_MIDDLE_PROXIMAL, "LEFT_MIDDLE_PROXIMAL"},
		{JointID::LEFT_MIDDLE_DISTAL, "LEFT_MIDDLE_DISTAL"},
		{JointID::LEFT_PINKY, "LEFT_PINKY"},
		{JointID::RIGHT_SHOULDER_PITCH, "RIGHT_SHOULDER_PITCH"},
		{JointID::RIGHT_SHOULDER_ROLL, "RIGHT_SHOULDER_ROLL"},
		{JointID::RIGHT_SHOULDER_YAW, "RIGHT_SHOULDER_YAW"},
		{JointID::RIGHT_ELBOW, "RIGHT_ELBOW"},
		{JointID::RIGHT_WRIST_PROSUP, "RIGHT_WRIST_PROSUP"},
		{JointID::RIGHT_WRIST_PITCH, "RIGHT_WRIST_PITCH"},
		{JointID::RIGHT_WRIST_YAW, "RIGHT_WRIST_YAW"},
		{JointID::RIGHT_HAND_FINGER, "RIGHT_HAND_FINGER"},
		{JointID::RIGHT_THUMB_OPPOSE, "RIGHT_THUMB_OPPOSE"},
		{JointID::RIGHT_THUMB_PROXIMAL, "RIGHT_THUMB_PROXIMAL"},
		{JointID::RIGHT_THUMB_DISTAL, "RIGHT_THUMB_DISTAL"},
		{JointID::RIGHT_INDEX_PROXIMAL, "RIGHT_INDEX_PROXIMAL"},
		{JointID::RIGHT_INDEX_DISTAL, "RIGHT_INDEX_DISTAL"},
		{JointID::RIGHT_MIDDLE_PROXIMAL, "RIGHT_MIDDLE_PROXIMAL"},
		{JointID::RIGHT_MIDDLE_DISTAL, "RIGHT_MIDDLE_DISTAL"},
		{JointID::RIGHT_PINKY, "RIGHT_PINKY"},
		{JointID::TORSO_YAW, "TORSO_YAW"},
		{JointID::TORSO_ROLL, "TORSO_ROLL"},
		{JointID::TORSO_PITCH, "TORSO_PITCH"},
		{JointID::NONE, "none"} };


const Joints BodyDB::joints = {
		{BodyPartType::HEAD, JointType::NECK_PITCH, JointID::NECK_PITCH, 0, 0},
		{BodyPartType::HEAD, JointType::NECK_ROLL, JointID::NECK_ROLL, 1, 1},
		{BodyPartType::HEAD, JointType::NECK_YAW, JointID::NECK_YAW, 2, 2},
		{BodyPartType::HEAD, JointType::EYES_TILT, JointID::EYES_TILT, 3, 3},
		{BodyPartType::HEAD, JointType::EYES_VERSION, JointID::EYES_VERSION, 4, 4},
		{BodyPartType::HEAD, JointType::EYES_VERGENCE, JointID::EYES_VERGENCE, 5, 5},
		{BodyPartType::LEFT_ARM, JointType::SHOULDER_PITCH, JointID::LEFT_SHOULDER_PITCH, 0, 6},
		{BodyPartType::LEFT_ARM, JointType::SHOULDER_ROLL, JointID::LEFT_SHOULDER_ROLL, 1, 7},
		{BodyPartType::LEFT_ARM, JointType::SHOULDER_YAW, JointID::LEFT_SHOULDER_YAW, 2, 8},
		{BodyPartType::LEFT_ARM, JointType::ELBOW, JointID::LEFT_ELBOW, 3, 9},
		{BodyPartType::LEFT_ARM, JointType::WRIST_PROSUP, JointID::LEFT_WRIST_PROSUP, 4, 10},
		{BodyPartType::LEFT_ARM, JointType::WRIST_PITCH, JointID::LEFT_WRIST_PITCH, 5, 11},
		{BodyPartType::LEFT_ARM, JointType::WRIST_YAW, JointID::LEFT_WRIST_YAW, 6, 12},
		{BodyPartType::LEFT_ARM, JointType::HAND_FINGER, JointID::LEFT_HAND_FINGER, 7, 13},
		{BodyPartType::LEFT_ARM, JointType::THUMB_OPPOSE, JointID::LEFT_THUMB_OPPOSE, 8, 14},
		{BodyPartType::LEFT_ARM, JointType::THUMB_PROXIMAL, JointID::LEFT_THUMB_PROXIMAL, 9, 15},
		{BodyPartType::LEFT_ARM, JointType::THUMB_DISTAL, JointID::LEFT_THUMB_DISTAL, 10, 16},
		{BodyPartType::LEFT_ARM, JointType::INDEX_PROXIMAL, JointID::LEFT_INDEX_PROXIMAL, 11, 17},
		{BodyPartType::LEFT_ARM, JointType::INDEX_DISTAL, JointID::LEFT_INDEX_DISTAL, 12, 18},
		{BodyPartType::LEFT_ARM, JointType::MIDDLE_PROXIMAL, JointID::LEFT_MIDDLE_PROXIMAL, 13, 19},
		{BodyPartType::LEFT_ARM, JointType::MIDDLE_DISTAL, JointID::LEFT_MIDDLE_DISTAL, 14, 20},
		{BodyPartType::LEFT_ARM, JointType::PINKY, JointID::LEFT_PINKY, 15, 21},
		{BodyPartType::RIGHT_ARM, JointType::SHOULDER_PITCH, JointID::RIGHT_SHOULDER_PITCH, 0, 22},
		{BodyPartType::RIGHT_ARM, JointType::SHOULDER_ROLL, JointID::RIGHT_SHOULDER_ROLL, 1, 23},
		{BodyPartType::RIGHT_ARM, JointType::SHOULDER_YAW, JointID::RIGHT_SHOULDER_YAW, 2, 24},
		{BodyPartType::RIGHT_ARM, JointType::ELBOW, JointID::RIGHT_ELBOW, 3, 25},
		{BodyPartType::RIGHT_ARM, JointType::WRIST_PROSUP, JointID::RIGHT_WRIST_PROSUP, 4, 26},
		{BodyPartType::RIGHT_ARM, JointType::WRIST_PITCH, JointID::RIGHT_WRIST_PITCH, 5, 27},
		{BodyPartType::RIGHT_ARM, JointType::WRIST_YAW, JointID::RIGHT_WRIST_YAW, 6, 28},
		{BodyPartType::RIGHT_ARM, JointType::HAND_FINGER, JointID::RIGHT_HAND_FINGER, 7, 29},
		{BodyPartType::RIGHT_ARM, JointType::THUMB_OPPOSE, JointID::RIGHT_THUMB_OPPOSE, 8, 30},
		{BodyPartType::RIGHT_ARM, JointType::THUMB_PROXIMAL, JointID::RIGHT_THUMB_PROXIMAL, 9, 31},
		{BodyPartType::RIGHT_ARM, JointType::THUMB_DISTAL, JointID::RIGHT_THUMB_DISTAL, 10, 32},
		{BodyPartType::RIGHT_ARM, JointType::INDEX_PROXIMAL, JointID::RIGHT_INDEX_PROXIMAL, 11, 33},
		{BodyPartType::RIGHT_ARM, JointType::INDEX_DISTAL, JointID::RIGHT_INDEX_DISTAL, 12, 34},
		{BodyPartType::RIGHT_ARM, JointType::MIDDLE_PROXIMAL, JointID::RIGHT_MIDDLE_PROXIMAL, 13, 35},
		{BodyPartType::RIGHT_ARM, JointType::MIDDLE_DISTAL, JointID::RIGHT_MIDDLE_DISTAL, 14, 36},
		{BodyPartType::RIGHT_ARM, JointType::PINKY, JointID::RIGHT_PINKY, 15, 37},
		{BodyPartType::TORSO, JointType::TORSO_YAW, JointID::TORSO_YAW, 0, 38},
		{BodyPartType::TORSO, JointType::TORSO_ROLL, JointID::TORSO_ROLL, 1, 39},
		{BodyPartType::TORSO, JointType::TORSO_PITCH, JointID::TORSO_PITCH, 2, 40} };


//-------------------------------------------------------------------------
double distance_between_positions(JointStorage pos1,  JointStorage pos2)
{
	arma::vec v_pos1( pos1.getValues());
	arma::vec v_pos2( pos2.getValues());

	return arma::norm(v_pos1 - v_pos2, 2.0);
}
//-------------------------------------------------------------------------

const JointStorage ICUB_controller::reset_positions = JointIDValues{ {
	{JointID::NECK_PITCH, 0.5333700000},
	{JointID::NECK_ROLL, 0.0002086500},
	{JointID::NECK_YAW, 0.0001759040},
	{JointID::EYES_TILT, 0.0001088080},
	{JointID::EYES_VERSION, -0.0002806160},
	{JointID::EYES_VERGENCE, -0.0003280490},
	{JointID::LEFT_SHOULDER_PITCH, -29.4321000000},
	{JointID::LEFT_SHOULDER_ROLL, 29.5261000000},
	{JointID::LEFT_SHOULDER_YAW, 0.0489219000},
	{JointID::LEFT_ELBOW, 44.8168000000},
	{JointID::LEFT_WRIST_PROSUP, 0.0068318600},
	{JointID::LEFT_WRIST_PITCH, 0.0090891300},
	{JointID::LEFT_WRIST_YAW, 0.0327174000},
	{JointID::LEFT_HAND_FINGER, 0.0046809800},
	{JointID::LEFT_THUMB_OPPOSE, 0.0020667500},
	{JointID::LEFT_THUMB_PROXIMAL, -0.0005082330},
	{JointID::LEFT_THUMB_DISTAL, -0.0002919410},
	{JointID::LEFT_INDEX_PROXIMAL, 0.0005654950},
	{JointID::LEFT_INDEX_DISTAL, 0.0007068610},
	{JointID::LEFT_MIDDLE_PROXIMAL, 0.0008197060},
	{JointID::LEFT_MIDDLE_DISTAL, 0.0005372380},
	{JointID::LEFT_PINKY, 0.0001174530},
	{JointID::RIGHT_SHOULDER_PITCH, -29.4319000000},
	{JointID::RIGHT_SHOULDER_ROLL, 29.5260000000},
	{JointID::RIGHT_SHOULDER_YAW, 0.0477196000},
	{JointID::RIGHT_ELBOW, 44.8166000000},
	{JointID::RIGHT_WRIST_PROSUP, 0.0050251500},
	{JointID::RIGHT_WRIST_PITCH, 0.0091485500},
	{JointID::RIGHT_WRIST_YAW, 0.0326768000},
	{JointID::RIGHT_HAND_FINGER, 0.0075651100},
	{JointID::RIGHT_THUMB_OPPOSE, 0.0036898800},
	{JointID::RIGHT_THUMB_PROXIMAL, 0.0012740500},
	{JointID::RIGHT_THUMB_DISTAL, 0.0226544000},
	{JointID::RIGHT_INDEX_PROXIMAL, -0.0002059350},
	{JointID::RIGHT_INDEX_DISTAL, 0.0000412867},
	{JointID::RIGHT_MIDDLE_PROXIMAL, 0.0011709800},
	{JointID::RIGHT_MIDDLE_DISTAL, 0.0006721090},
	{JointID::RIGHT_PINKY, 0.0012035900},
	{JointID::TORSO_YAW, 0.0012035900},
	{JointID::TORSO_ROLL, 0.0012035900},
	{JointID::TORSO_PITCH, 0.0012035900} }
};


ICUB_controller::ICUB_controller(
		yarp::os::Network &_yarp,
        const BodyPartTypes& _body_part_types
        ) : curr_body_parts(_body_part_types), yarp(_yarp), mode(Mode::NONE)
{

}


void ICUB_controller::init(
        const Mode &_mode,
        const std::string &robotname
        )
{

    mode = _mode;

    std::string root = "/";
    root += robotname + "/";

    std::string local_ports = "/";
    switch(mode)
    {
        case Mode::TORQUE :
            local_ports += "torqueModule";
            break;
        case Mode::VELOCITY :
            local_ports += "velocityModule";
            break;
        case Mode::POSITION :
        	local_ports += "positionModule";
        	break;
        case Mode::POSITION_DIRECT :
        	local_ports += "positionDirectModule";
        	break;
    }

    for(auto used_body_part: curr_body_parts )
    {
        const std::string &body_part_name = BodyDB::getBodyPartName(used_body_part);
        std::string remote_ports = root;
        remote_ports += body_part_name;

        std::string cur_local_ports = local_ports;
        cur_local_ports += "_";
        cur_local_ports += body_part_name;


        yError("remote port: %s",remote_ports.c_str());

        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("local", cur_local_ports.c_str());   //local port names
        options.put("remote", remote_ports.c_str());         //where we connect to

        // create a device
        robot_devices[body_part_name] = std::unique_ptr<yarp::dev::PolyDriver>(new yarp::dev::PolyDriver(options) );

        if (not robot_devices[body_part_name]->isValid()) {
            yError("Device not available.  Here are the known devices:\n");
            yError("%s", yarp::dev::Drivers::factory().toString().c_str());
            throw BadDeviceException();
        }

        // acquire the control-mode interface
        yarp::dev::IControlMode2 *control_mode;
        if(not robot_devices[body_part_name]->view(control_mode) )
        {
            yError("problems acquiring the control-mode interface\n");
            throw BadInterfaceException();
        }
        control_modes[body_part_name] = control_mode;

        // acquire the position-control interface
        yarp::dev::IPositionControl *position_control;
        if(not robot_devices[body_part_name]->view(position_control))
        {
            yError("problems acquiring the position-control interface\n");
            throw BadInterfaceException();
        }
        positions[body_part_name] = position_control;
        positions[body_part_name]->getAxes(&joints_number);

        // acquire the position-direct-control interface
        yarp::dev::IPositionDirect *position_direct_control;
        if(not robot_devices[body_part_name]->view(position_direct_control))
        {
            yError("problems acquiring the position-direct-control interface\n");
            throw BadInterfaceException();
        }
        direct_positions[body_part_name] = position_direct_control;
        direct_positions[body_part_name]->getAxes(&joints_number);

        // acquire the velocity-control interface
        yarp::dev::IVelocityControl *velocity_control;
        if(not robot_devices[body_part_name]->view(velocity_control))
        {
            yError("problems acquiring the velocity-control interface\n");
            throw BadInterfaceException();
        }
        velocities[body_part_name] = velocity_control;

        // acquire the torque-control interface
        yarp::dev::ITorqueControl *torque_control;
        if (not  robot_devices[body_part_name]->view(torque_control))
        {
            yError("problems acquiring the torque-control interface\n");
            throw BadInterfaceException();
        }
        torques[body_part_name] = torque_control;

        // acquire the amplifier-control interface
        yarp::dev::IAmplifierControl *amplifier_control;
        if (not  robot_devices[body_part_name]->view(amplifier_control))
        {
            yError("problems acquiring the amplifier-control interface\n");
            throw BadInterfaceException();
        }
        amplifiers[body_part_name] = amplifier_control;

        // acquire the encoders interface
        yarp::dev::IEncoders *encs;
        if (not  robot_devices[body_part_name]->view(encs))
        {
            yError("problems acquiring the encoders interface\n");
            throw BadInterfaceException();
        }
        encoders[body_part_name] = encs;

    }

}


void ICUB_controller::saveJoints(const std::string &filename)
{
	std::ofstream file(filename);
	JointStorage joints = read();
	for(const auto &joint_value: joints.jointValues)
		file << (BodyDB::getJointName(joint_value.first)) << " "
			 << joint_value.second << std::endl;

}


JointStorage ICUB_controller::loadJoints(const std::string &filename)
{
	std::ifstream file(filename);
	JointStorage joints;
	if(file.is_open())
	{
		while(not file.eof())
		{
			std::string joint_name;
			double joint_value;
			file >> joint_name >> joint_value;

			joints.setJointValue(
					BodyDB::getJoint(joint_name), joint_value);
		}
	}

	return joints;
}


void ICUB_controller::setControlMode(const Joint &joint, const Mode &_mode)
{

	JointID joint_id = joint.joint_id;
	BodyPartType body_part_type = joint.body_part_type;
    std::string body_part_name = BodyDB::getBodyPartName(body_part_type);
    int joint_number = BodyDB::getJointNumber(joint_id);

    Mode current_mode;
    if (_mode != Mode::NONE )
        current_mode = _mode;
    else
        current_mode = mode;

    // set comtrol mode
    int mode = 0;
    control_modes[body_part_name]->getControlMode(joint_number, &mode);
    switch(current_mode)
    {
        case Mode::TORQUE :
            if (mode != VOCAB_CM_TORQUE)
                control_modes[body_part_name]->setTorqueMode(joint_number);
            break;
        case Mode::VELOCITY :
            if (mode != VOCAB_CM_VELOCITY)
                control_modes[body_part_name]->setVelocityMode(joint_number);
            break;
        case Mode::POSITION :
            if (mode != VOCAB_CM_POSITION)
                control_modes[body_part_name]->setPositionMode(joint_number);
            break;
        case Mode::POSITION_DIRECT :
            if (mode != VOCAB_CM_POSITION_DIRECT)
                control_modes[body_part_name]->setControlMode(joint_number,VOCAB_CM_POSITION_DIRECT);
            break;
    }
}

void ICUB_controller::setControlModes(const Mode &_mode)
{
    Mode curr_mode;
    if (_mode != Mode::NONE )
        curr_mode = _mode;
    else
        curr_mode = mode;

    for(auto body_part_type: curr_body_parts )
        for (const auto &joint: BodyDB::getJoints(body_part_type))
            setControlMode(joint, curr_mode);
}

BodyPartTypes ICUB_controller::getBodyParts()
{
    return curr_body_parts;
}

void ICUB_controller::move(const Joint &joint, double value)
{

	JointID joint_id = joint.joint_id;
	BodyPartType body_part_type = joint.body_part_type;
    std::string body_part_name = BodyDB::getBodyPartName(body_part_type);
    int joint_number = BodyDB::getJointNumber(joint_id);

    switch(mode)
    {
        case Mode::TORQUE :
            setControlMode(joint, Mode::TORQUE);
            torques[body_part_name]->setRefTorque(joint_number, value);
            break;
        case Mode::VELOCITY :
            setControlMode(joint, Mode::VELOCITY);
            velocities[body_part_name]->velocityMove(joint_number, value);
            break;
        case Mode::POSITION :
            setControlMode(joint, Mode::POSITION);
            positions[body_part_name]->positionMove(joint_number, value);
            break;
        case Mode::POSITION_DIRECT :
            setControlMode(joint, Mode::POSITION_DIRECT);
            direct_positions[body_part_name]->setPosition(joint_number, value);
            break;
    }
}

void ICUB_controller::setSpeed(const Joint &joint, double speed)
{

	JointID joint_id = joint.joint_id;
	BodyPartType body_part_type = joint.body_part_type;
    std::string body_part_name = BodyDB::getBodyPartName(body_part_type);
    int joint_number = BodyDB::getJointNumber(joint_id);

    switch(mode)
    {
        case Mode::VELOCITY :
            setControlMode(joint, Mode::VELOCITY);
            velocities[body_part_name]->setRefAcceleration(joint_number, speed);
            break;
        case Mode::POSITION :
            setControlMode(joint, Mode::POSITION);
            positions[body_part_name]->setRefSpeed(joint_number, speed);
            break;
    }
}

double ICUB_controller::read(const Joint &joint)
{
	JointID joint_id = joint.joint_id;
	BodyPartType body_part_type = joint.body_part_type;
    std::string body_part_name = BodyDB::getBodyPartName(body_part_type);
    int joint_number = BodyDB::getJointNumber(joint_id);

    double value = 0;
    encoders[body_part_name]->getEncoder(joint_number, &value);

    return value;
}


void ICUB_controller::move(JointID joint_id, double value)
{
	move(BodyDB::getJoint(joint_id), value);
}

void ICUB_controller::setSpeed(JointID joint_id, double speed)
{
	setSpeed(BodyDB::getJoint(joint_id), speed);
}

double ICUB_controller::read(JointID joint_id)
{
	return read(BodyDB::getJoint(joint_id));
}

void ICUB_controller::move(const std::string &joint_name, double value)
{
	move(BodyDB::getJoint(joint_name), value);
}

void ICUB_controller::setSpeed(const std::string &joint_name, double speed)
{
	setSpeed(BodyDB::getJoint(joint_name), speed);
}

double ICUB_controller::read(const std::string &joint_name)
{
	return read(BodyDB::getJoint(joint_name));
}

void ICUB_controller::setSpeed(double speed)
{
	for(const auto &joint: BodyDB::joints)
		setSpeed(joint, speed);
}

void ICUB_controller::move(const JointStorage &joints)
{
	for(auto &joint_value: joints.jointValues)
		move(joint_value.first, joint_value.second);
}

JointStorage ICUB_controller::read()
{
    JointStorage store;
    double value = 0;
    for (auto body_part_type: curr_body_parts )
    {
        for( const auto &joint: BodyDB::getJoints(body_part_type) )
        {
            encoders[BodyDB::getBodyPartName(body_part_type)]->getEncoder(
                    BodyDB::getJointNumber(joint.joint_id), &value );

            store.setJointValue(joint, value);
        }
    }
    return store;
}




void ICUB_controller::reset(JointStorage custom_reset_positions,
		double distance_epsilon,
		double speed, double time_step)
{
    //initial distance
    double distance_from_reset =
    		distance_between_positions(
    				custom_reset_positions,
					reset_positions);

    // move all joints
	setSpeed(speed);
	move(reset_positions);

    while( distance_from_reset > distance_epsilon )
    {
        // wait for a delta-time interval
    	yarp::os::Time::delay(time_step);

    	custom_reset_positions = read();

    	// current distance
        distance_from_reset =
        		distance_between_positions(
        				custom_reset_positions,
    					reset_positions);
    }
}

void ICUB_controller::reset(double distance_epsilon,
		double speed, double time_step)
{

    // read current positions
    JointStorage curr_joint_positions = read();

    reset(curr_joint_positions, distance_epsilon,
    		speed, time_step);

}

//-------------------------------------------------------------------------

