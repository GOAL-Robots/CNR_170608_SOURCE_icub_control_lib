/*******************************************************************************
 * Copyright (C) 2017 Francesco Mannella
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/
#ifndef ICUB_CONTROLLERS_H
#define ICUB_CONTROLLERS_H

#include <armadillo>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <map>
#include <vector>
#include <memory>
#include <algorithm>


//using namespace yarp::math;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// ENUM TYPES

/** Type defining the control used */
enum class Mode { TORQUE, VELOCITY, POSITION, POSITION_DIRECT, NONE };

 /** joint identifiers */
 enum class JointType
 {
     // head
     NECK_PITCH, NECK_ROLL, NECK_YAW,
     EYES_TILT, EYES_VERSION, EYES_VERGENCE,
     // arm (left and right have the same kind of joints)
     SHOULDER_PITCH, SHOULDER_ROLL, SHOULDER_YAW, ELBOW,
     WRIST_PROSUP, WRIST_PITCH, WRIST_YAW,
     // hand
     HAND_FINGER, THUMB_OPPOSE, THUMB_PROXIMAL, THUMB_DISTAL,
     INDEX_PROXIMAL, INDEX_DISTAL, MIDDLE_PROXIMAL, MIDDLE_DISTAL, PINKY,
     // torso
     TORSO_YAW, TORSO_ROLL, TORSO_PITCH,
     //
     NONE
 };

 /** Type defining the body parts in the icub simulator.
  *  Left and right arms contain the hand joints
  *
  */
 enum class BodyPartType
 {
	 HEAD, LEFT_ARM, RIGHT_ARM, TORSO, NONE
 };

 enum class JointID
 {
	 // head
	 NECK_PITCH, NECK_ROLL, NECK_YAW,
	 EYES_TILT, EYES_VERSION, EYES_VERGENCE,
	 // arm (left and right have the same kind of joints)
	 LEFT_SHOULDER_PITCH, LEFT_SHOULDER_ROLL, LEFT_SHOULDER_YAW, LEFT_ELBOW,
	 LEFT_WRIST_PROSUP, LEFT_WRIST_PITCH, LEFT_WRIST_YAW,
	 RIGHT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, RIGHT_SHOULDER_YAW, RIGHT_ELBOW,
	 RIGHT_WRIST_PROSUP, RIGHT_WRIST_PITCH, RIGHT_WRIST_YAW,
	 // hand
	 LEFT_HAND_FINGER, LEFT_THUMB_OPPOSE, LEFT_THUMB_PROXIMAL, LEFT_THUMB_DISTAL,
	 LEFT_INDEX_PROXIMAL, LEFT_INDEX_DISTAL, LEFT_MIDDLE_PROXIMAL,
	 LEFT_MIDDLE_DISTAL, LEFT_PINKY,
	 RIGHT_HAND_FINGER, RIGHT_THUMB_OPPOSE, RIGHT_THUMB_PROXIMAL, RIGHT_THUMB_DISTAL,
	 RIGHT_INDEX_PROXIMAL, RIGHT_INDEX_DISTAL, RIGHT_MIDDLE_PROXIMAL,
	 RIGHT_MIDDLE_DISTAL, RIGHT_PINKY,
	 // torso
	 TORSO_YAW, TORSO_ROLL, TORSO_PITCH,
	 //
	 NONE
 };

 //------------------------------------------------------------------------------------------
 //------------------------------------------------------------------------------------------
 // JOINT TYPE

 struct Joint
 {
	 BodyPartType body_part_type;
	 JointType joint_type;
	 JointID joint_id;
	 int joint_number;
	 int joint_id_number;
 };

// add less operator for joint so that it can be used in stl containers
namespace std
{
    template<> struct less<Joint>
    {
       bool operator() (const Joint& lhs, const Joint& rhs) const
       {
           return lhs.joint_id_number < rhs.joint_id_number;
       }
    };
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// ALIASES

using robotDeviceMap = std::map<std::string, std::unique_ptr<yarp::dev::PolyDriver> >;
using controlModeMap = std::map<std::string, yarp::dev::IControlMode2 * >;
using torqueControlMap = std::map<std::string, yarp::dev::ITorqueControl * >;
using velocityControlMap = std::map<std::string, yarp::dev::IVelocityControl * >;
using positionControlMap = std::map<std::string, yarp::dev::IPositionControl * >;
using positionDirectControlMap = std::map<std::string, yarp::dev::IPositionDirect * >;
using amplifierControlMap = std::map<std::string, yarp::dev::IAmplifierControl * >;
using encodersMap = std::map<std::string, yarp::dev::IEncoders * >;
using Joints = std::vector<Joint>;
using BodyPartTypes = std::vector<BodyPartType>;
using BodyPartTypeName = std::pair<BodyPartType, std::string>;
using BodyPartTypesToBodyPartNames = std::map<BodyPartType, std::string>;
using JointTypeName = std::pair<JointID, std::string>;
using JointIDsToJointNames = std::map<JointID, std::string>;
using JointValues = std::map<Joint, double >;
using JointIDValues = std::map<JointID, double >;
using JointValue = JointValues::value_type;
using JointIDValue = JointIDValues::value_type;


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// CLASSES

 /**
  * Manages the mappings from body parts to joints
  */
 struct BodyDB
 {

 	static const BodyPartTypesToBodyPartNames bodyPartTypesToBodyPartNames;
 	static const JointIDsToJointNames jointIDsToJointNames;
 	static const Joints joints;

 	static BodyPartType getBodyPartType(const std::string &bodypart_name)
 	{
 		auto bodypart_type_name_it =  std::find_if(
 				begin(bodyPartTypesToBodyPartNames),
 				end(bodyPartTypesToBodyPartNames),
 				[&](const BodyPartTypeName &type_name){
 			return type_name.second == bodypart_name;
 		});

 		if (bodypart_type_name_it != bodyPartTypesToBodyPartNames.end())
 			return bodypart_type_name_it->first;
 		return BodyPartType::NONE;
 	}

 	static JointID getJointID(const std::string &joint_name)
 	{
 		auto joint_type_name_it =  std::find_if(
 				begin(jointIDsToJointNames),
 				end(jointIDsToJointNames),
 				[&](const JointTypeName &joint_type_name){
 			return joint_type_name.second == joint_name;
 		});

 		if (joint_type_name_it != jointIDsToJointNames.end())
 			return joint_type_name_it->first;
 		return JointID::NONE;
 	}


 	static std::string getBodyPartName(BodyPartType bodypart_type)
 	{
 		auto bodypart_type_name_it =  bodyPartTypesToBodyPartNames.find(bodypart_type);

 		if ( bodypart_type_name_it->first ==  bodypart_type )
 			return bodypart_type_name_it->second;

 		return bodyPartTypesToBodyPartNames.at(BodyPartType::NONE);
 	}

 	static std::string getBodyPartName(const Joint &joint)
 	{
 		return getBodyPartName(joint.body_part_type);
 	}

 	static std::string getJointName(JointID joint_id)
 	{
 		auto joint_type_name_it = jointIDsToJointNames.find(joint_id);

 		if ( joint_type_name_it->first ==  joint_id )
 			return joint_type_name_it->second;

 		return jointIDsToJointNames.at(JointID::NONE);
 	}

 	static std::string getJointName(const Joint &joint)
 	{
 		return getJointName(joint.joint_id);
 	}

 	static JointType getJointType(JointID joint_id)
 	{
 		auto joint_it =  std::find_if(
 				begin(joints), end(joints),
 				[&](const Joint &joint){ return joint.joint_id == joint_id; });

 		if ( joint_it != joints.end())
 			return joint_it->joint_type;

 		return JointType::NONE;
 	}

 	static BodyPartType getBodyPartType(JointID joint_id)
 	{
 		auto joint_it =  std::find_if(
 				begin(joints), end(joints),
 				[&](const Joint &joint){ return joint.joint_id == joint_id; });

 		if ( joint_it != joints.end())
 			return joint_it->body_part_type;

 		return BodyPartType::NONE;
 	}

 	static int getJointNumber(JointID joint_id)
 	{
 		auto joint_it =  std::find_if(
 				begin(joints), end(joints),
 				[&](const Joint &joint){ return joint.joint_id == joint_id; });

 		if ( joint_it != joints.end())
 			return joint_it->joint_number;

 		return -1;
 	}

 	static Joint getJoint(JointID joint_id)
 	{
 		auto joint_it =  std::find_if( begin(joints), end(joints),
 				[&](const Joint &joint){ return joint.joint_id == joint_id; });

 		if ( joint_it != joints.end())
 			return *joint_it;

 		return Joint();
 	}


	static Joint getJoint(const std::string &joint_name)
 	{
		return getJoint(getJointID(joint_name));
 	}

 	static Joints getJoints(BodyPartType body_part_type)
 	{
 		Joints res;
 		std::copy_if( begin(joints), end(joints), std::back_inserter(res),
 				[&](const Joint &joint){
 			return joint.body_part_type == body_part_type;
 		});

 		return res;
 	}
 };

 /**
  * Manages the current values of all joints
  */
struct JointStorage
{

	JointValues jointValues;

	JointStorage() {
		for(auto joint: BodyDB::joints)
			jointValues.insert({joint, 0.0});
	}

	JointStorage(const JointValues &joint_values) {
		for(auto &joint_value: joint_values)
			jointValues.insert({joint_value.first, joint_value.second});
	}

	JointStorage(const JointIDValues &joint_id_values) {
		for(auto &joint_id_value: joint_id_values)
			jointValues.insert({BodyDB::getJoint(joint_id_value.first), joint_id_value.second});
	}

    double getJointValue(const Joint &joint)
    {
		auto joint_value_it = jointValues.find(joint);

		if ( joint_value_it != jointValues.end())
			return joint_value_it->second;

		return 0.0; //TODO: find better return none value (or throw something)
    }

    double getJointValue(const std::string &joint_name)
    {

    	const Joint joint = BodyDB::getJoint(joint_name);
    	auto joint_value_it = jointValues.find(joint);

		if ( joint_value_it != jointValues.end())
			return joint_value_it->second;

		return 0.0; //TODO: find better return none value (or throw something)
    }

    double getJointValue(JointID joint_id)
    {

    	const Joint joint = BodyDB::getJoint(joint_id);
    	auto joint_value_it = jointValues.find(joint);

		if ( joint_value_it != jointValues.end())
			return joint_value_it->second;

		return 0.0; //TODO: find better return none value (or throw something)
    }

    JointValues getJointValues(BodyPartType body_part_type)
	{
    	JointValues matches;

    	for(JointValue &joint_value: jointValues)
    		if (joint_value.first.body_part_type == body_part_type)
    			matches.insert(joint_value);

    	return matches;
	}


    std::vector<double> getValues(BodyPartType body_part_type)
	{
    	JointValues matches = getJointValues(body_part_type);
    	std::vector<double> values;

    	for(const auto &match: matches)
    		values.push_back(match.second);

		return values;
	}

    std::vector<double> getValues()
	{
    	std::vector<double> values;

    	for(const auto &joint_value: jointValues)
    		values.push_back(joint_value.second);

		return values;
	}


    void setJointValue(const Joint &joint, double value)
    {
		auto joint_value_it =  jointValues.find(joint);

		if ( joint_value_it != jointValues.end())
			joint_value_it->second = value;

		// TODO else throw exception (joint not found)

    }

};


/**
 * High level controller with 'move' and 'read' methods
 */
class ICUB_controller
{

	yarp::os::Network &yarp; //!< yarp connection

    BodyPartTypes curr_body_parts; //!< names of the bodies to use
    robotDeviceMap robot_devices; //!< the map of devices for each body part
    controlModeMap control_modes; //!< the map of mode contorllers for each body part
    torqueControlMap torques; //!< the map of torque controllers for each body part
    velocityControlMap velocities; //!< the map of velocity controllers for each body part
    positionControlMap positions; //!< the map of position controllers for each body part
    positionDirectControlMap direct_positions; //!< the map of position direct controllers for each body part
    amplifierControlMap amplifiers; //!< the map of amplifier controllers for each body part
    encodersMap encoders; //!< the map of encoders for each body part

    Mode mode;
    int joints_number = 0;


    public:

    ICUB_controller() = delete;

    static const JointStorage reset_positions;

    /**
     * @param _yarp handler to the yarp net
     * @param _bP a vector of the body parts to use in the simulation. (default to all)
     */
    ICUB_controller(
    		yarp::os::Network& _yarp,
            const BodyPartTypes& _body_part_types = { BodyPartType::HEAD,
                BodyPartType::LEFT_ARM, BodyPartType::RIGHT_ARM,
                BodyPartType::TORSO }
            );

    /** Initialize the controller
     * @param mode the control mode (see the Mode enum type)
     */
    void init(
            const Mode &mode = Mode::POSITION,
            const std::string &robotname = "icubSim"
            );

    /**
     *	save joints to file
     *	@param filename	the name of the file where to save
     */
    void saveJoints(const std::string &filename = "icub_joints");

    /**
     *	save joints to file
     *	@param filename	the name of the file from where to load
     */
    JointStorage loadJoints(const std::string &filename = "icub_joints");


    /** Set control mode
     * @param joint the joint info object
     * @param mode the control mode
     */
    void setControlMode(const Joint &joint,
            const Mode &mode = Mode::NONE);

    //set control mode for each joint
    void setControlModes(const Mode &mode = Mode::NONE);

    /** Gets the current body parts */
    BodyPartTypes getBodyParts();

    /** Move a joint
     * @param joint the joint info object
     * @param value a float value (can be the desired position, velocity or torque, based on the control mode)
     */
    void move(const Joint &joint, double value);

    /** Set the speed of joint change
     * @param joint the joint info object
     * @param speed of joint change
     */
    void setSpeed(const Joint &joint, double speed);

    /** Read current joint position
     * @param joint the joint info object
     */
    double read(const Joint &joint);

    /** Move a joint
     * @param joint_id the id of the joint info object
     * @param value a float value (can be the desired position, velocity or torque, based on the control mode)
     */
    void move(JointID joint_id, double value);

    /** Set the speed of joint change
     * @param joint_id the id of the joint info object
     * @param speed of joint change
     */
    void setSpeed(JointID joint_id, double speed);

    /** Read current joint position
     * @param joint_id the id of the joint info object
     */
    double read(JointID joint_id);

    /** Move a joint
     * @param joint_name the name of the joint info object
     * @param value a float value (can be the desired position, velocity or torque, based on the control mode)
     */
    void move(const std::string &joint_name, double value);

    /** Set the speed of joint change
     * @param joint_name the name of the joint info object
     * @param speed of joint change
     */
    void setSpeed(const std::string &joint_name, double speed);

    /** Read current joint position
     * @param joint_name the name of the joint info object
     */
    double read(const std::string &joint_name);

    /** Set speed of all joints
      * @param speed of joint change
      */
    void setSpeed(double speed);

    /** Move all joints
     * @param joints joints list
     */
    void move(const JointStorage &joints);

    /** Read current  positions of all joints
     */
    JointStorage read();

    /** reset to default position
     * @param distance_epsilon  the minimum distance to acquire
     * @param speed the speed of the movement to reset
     * @param time_step the time_step for computing current distance
     */
    void reset(double distance_epsilon = 5.0,
    		double speed = 100000.0,
			double time_step = 0.1);

    /** reset to custom position
     * @param custom_reset_positions the container with all joint positions
     * @param distance_epsilon  the minimum distance to acquire
     * @param speed the speed of the movement to reset
     * @param time_step the time_step for computing current distance
     */
    void reset( JointStorage custom_reset_positions,
    		double distance_epsilon = 5.0,
    		double speed = 100000.0,
			double time_step = 0.1);
};


double distance_between_positions(JointStorage pos1,  JointStorage pos2);

/** throwed when a joint value does not exist */
class NullValueException : public std::exception
{
    virtual const char* what() const throw()
    {
        return "You did not read this value yet.";
    }
};


/** throwed when a device (body part ini) is not found */
class BadDeviceException : public std::exception
{
    virtual const char* what() const throw()
    {
        return "No Device found";
    }
};

/** throwed when a controller (controlMode, IPosition Ivelocity or ITorque) is not found */
class BadInterfaceException : public std::exception
{
    virtual const char* what() const throw()
    {
        return "Problems Acquiring Interfaces";
    }
};

#endif // ICUB_CONTROLLERS_H
