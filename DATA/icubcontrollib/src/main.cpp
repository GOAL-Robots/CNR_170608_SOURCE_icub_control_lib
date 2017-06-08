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
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>

int main(void) 
{

    {
        // POSITION CONTROL

        Network yarp;

        double time_step = 1.0/150.0;

        // initialize the controller
        ICUB_controller icub( yarp );

        // we will use position control
        icub.init(Mode::POSITION_DIRECT);


        // print joints to stderr
        for(auto joint_value: ICUB_controller::reset_positions.jointValues )
        {
        	const Joint & joint = joint_value.first;
        	double value = joint_value.second;

            yInfo("joint: %10s  value: %8.4f\n",
					BodyDB::getJointName(joint).c_str(),
					value );
        }

        JointStorage init_joint_positions = icub.read();

        icub.reset();

        // retrieve the value of the chosen joint
        double value = init_joint_positions.getJointValue(JointID::LEFT_SHOULDER_PITCH);

        // initially set the desired position to the current position
        double target_position = value;

        // set maximum speed
        icub.setSpeed(100000.0);

        for(double t = 0; t< 2.4*M_PI; t+=0.01 )
        {
            // update desired position
            target_position = (sin(t)*0.5+0.5)*(M_PI/2.0)*(180.0/M_PI);

            // read current joint position
            value = icub.read(JointID::LEFT_SHOULDER_ROLL);

            // move to the current desired position
            icub.move( JointID::LEFT_SHOULDER_ROLL, target_position);
            icub.move( JointID::TORSO_ROLL, target_position*0.1);
            icub.move( JointID::TORSO_PITCH, target_position*0.1);

            yInfo("%s: %8.4f\n",
            		BodyDB::getJointName(JointID::LEFT_SHOULDER_ROLL).c_str(),
					value);

            // wait for a delta-time interval
            yarp::os::Time::delay(time_step);
        }

        // RESET to initial position -----------------------------------------------------

		icub.reset();
    }

    return 0;
}
