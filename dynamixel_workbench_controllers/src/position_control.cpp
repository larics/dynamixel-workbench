/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "dynamixel_workbench_controllers/position_control.h"

PositionControl::PositionControl()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Position");
  dxl_wb_->addSyncRead("Present_Current");
  dxl_wb_->addSyncRead("Present_Position");
  dxl_wb_->addSyncRead("Present_Velocity");
  dxl_wb_->addSyncRead("Present_Input_Voltage");
  voltage_old = 0;
  // BBULK
  /*dxl_wb_->initBulkRead();
  for (int index = 0; index < dxl_cnt_; index ++)
  {
    dxl_wb_->addBulkReadParam(dxl_id_[index], "Present_Position");
    dxl_wb_->addBulkReadParam(dxl_id_[index], "Present_Current");
  }*/

  initPublisher();
  initSubscriber();
  initServer();
}

PositionControl::~PositionControl()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void PositionControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  dynamixel_workbench controller; position control example             \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void PositionControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 1);
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void PositionControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 1, &PositionControl::goalJointPositionCallback, this);
}

void PositionControl::initServer()
{
  joint_command_server_ = node_handle_.advertiseService("joint_command", &PositionControl::jointCommandMsgCallback, this);
}

void PositionControl::dynamixelStatePublish(bool read_voltage)
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;
  
  //bool temp = dxl_wb_->setBulkRead();

  int32_t* present_current = dxl_wb_->syncRead("Present_Current");
  for (int index = 0; index < dxl_cnt_; index++)
  {
    //dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    //dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    //dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    //dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    //dynamixel_state[index].present_current     = dxl_wb_->itemRead(dxl_id_[index], "Present_Current");
    //dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    //dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    //dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");
    //dynamixel_state[index].present_position    = present_position[index];
    dynamixel_state[index].present_current     = present_current[index];
    dynamixel_state[index].present_current_real = double(dynamixel_state[index].present_current)*0.00269; // Convert to amps
    // BBULK
    //dynamixel_state[index].present_position    = dxl_wb_->bulkRead(dxl_id_[index], "Present_Position");
    //dynamixel_state[index].present_current     = dxl_wb_->bulkRead(dxl_id_[index], "Present_Current");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }

  int32_t* present_position = dxl_wb_->syncRead("Present_Position");
  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state_list.dynamixel_state[index].present_position = present_position[index];
    dynamixel_state_list.dynamixel_state[index].present_position_real = float(present_position[index]-2048)*0.0015358897;
  }

  if (read_voltage == true)
  {
    int32_t* present_input_voltage = dxl_wb_->syncRead("Present_Input_Voltage");
    for (int index = 0; index < dxl_cnt_; index++)
    {
      dynamixel_state_list.dynamixel_state[index].present_input_voltage = present_input_voltage[index];
      voltage_old = dynamixel_state_list.dynamixel_state[index].present_input_voltage;
      dynamixel_state_list.dynamixel_state[index].present_input_voltage_real = float(dynamixel_state_list.dynamixel_state[index].present_input_voltage)*0.1;
    }
  }
  else
  {
    for(int index = 0; index < dxl_cnt_; index++)
    {
      dynamixel_state_list.dynamixel_state[index].present_input_voltage = voltage_old;
      dynamixel_state_list.dynamixel_state[index].present_input_voltage_real = float(dynamixel_state_list.dynamixel_state[index].present_input_voltage)*0.1;
    }
  }

  /*int32_t* present_velocity = dxl_wb_->syncRead("Present_Velocity");
  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state_list.dynamixel_state[index].present_velocity = present_velocity[index];
  }*/

  // Torque is based on parameters provided by robotis (graphically identified). T = (1.67I-0.05)*U/11.1
  // The 11.1 is voltage the graph was provided with.
  for(int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state_list.dynamixel_state[index].present_torque_real = (1.67*dynamixel_state_list.dynamixel_state[index].present_current_real
      -0.05)*(dynamixel_state_list.dynamixel_state[index].present_input_voltage_real/11.1);
  }

  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void PositionControl::jointStatePublish()
{
  int32_t present_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

  int32_t present_velocity[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

  sensor_msgs::JointState dynamixel_;
  dynamixel_.header.stamp = ros::Time::now();

  for (int index = 0; index < dxl_cnt_; index++)
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(dxl_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
  }
  joint_states_pub_.publish(dynamixel_);
}

void PositionControl::controlLoop(bool read_voltage)
{
  dynamixelStatePublish(read_voltage);
  //jointStatePublish();
}

bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  int32_t goal_position = 0;
  int32_t present_position = 0;

  if (req.unit == "rad")
  {
    goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
  }
  else if (req.unit == "raw")
  {
    goal_position = req.goal_position;
  }
  else
  {
    goal_position = req.goal_position;
  }

  bool ret = dxl_wb_->goalPosition(req.id, goal_position);

  res.result = ret;
}

void PositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double goal_position[dxl_cnt_] = {0.0, };

  for (int index = 0; index < dxl_cnt_; index++)
    goal_position[index] = msg->position.at(index);

  int32_t goal_dxl_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
  {
    goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
  }

  dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "position_control");
  PositionControl pos_ctrl;

  ros::Rate loop_rate(250);
  double start_time = ros::Time::now().toSec();
  double elapsed_time = 0.0;
  int voltage_iter = 0, voltage_iter_old = -1;
  bool read_voltage = false;
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
    read_voltage = false;
    elapsed_time = ros::Time::now().toSec() - start_time;
    voltage_iter = floor(elapsed_time/15.0);
    if (voltage_iter != voltage_iter_old)
    {
      voltage_iter_old = voltage_iter;
      read_voltage = true;
    }
    pos_ctrl.controlLoop(read_voltage);
  }

  return 0;
}
