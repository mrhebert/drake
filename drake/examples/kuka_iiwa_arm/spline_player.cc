/// @file
///
/// Description!

#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>


#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_gps_trial_command.hpp"
#include "drake/lcmt_gps_controller_gains.hpp"
#include "drake/lcmt_gps_sample_result.hpp"
#include "drake/lcmt_gps_data.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

using namespace std;
using ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;
using namespace std::this_thread;
using namespace std::chrono;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using std::cout;
using std::endl;

enum SampleType { //IF PROTOBUF CHANGES, THIS NEEDS TO CHANGE. NEED TO INVESTIGATE IF I CAN JUST IMPORT!
  ACTION = 0,
  JOINT_ANGLES = 1,
  JOINT_VELOCITIES = 2,
  END_EFFECTOR_POINTS = 3,
  END_EFFECTOR_POINT_VELOCITIES = 4,
  END_EFFECTOR_POINT_JACOBIANS = 5,
  END_EFFECTOR_POINT_ROT_JACOBIANS = 6,
  END_EFFECTOR_POSITIONS = 7,
  END_EFFECTOR_ROTATIONS = 8,
  END_EFFECTOR_JACOBIANS = 9,
  END_EFFECTOR_HESSIANS = 10,
  RGB_IMAGE = 11,
  DEPTH_IMAGE = 12,
  RGB_IMAGE_SIZE = 13,
  CONTEXT_IMAGE = 14,
  CONTEXT_IMAGE_SIZE = 15,
  IMAGE_FEAT = 16,
  END_EFFECTOR_POINTS_NO_TARGET = 17,
  END_EFFECTOR_POINT_VELOCITIES_NO_TARGET = 18,
  TOTAL_DATA_TYPES = 19,
  JOINT_TORQUES = 20,
};


namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {


const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmRunControllerChannel = "GPS_RUN_CONTROLLER";
const char* const kLcmControllerData = "GPS_DATA_RESULT";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmFFCommandChannel = "IIWA_FEED_FORWARD_COMMAND";
const char* const kLcmCancelPlanChannel = "CANCEL_PLAN";

const char* const kURDF = "IIWA_COMMAND";
const char* const BASE_FRAME = "iiwa_link_0";
const char* const EE_FRAME = "iiwa_link_ee";
 const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";


  class RobotController {
  public:

    /// tree is aliased
    explicit RobotController(const RigidBodyTree<double>& tree)
        : tree_(tree) {
      lcm_.subscribe(kLcmStatusChannel, &RobotController::HandleStatus, this);
      lcm_.subscribe(kLcmCommandChannel, &RobotController::HandleCommand, this);
      lcm_.subscribe(kLcmFFCommandChannel, &RobotController::HandleFeedForwardCommand, this);
    }

    void runSpline() {
      gotStatusupdate_ = false;
      while (0 == lcm_.handleTimeout(10)) { }


      int currentCommand = 0;
      int numCommands = spline_command_list_.size();
      double dt = 0;
      double desired_dt = 0;
      int start_time = 0;
      double total_time = 0;
      double total_spline_time = 0;

      while(currentCommand < numCommands) {
        while (gotStatusupdate_ == false) {
          lcm_.handle();
        }
        gotStatusupdate_ = false;

        dt = (iiwa_status_.utime - start_time);

        // cout << "DT " << dt << " desired_dt " << desired_dt << " " << numCommands << " " << currentCommand<< endl;
        if (dt > desired_dt) {
          start_time = iiwa_status_.utime;
          lcmt_iiwa_command iiwa_command;
          iiwa_command = spline_command_list_.at(currentCommand);
          for(int j = 0; j < 7; j++){
            iiwa_command.joint_position.at(j) = iiwa_status_.joint_position_measured.at(j);
          }
          iiwa_command.utime = start_time;
          // for ( int i = 0; i < 7; i++) {
          //   cout <<currentCommand << "PPUBLISH COMMADN " << spline_command_list_.at(currentCommand).joint_torque.at(i) << endl;
          // }
          lcm_.publish(kLcmCommandChannel, &iiwa_command);
          status_list_.push_back(iiwa_status_);
          command_list_.push_back(iiwa_command);


          vector<double> list;
          list.clear();
          list.push_back(dt);
          list.push_back(desired_dt);
          if(currentCommand != 0) {
            total_time = total_time + dt;
            total_spline_time = total_spline_time + desired_dt;
          }
          list.push_back(total_time);
          list.push_back(total_spline_time);
          time_list_.push_back(list);


          currentCommand++;
          if (currentCommand >= numCommands) {
            break;
          }

          desired_dt = spline_command_list_.at(currentCommand).utime - spline_command_list_.at(currentCommand - 1).utime;


          while (desired_dt == 0){
            currentCommand++;
            if (currentCommand >= numCommands) {
              break;
            }
            desired_dt = spline_command_list_.at(currentCommand).utime - spline_command_list_.at(currentCommand - 1).utime;
          }
        }
      }

      VectorXd curr(7);

      for(int j = 0; j < 7; j++){
        curr(j) = iiwa_status_.joint_position_measured.at(j);
      }
      send_reset(curr);

      write_command_to_file(command_list_ , "/home/momap/TorqueReplayData/command_list_.csv");
      write_command_to_file(spline_command_list_ , "/home/momap/TorqueReplayData/spline_command_list_.csv");
      write_status_to_file(spline_status_list_ , "/home/momap/TorqueReplayData/spline_status_list_.csv");
      write_status_to_file(status_list_ , "/home/momap/TorqueReplayData/status_list_.csv");
      write_time_to_file(time_list_ , "/home/momap/TorqueReplayData/timing_list_.csv");

    }


  void write_command_to_file(vector<lcmt_iiwa_command> data, std::string fname){
    ofstream file;
    file.open(fname);
    cout << fname << " " << data.size() << endl;
    for(unsigned int i = 0; i < data.size(); i++) {
      for(int j = 0; j < 7; j++){
        if (j < 6){
          file << boost::lexical_cast<std::string>(data.at(i).joint_torque.at(j)) + " , ";
        }else{
          file << boost::lexical_cast<std::string>(data.at(i).joint_torque.at(j));
        }
      }
      file << "\n";
    }
    file.close();
  }
  void write_status_to_file(vector<lcmt_iiwa_status> data, std::string fname){
    ofstream file;
    file.open(fname);
    cout << fname << " " << data.size() << endl;
    for(unsigned int i = 0; i < data.size(); i++) {
      for(int j = 0; j < 7; j++){
        if (j < 6){
          file << boost::lexical_cast<std::string>(data.at(i).joint_position_measured.at(j)) + " , ";
        }else{
          file << boost::lexical_cast<std::string>(data.at(i).joint_position_measured.at(j));
        }
      }
      file << "\n";
    }
    file.close();
  }
  void write_time_to_file(vector<vector<double>> data, std::string fname){
    ofstream file;
    file.open(fname);
    cout << fname << " " << data.size() << endl;
    for(unsigned int i = 0; i < data.size(); i++) {
      file << boost::lexical_cast<std::string>(data.at(i).at(0)) << ", " << boost::lexical_cast<std::string>(data.at(i).at(1)) << ", " << boost::lexical_cast<std::string>(data.at(i).at(2))  << ", " << boost::lexical_cast<std::string>(data.at(i).at(3));
      file << "\n";
    }
    file.close();
  }




  void send_reset(Eigen::Ref<VectorXd> goal_pos) {
       lcm_.handle();

       robotlocomotion::robot_plan_t plan;
       plan.utime = iiwa_status_.utime;
       plan.robot_name = "lbr_iiwa_14";
       plan.num_states = 2;

      //  plan.plan_info.clear();
       plan.plan_info.push_back(1);
       plan.plan_info.push_back(1);
       plan.num_grasp_transitions = 0;
       plan.left_arm_control_type = 0;
       plan.right_arm_control_type = 0;
       plan.left_leg_control_type = 0;
       plan.right_leg_control_type = 0;
       plan.num_bytes = 0;

       bot_core::robot_state_t start;
       bot_core::robot_state_t goal;
       start.utime = 0.0;
       goal.utime = (2 * 1e6);
       start.num_joints = 7;
       goal.num_joints = 7;

      //  start.joint_name.clear();
      //  goal.joint_name.clear();

       start.joint_name.push_back("iiwa_joint_1");
       start.joint_name.push_back("iiwa_joint_2");
       start.joint_name.push_back("iiwa_joint_3");
       start.joint_name.push_back("iiwa_joint_4");
       start.joint_name.push_back("iiwa_joint_5");
       start.joint_name.push_back("iiwa_joint_6");
       start.joint_name.push_back("iiwa_joint_7");
       goal.joint_name.push_back("iiwa_joint_1");
       goal.joint_name.push_back("iiwa_joint_2");
       goal.joint_name.push_back("iiwa_joint_3");
       goal.joint_name.push_back("iiwa_joint_4");
       goal.joint_name.push_back("iiwa_joint_5");
       goal.joint_name.push_back("iiwa_joint_6");
       goal.joint_name.push_back("iiwa_joint_7");


      //  start.joint_velocity.clear();
      //  goal.joint_velocity.clear();
      //  start.joint_effort.clear();
      //  goal.joint_position.clear();
      //  start.joint_position.clear();
      //  goal.joint_effort.clear();

      for (int i = 0; i < 7; i++) {
        start.joint_velocity.push_back(0.0);
        goal.joint_velocity.push_back(0.0);
        start.joint_effort.push_back(0.0);
        goal.joint_effort.push_back(0.0);
        goal.joint_position.push_back(goal_pos(i));
        start.joint_position.push_back(iiwa_status_.joint_position_measured.at(i));
        std::cout << "EEEE" << iiwa_status_.joint_position_measured.at(i) << "," << goal_pos(i)<< std::endl;
      }

      // plan.plan.clear();
      plan.plan.push_back(start);
      plan.plan.push_back(goal);
      cout << "2! "  << endl;

      lcm_.publish(kLcmPlanChannel, &plan);
      cout << "3! "  << endl;

  }

     void wait_for_convergance(Eigen::Ref<VectorXd> goal_pos){
       bool currently_resetting = true;
       double epsilon = 0.03;
       while (currently_resetting == true) {
         lcm_.handle();

         VectorXd curr(7);
         for (int i = 0; i < 7; i++) {
           curr(i) = iiwa_status_.joint_position_measured.at(i);
         }

         double diff = (goal_pos - curr).squaredNorm();
         cout << "DIFF: " << diff << endl;

         if(diff < epsilon) {
           currently_resetting = false;
         }
       }
     }


     void record_spline(Eigen::Ref<VectorXd> goal_pos){
       bool currently_resetting = true;
       double epsilon = 0.03;
       while (currently_resetting == true) {
         lcm_.handle();


        //  cout << spline_command_list_.back().utime << " " << iiwa_command_.utime << endl;
         if(spline_command_list_.size() == 0 || spline_command_list_.back().utime != iiwa_command_.utime ) {
           spline_status_list_.push_back(iiwa_status_);
           spline_command_list_.push_back(iiwa_command_);
           spline_ff_command_list_.push_back(iiwa_feedforward_command_);
         }

         VectorXd curr(7);
         for (int i = 0; i < 7; i++) {
           curr(i) = iiwa_status_.joint_position_measured.at(i);
         }

         double diff = (goal_pos - curr).squaredNorm();
         if(diff < epsilon) {
           currently_resetting = false;
           break;
         }
       }
     }


     void generate_spline() {
       Eigen::VectorXd goal(7);
       Eigen::VectorXd init_pos(7);

       for(int i = 0; i < 7; i++) {
         goal(i) = 0.4;
         init_pos(i) = 0.0;
       }

       send_reset(init_pos);
       wait_for_convergance(init_pos);

      // sleep_until(system_clock::now() + seconds(1.0));

       send_reset(goal);
       record_spline(goal);


       send_reset(init_pos);
       wait_for_convergance(init_pos);

       lcm_.publish(kLcmCancelPlanChannel, &iiwa_status_);


       runSpline();
       cout << "END OF METHOD!!! "  << endl;
     }




  private:
    void HandleStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                      const lcmt_iiwa_status* status) {
      iiwa_status_ = *status;
      gotStatusupdate_= true;
    }
    void HandleCommand(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                      const lcmt_iiwa_command* command) {
      iiwa_command_ = *command;
      // for ( int i = 0; i < 7; i++) {
      //   cout << "GOT ONE" << iiwa_command_.joint_torque.at(i) << endl;
      // }
    }
    void HandleFeedForwardCommand(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                      const lcmt_iiwa_command* command) {
      iiwa_feedforward_command_ = *command;
    }


    vector<lcmt_iiwa_status> spline_status_list_;
    vector<lcmt_iiwa_command> spline_command_list_;
    vector<lcmt_iiwa_command> spline_ff_command_list_;
    vector<vector<double>> time_list_;

    vector<lcmt_iiwa_status> status_list_;
    vector<lcmt_iiwa_command> command_list_;


    lcmt_iiwa_status iiwa_status_;
    lcmt_iiwa_command iiwa_command_;
    lcmt_iiwa_command iiwa_feedforward_command_;

    lcm::LCM lcm_;
    int kNumJoints_ = 7;
    const RigidBodyTree<double>& tree_;
    bool gotStatusupdate_;
  };



 int do_main(int argc, const char* argv[]) {

   auto tree = std::make_unique<RigidBodyTree<double>>();
   parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
       GetDrakePath() + "/manipulation/models/iiwa_description/urdf/"
           "iiwa14_primitive_collision.urdf",
       multibody::joints::kFixed, tree.get());


   RobotController r(*tree);
   r.generate_spline();

   return 0;
 }

 }  // namespace
 }  // namespace kuka_iiwa_arm
 }  // namespace examples
 }  // namespace drake

 int main(int argc, const char* argv[]) {
   return drake::examples::kuka_iiwa_arm::do_main(argc, argv);
 }
