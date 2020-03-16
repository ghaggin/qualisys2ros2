/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <cmath>

#include <mocap_qualisys/QualisysDriver.h>

#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace chrono;

namespace mocap {

double QualisysDriver::deg2rad = M_PI / 180.0;

bool QualisysDriver::init() {
    // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
    // of the workspace options
    declare_parameter("server_address_", "192.168.129.216");
    declare_parameter("server_base_port");
    // declare_parameter("model_list_");
    declare_parameter("frame_rate_");
    declare_parameter("max_accel");
    declare_parameter("publish_tf_");
    declare_parameter("fixed_frame_id_");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Interrupted while waiting for the parameters client. Exiting.");
            rclcpp::shutdown();
        }
    }

    // server_address_ = parameters_client->get_parameter<string>("server_address_",
    // "192.168.129.216"); base_port      =
    // parameters_client->get_parameter<int>("server_base_port", 22222); model_list_     =
    // parameters_client->get_parameter<vector<string>>("model_list_"); frame_rate_     =
    // parameters_client->get_parameter<int>("frame_rate_", 100); max_accel      =
    // parameters_client->get_parameter<double>("max_accel", 10.0); publish_tf_     =
    // parameters_client->get_parameter<bool>("publish_tf_", true); fixed_frame_id_ =
    // parameters_client->get_parameter<string>("fixed_frame_id_", "mocap");

    /**
     * Hard Coding for now since the parameters arent working for me
     */
    server_address_ = "127.0.0.1";
    base_port_      = 22222;
    model_list_     = vector<string>(0);
    model_list_.push_back("Quad1");
    frame_rate_     = 100;
    max_accel_      = 10.0;
    publish_tf_     = true;
    fixed_frame_id_ = "mocap";

    frame_interval_ = 1.0 / static_cast<double>(frame_rate_);
    double& dt      = frame_interval_;
    process_noise_.topLeftCorner<6, 6>() =
        0.5 * Matrix<double, 6, 6>::Identity() * dt * dt * max_accel_;
    process_noise_.bottomRightCorner<6, 6>() = Matrix<double, 6, 6>::Identity() * dt * max_accel_;
    process_noise_ *= process_noise_; // Make it a covariance
    measurement_noise_ = Matrix<double, 6, 6>::Identity() * 1e-3;
    measurement_noise_ *= measurement_noise_; // Make it a covariance
    model_set_.insert(model_list_.begin(), model_list_.end());

    // Connecting to the server
    RCLCPP_INFO(get_logger(),
                "Connecting to the Qualisys at: " + server_address_ + ":" + to_string(base_port_));

    if (!port_protocol_.Connect((char*)server_address_.data(), base_port_, 0, 1, 7)) {
        RCLCPP_FATAL(get_logger(),
                     "Could not find the Qualisys at: " + server_address_ + ":" +
                         to_string(base_port_));
        rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Connected to " + server_address_ + ":" + to_string(base_port_));

    // Get 6DOF settings
    port_protocol_.Read6DOFSettings();

    auto callback = [this]() -> void { this->run(); };
    timer_        = this->create_wall_timer(duration<double>(frame_interval_), callback);
    return true;
}

void QualisysDriver::disconnect() {
    RCLCPP_INFO(get_logger(),
                "Disconnected with the server " + server_address_ + ":" + to_string(base_port_));
    port_protocol_.StreamFramesStop();
    port_protocol_.Disconnect();
    return;
}

void QualisysDriver::run() {
    prt_packet_ = port_protocol_.GetRTPacket();
    CRTPacket::EPacketType e_type;
    port_protocol_.GetCurrentFrame(CRTProtocol::Component6dEuler);

    if (port_protocol_.ReceiveRTPacket(e_type, true)) {
        switch (e_type) {
        // Case 1 - sHeader.nType 0 indicates an error
        case CRTPacket::PacketError:
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Error when streaming frames: " << port_protocol_.GetRTPacket()->GetErrorString());
            break;

        // Case 2 - No more data
        case CRTPacket::PacketNoMoreData:
            RCLCPP_WARN_STREAM(get_logger(), "No more data");
            break;

        // Case 3 - Data received
        case CRTPacket::PacketData:
            handleFrame();
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Unknown CRTPacket case");
            break;
        }
    }

    return;
}

void QualisysDriver::handleFrame() {
    // Number of rigid bodies
    int body_count = prt_packet_->Get6DOFEulerBodyCount();
    // Assign each subject with a thread
    vector<boost::thread> subject_threads;
    subject_threads.reserve(body_count);

    for (int i = 0; i < body_count; ++i) {
        string subject_name(port_protocol_.Get6DOFBodyName(i));

        // Process the subject if required
        if (model_set_.empty() || model_set_.count(subject_name)) {
            // Create a new subject if it does not exist
            if (subjects_.find(subject_name) == subjects_.end()) {
                subjects_[subject_name] =
                    Subject::SubjectPtr(new Subject(this, subject_name, fixed_frame_id_));
                subjects_[subject_name]->setParameters(
                    process_noise_, measurement_noise_, frame_rate_);
            }
            // Handle the subject in a different thread by running handleSubject(i);
            subject_threads.emplace_back(&QualisysDriver::handleSubject, this, i);
        }
    }

    // Wait for all the threads to stop
    for (auto it = subject_threads.begin(); it != subject_threads.end(); ++it) {
        it->join();
    }

    // Send out warnings
    for (auto it = subjects_.begin(); it != subjects_.end(); ++it) {
        Subject::Status status = it->second->getStatus();
        if (status == Subject::LOST)
            RCLCPP_WARN(get_logger(), "Lose track of subject %s", (it->first).c_str());
        else if (status == Subject::INITIALIZING)
            RCLCPP_WARN(get_logger(), "Initialize subject %s", (it->first).c_str());
    }

    return;
}

void QualisysDriver::handleSubject(const int& sub_idx) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx_);

    // Name of the subject
    string subject_name(port_protocol_.Get6DOFBodyName(sub_idx));

    // Pose of the subject
    float x, y, z, roll, pitch, yaw;
    prt_packet_->Get6DOFEulerBody(sub_idx, x, y, z, roll, pitch, yaw);
    write_lock.unlock();

    // If the subject is lost
    if (isnan(x) || isnan(y) || isnan(z) || isnan(roll) || isnan(pitch) || isnan(yaw)) {
        subjects_[subject_name]->disable();
        return;
    }

    // Qualisys sometimes flips 180 degrees around the x axis
    // if(roll > 90)
    //  roll -= 180;
    // else if(roll < -90)
    //  roll += 180;

    // Convert the msgs to Eigen type
    Eigen::Quaterniond m_att;

    tf2::Quaternion q;
    q.setEuler(yaw * deg2rad, pitch * deg2rad, roll * deg2rad);
    m_att.x() = q.getX();
    m_att.y() = q.getY();
    m_att.z() = q.getZ();
    m_att.w() = q.getW();

    // Convert mm to m
    Eigen::Vector3d m_pos(x / 1000.0, y / 1000.0, z / 1000.0);
    // Re-enable the object if it is lost previously
    if (subjects_[subject_name]->getStatus() == Subject::LOST) {
        subjects_[subject_name]->enable();
    }

    // Compute the timestamp
    // double time = ros::Time::now().toSec();
    if (start_time_local_ == time_point<system_clock, duration<double>>()) {
        start_time_local_  = chrono::system_clock::now();
        start_time_packet_ = time_point<system_clock, duration<double>>(
            duration<double>(prt_packet_->GetTimeStamp() / 1e6));
    }

    const auto packet_time = time_point<system_clock, duration<double>>(
        duration<double>(prt_packet_->GetTimeStamp() / 1e6));
    const auto time = start_time_local_ + (packet_time - start_time_packet_);

    // Feed the new measurement to the subject
    subjects_[subject_name]->processNewMeasurement(time.time_since_epoch().count(), m_att, m_pos);

    // Publish tf if requred
    if (publish_tf_ && subjects_[subject_name]->getStatus() == Subject::TRACKED) {
        Quaterniond att = subjects_[subject_name]->getAttitude();
        Vector3d    pos = subjects_[subject_name]->getPosition();

        // tf2::Quaternion att_tf;
        // tf2::Vector3    pos_tf;

        // tf::quaternionEigenToTF(att, att_tf);
        // tf2::fromMsg(toMsg(att), att_tf);

        // tf::vectorEigenToTF(pos, pos_tf);
        // tf2::fromMsg(toMsg(pos), pos_tf);

        // tf::StampedTransform stamped_transform = tf::StampedTransform(
        //     tf::Transform(att_tf, pos_tf), ros::Time::now(), fixed_frame_id_, subject_name);
        geometry_msgs::msg::TransformStamped stamped_transform;
        stamped_transform.header.stamp    = rclcpp::Node::now();
        stamped_transform.header.frame_id = fixed_frame_id_;
        stamped_transform.child_frame_id  = subject_name;

        stamped_transform.transform.translation.x = pos.x();
        stamped_transform.transform.translation.y = pos.y();
        stamped_transform.transform.translation.z = pos.z();

        stamped_transform.transform.rotation.x = att.x();
        stamped_transform.transform.rotation.x = att.y();
        stamped_transform.transform.rotation.x = att.z();
        stamped_transform.transform.rotation.x = att.w();

        write_lock.lock();
        tf_broadcaster_->sendTransform(stamped_transform);
        write_lock.unlock();
    }

    return;
}
} // namespace mocap
