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

using namespace std;
using namespace Eigen;
using namespace chrono;

namespace mocap {

double QualisysDriver::deg2rad = M_PI / 180.0;

bool QualisysDriver::init() {
    // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
    // of the workspace options
    declare_parameter("server_address", "192.168.129.216");
    declare_parameter("server_base_port");
    // declare_parameter("model_list");
    declare_parameter("frame_rate");
    declare_parameter("max_accel");
    declare_parameter("publish_tf");
    declare_parameter("fixed_frame_id");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Interrupted while waiting for the parameters client. Exiting.");
            rclcpp::shutdown();
        }
    }

    // server_address = parameters_client->get_parameter<string>("server_address",
    // "192.168.129.216"); base_port      =
    // parameters_client->get_parameter<int>("server_base_port", 22222); model_list     =
    // parameters_client->get_parameter<vector<string>>("model_list"); frame_rate     =
    // parameters_client->get_parameter<int>("frame_rate", 100); max_accel      =
    // parameters_client->get_parameter<double>("max_accel", 10.0); publish_tf     =
    // parameters_client->get_parameter<bool>("publish_tf", true); fixed_frame_id =
    // parameters_client->get_parameter<string>("fixed_frame_id", "mocap");

    /**
     * Hard Coding for now since the parameters arent working for me
     */
    server_address = "127.0.0.1";
    base_port      = 22222;
    model_list     = vector<string>(0);
    frame_rate     = 100;
    max_accel      = 10.0;
    publish_tf     = true;
    fixed_frame_id = "mocap";

    frame_interval = 1.0 / static_cast<double>(frame_rate);
    double& dt     = frame_interval;
    process_noise.topLeftCorner<6, 6>() =
        0.5 * Matrix<double, 6, 6>::Identity() * dt * dt * max_accel;
    process_noise.bottomRightCorner<6, 6>() = Matrix<double, 6, 6>::Identity() * dt * max_accel;
    process_noise *= process_noise; // Make it a covariance
    measurement_noise = Matrix<double, 6, 6>::Identity() * 1e-3;
    measurement_noise *= measurement_noise; // Make it a covariance
    model_set.insert(model_list.begin(), model_list.end());

    // Connecting to the server
    RCLCPP_INFO(get_logger(),
                "Connecting to the Qualisys at: " + server_address + ":" + to_string(base_port));

    if (!port_protocol.Connect((char*)server_address.data(), base_port, 0, 1, 7)) {
        RCLCPP_FATAL(get_logger(),
                     "Could not find the Qualisys at: " + server_address + ":" +
                         to_string(base_port));
        rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Connected to " + server_address + ":" + to_string(base_port));

    // Get 6DOF settings
    port_protocol.Read6DOFSettings();

    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

    auto callback = [this]() -> void { this->run(); };
    timer_        = this->create_wall_timer(duration<double>(frame_interval), callback);
    return true;
}

void QualisysDriver::disconnect() {
    RCLCPP_INFO(get_logger(),
                "Disconnected with the server " + server_address + ":" + to_string(base_port));
    port_protocol.StreamFramesStop();
    port_protocol.Disconnect();
    return;
}

void QualisysDriver::run() {
    prt_packet = port_protocol.GetRTPacket();
    CRTPacket::EPacketType e_type;
    port_protocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

    if (port_protocol.ReceiveRTPacket(e_type, true)) {
        switch (e_type) {
        // Case 1 - sHeader.nType 0 indicates an error
        case CRTPacket::PacketError:
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Error when streaming frames: " << port_protocol.GetRTPacket()->GetErrorString());
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
    int body_count = prt_packet->Get6DOFEulerBodyCount();
    // Assign each subject with a thread
    vector<boost::thread> subject_threads;
    subject_threads.reserve(body_count);

    for (int i = 0; i < body_count; ++i) {
        string subject_name(port_protocol.Get6DOFBodyName(i));

        // Process the subject if required
        if (model_set.empty() || model_set.count(subject_name)) {
            // Create a new subject if it does not exist
            if (subjects.find(subject_name) == subjects.end()) {
                subjects[subject_name] =
                    Subject::SubjectPtr(new Subject(this, subject_name, fixed_frame_id));
                subjects[subject_name]->setParameters(process_noise, measurement_noise, frame_rate);
            }
            // Handle the subject in a different thread
            subject_threads.emplace_back(&QualisysDriver::handleSubject, this, i);
            // handleSubject(i);
        }
    }

    // Wait for all the threads to stop
    for (auto it = subject_threads.begin(); it != subject_threads.end(); ++it) {
        it->join();
    }

    // Send out warnings
    for (auto it = subjects.begin(); it != subjects.end(); ++it) {
        Subject::Status status = it->second->getStatus();
        if (status == Subject::LOST)
            RCLCPP_WARN(get_logger(), "Lose track of subject %s", (it->first).c_str());
        else if (status == Subject::INITIALIZING)
            RCLCPP_WARN(get_logger(), "Initialize subject %s", (it->first).c_str());
    }

    return;
}

void QualisysDriver::handleSubject(const int& sub_idx) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    // Name of the subject
    string subject_name(port_protocol.Get6DOFBodyName(sub_idx));
    // Pose of the subject
    float x, y, z, roll, pitch, yaw;
    prt_packet->Get6DOFEulerBody(sub_idx, x, y, z, roll, pitch, yaw);
    write_lock.unlock();

    // If the subject is lost
    if (isnan(x) || isnan(y) || isnan(z) || isnan(roll) || isnan(pitch) || isnan(yaw)) {
        subjects[subject_name]->disable();
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
    if (subjects[subject_name]->getStatus() == Subject::LOST) {
        subjects[subject_name]->enable();
    }

    // Compute the timestamp
    // double time = ros::Time::now().toSec();
    if (start_time_local_ == time_point<system_clock, duration<double>>()) {
        start_time_local_  = chrono::system_clock::now();
        start_time_packet_ = time_point<system_clock, duration<double>>(
            duration<double>(prt_packet->GetTimeStamp() / 1e6));
    }

    const auto packet_time = time_point<system_clock, duration<double>>(
        duration<double>(prt_packet->GetTimeStamp() / 1e6));
    const auto time = start_time_local_ + (packet_time - start_time_packet_);

    // Feed the new measurement to the subject
    subjects[subject_name]->processNewMeasurement(time.time_since_epoch().count(), m_att, m_pos);

    // Publish tf if requred
    if (publish_tf && subjects[subject_name]->getStatus() == Subject::TRACKED) {
        Quaterniond att = subjects[subject_name]->getAttitude();
        Vector3d    pos = subjects[subject_name]->getPosition();
        // tf2::Quaternion att_tf;
        // tf2::Vector3    pos_tf;

        // tf::quaternionEigenToTF(att, att_tf);
        // tf2::fromMsg(toMsg(att), att_tf);

        // tf::vectorEigenToTF(pos, pos_tf);
        // tf2::fromMsg(toMsg(pos), pos_tf);

        // tf::StampedTransform stamped_transform = tf::StampedTransform(
        //     tf::Transform(att_tf, pos_tf), ros::Time::now(), fixed_frame_id, subject_name);
        geometry_msgs::msg::TransformStamped stamped_transform;
        stamped_transform.header.stamp    = rclcpp::Node::now();
        stamped_transform.header.frame_id = fixed_frame_id;
        stamped_transform.child_frame_id  = "?";

        stamped_transform.transform.translation.x = pos.x();
        stamped_transform.transform.translation.y = pos.y();
        stamped_transform.transform.translation.z = pos.z();

        stamped_transform.transform.rotation.x = att.x();
        stamped_transform.transform.rotation.x = att.y();
        stamped_transform.transform.rotation.x = att.z();
        stamped_transform.transform.rotation.x = att.w();

        // tf2::Stamped<tf2::Transform> stamped_transform(
        //     tf2::Transform(att_tf, pos_tf), system_clock::now(), fixed_frame_id);

        tf2_msgs::msg::TFMessage tfs;
        tfs.transforms.push_back(stamped_transform);

        write_lock.lock();
        tf_publisher_->publish(tfs);
        write_lock.unlock();
    }

    return;
}
} // namespace mocap
