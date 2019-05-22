/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: David Gossow
 *//*
 * single_client.h
 *
 *  Created on: Jul 17, 2012
 *      Author: gossow
 */

#ifndef SINGLE_CLIENT_H_
#define SINGLE_CLIENT_H_

#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>

//#include <tf/tf.h>
#include <tf2_ros/buffer.h>

#include <deque>

#include "message_context.h"
#include "state_machine.h"
#include "../interactive_marker_client.h"


namespace interactive_markers
{

class SingleClient
{
public:

  SingleClient(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& server_id,
      tf2_ros::Buffer& tf,
      const std::string& target_frame,
      const InteractiveMarkerClient::CbCollection& callbacks );

  ~SingleClient();

  // Process message from the update channel
  void process(visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr msg, bool enable_autocomplete_transparency = true);

  // Process message from the init channel
  void process(visualization_msgs::msg::InteractiveMarkerInit::ConstSharedPtr msg, bool enable_autocomplete_transparency = true);

  // true if INIT messages are not needed anymore
  bool isInitialized();

  // transform all messages with missing transforms
  void update();

private:

  std::shared_ptr<rclcpp::Node> node_;

  // check if we can go from init state to normal operation
  void checkInitFinished();

  void checkKeepAlive();

  enum StateT
  {
    INIT,
    RECEIVING,
    TF_ERROR
  };

  StateMachine<StateT> state_;

  // updateTf implementation (for one queue)
  void transformInitMsgs( );
  void transformUpdateMsgs( );

  void pushUpdates();

  void errorReset( std::string error_msg );

  // sequence number and time of first ever received update
  uint64_t first_update_seq_num_;

  // sequence number and time of last received update
  uint64_t last_update_seq_num_;
  rclcpp::Time last_update_time_;

  // true if the last outgoing update is too long ago
  // and we've already sent a notification of that
  bool update_time_ok_;

  using UpdateMessageContext = MessageContext<visualization_msgs::msg::InteractiveMarkerUpdate>;
  using InitMessageContext = MessageContext<visualization_msgs::msg::InteractiveMarkerInit>;

  // Queue of Updates waiting for tf and numbering
  using M_UpdateMessageContext = std::deque< UpdateMessageContext >;
  using M_InitMessageContext = std::deque< InitMessageContext >;

  // queue for update messages
  M_UpdateMessageContext update_queue_;

  // queue for init messages
  M_InitMessageContext init_queue_;

  tf2_ros::Buffer& tf_;
  std::string target_frame_;

  const InteractiveMarkerClient::CbCollection& callbacks_;

  std::string server_id_;

  bool warn_keepalive_;
};

}

#endif /* SINGLE_CLIENT_H_ */
