/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
 */

#ifndef TF2_ROS__ASYNC_BUFFER_INTERFACE_H_
#define TF2_ROS__ASYNC_BUFFER_INTERFACE_H_

#include <functional>
#include <future>
#include <string>
#include <utility>

#include "tf2_ros/visibility_control.h"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tf2_ros
{

class TransformStampedFuture : public std::shared_future<geometry_msgs::msg::TransformStamped>
{
  using BaseType = std::shared_future<geometry_msgs::msg::TransformStamped>;

public:
  /// Constructor
  explicit TransformStampedFuture(BaseType && future) noexcept
  : BaseType(std::move(future)) {}

  /// Copy constructor
  TransformStampedFuture(const TransformStampedFuture & ts_future) noexcept
  : BaseType(ts_future),
    handle_(ts_future.handle_) {}

  /// Move constructor
  TransformStampedFuture(TransformStampedFuture && ts_future) noexcept
  : BaseType(std::move(ts_future)),
    handle_(std::move(ts_future.handle_)) {}

  void setHandle(const tf2::TransformableRequestHandle handle)
  {
    handle_ = handle;
  }

  tf2::TransformableRequestHandle getHandle() const
  {
    return handle_;
  }

private:
  tf2::TransformableRequestHandle handle_ {};
};

using TransformReadyCallback = std::function<void (const TransformStampedFuture &)>;

/**
 * \brief Abstract interface for asynchronous operations on a `tf2::BufferCoreInterface`.
 * Implementations include tf2_ros::Buffer.
 */
class AsyncBufferInterface
{
public:
  TF2_ROS_PUBLIC
  virtual
  ~AsyncBufferInterface() = default;


  /**
   * \brief Cancel the future to make sure the callback of requested transform is clean.
   * \param ts_future The future to the requested transform.
   */
  // virtual void
  // cancel(const TransformStampedFuture & ts_future) = 0;
};  // class AsyncBufferInterface

}  // namespace tf2_ros

#endif  // TF2_ROS__ASYNC_BUFFER_INTERFACE_H_
