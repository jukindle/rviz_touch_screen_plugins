/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz2_touch_screen_plugins/view_controllers/ortho/touch_screen_ortho_view_controller.hpp"

#include <utility>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_common/display_context.hpp"
#include "rviz_rendering/orthographic.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace rviz2_touch_screen_plugins
{
  namespace view_controllers
  {

    static const float ORTHO_VIEW_CONTROLLER_CAMERA_Z = 500;

    TouchScreenTopDownOrtho::TouchScreenTopDownOrtho()
        : dragging_(false)
    {
      scale_property_ = new rviz_common::properties::FloatProperty(
          "Scale", 10, "How much to scale up the size of things in the scene.", this);
      angle_property_ = new rviz_common::properties::FloatProperty(
          "Angle", 0, "Angle around the Z axis to rotate.", this);
      x_property_ = new rviz_common::properties::FloatProperty(
          "X", 0, "X component of camera position.", this);
      y_property_ = new rviz_common::properties::FloatProperty(
          "Y", 0, "Y component of camera position.", this);
      track_ori_property_ = new rviz_common::properties::BoolProperty(
          "Track Orientation", false, "Set if orientation of target frame is tracked.", this);
      controls_width_property_ = new rviz_common::properties::FloatProperty(
          "Control field width", 0.05, "Sets the width of the control fields (relative to width / height of 3D viewer).", this);
      controls_width_property_->setMin(0.0);
      controls_width_property_->setMax(1.0);
      en_move_property_ = new rviz_common::properties::BoolProperty(
          "Enable movement", true, "Set if movement is enabled.", this);
      en_scale_property_ = new rviz_common::properties::BoolProperty(
          "Enable scaling", true, "Set if scaling is enabled.", this);
      en_rot_property_ = new rviz_common::properties::BoolProperty(
          "Enable rotation", true, "Set if rotation is enabled.", this);
      en_reset_property_ = new rviz_common::properties::BoolProperty(
          "Enable reset", true, "Set if reset is enabled.", this);
    }

    void TouchScreenTopDownOrtho::onInitialize()
    {
      FramePositionTrackingViewController::onInitialize();

      camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
      auto camera_parent = getCameraParent(camera_);
      camera_parent->setFixedYawAxis(false);
      invert_z_->hide();
    }

    void TouchScreenTopDownOrtho::reset()
    {
      scale_property_->setFloat(10);
      angle_property_->setFloat(-M_PI_2);
      x_property_->setFloat(0);
      y_property_->setFloat(0);
    }

    void TouchScreenTopDownOrtho::handleMouseEvent(rviz_common::ViewportMouseEvent &event)
    {
      // Get the actual dimensions of the viewport
      int viewport_width = camera_->getViewport()->getActualWidth();
      int viewport_height = camera_->getViewport()->getActualHeight();

      int32_t diff_x = 0;
      int32_t diff_y = 0;

      if (event.type == QEvent::MouseButtonPress)
      {
        dragging_ = true;
      }
      else if (event.type == QEvent::MouseButtonRelease)
      {
        dragging_ = false;
      }
      else if (dragging_ && event.type == QEvent::MouseMove)
      {
        diff_x = event.x - event.last_x;
        diff_y = event.y - event.last_y;
        renderOnMove();
      }

      // Get configured control fields width
      double cfw = controls_width_property_->getFloat();

      // Top right corner click causes reset
      if (en_reset_property_->getBool() && event.type == QEvent::MouseButtonPress && event.left() && event.x >= (1.0 - cfw) * viewport_width && event.y <= cfw * viewport_height)
      {
        x_property_->setFloat(0);
        y_property_->setFloat(0);
      }
      // Right side of screen used to scroll
      else if (en_scale_property_->getBool() && event.left() && event.x >= (1.0 - cfw) * viewport_width)
      {
        setCursor(Zoom);
        scale_property_->multiply(1.0f - diff_y * 0.01f);
      }
      // Top side of screen used to rotate
      else if (en_rot_property_->getBool() && event.left() && event.y <= cfw * viewport_height)
      {
        setCursor(Rotate2D);
        angle_property_->add(diff_x * 0.005f);
        orientCamera();
      }

      // Rest of screen used to move
      else if (en_move_property_->getBool())
      {
        setCursor(MoveXY);
        float scale = scale_property_->getFloat();
        move(-diff_x / scale, diff_y / scale);
      }
      else
      {
        setCursor(Default);
      }
    }

    void TouchScreenTopDownOrtho::renderOnMove()
    {
      context_->queueRender();
      emitConfigChanged();
    }

    void TouchScreenTopDownOrtho::orientCamera()
    {
      auto camera_parent = getCameraParent(camera_);
      camera_parent->setOrientation(
          Ogre::Quaternion(Ogre::Radian(angle_property_->getFloat()), Ogre::Vector3::UNIT_Z));
    }

    void TouchScreenTopDownOrtho::mimic(ViewController *source_view)
    {
      FramePositionTrackingViewController::mimic(source_view);

      if (source_view->getClassId() == "rviz2_touch_screen_plugins/TouchScreenTopDownOrtho")
      {
        auto source_ortho = qobject_cast<TouchScreenTopDownOrtho *>(source_view);
        scale_property_->setFloat(source_ortho->scale_property_->getFloat());
        angle_property_->setFloat(source_ortho->angle_property_->getFloat());
        x_property_->setFloat(source_ortho->x_property_->getFloat());
        y_property_->setFloat(source_ortho->y_property_->getFloat());
      }
      else if (source_view->getFocalPointStatus().exists_)
      {
        setPosition(source_view->getFocalPointStatus().value_);
      }
      else
      {
        // if the previous view does not have a focal point and is not the same as this, the camera is
        // placed at (x, y, ORTHO_VIEW_CONTROLLER_CAMERA_Z), where x and y are first two coordinates of
        // the old camera position.
        auto source_camera_parent = getCameraParent(source_view->getCamera());
        setPosition(source_camera_parent->getPosition());
      }
    }

    void TouchScreenTopDownOrtho::update(float dt, float ros_dt)
    {
      FramePositionTrackingViewController::update(dt, ros_dt);

      if (track_ori_property_->getBool())
      {
        target_scene_node_->setOrientation(reference_orientation_);
      }

      updateCamera();
    }

    void TouchScreenTopDownOrtho::lookAt(const Ogre::Vector3 &point)
    {
      setPosition(point - target_scene_node_->getPosition());
    }

    void TouchScreenTopDownOrtho::onTargetFrameChanged(
        const Ogre::Vector3 &old_reference_position, const Ogre::Quaternion &old_reference_orientation)
    {
      (void)old_reference_orientation;

      move(
          old_reference_position.x - reference_position_.x,
          old_reference_position.y - reference_position_.y);
    }

    void TouchScreenTopDownOrtho::updateCamera()
    {
      orientCamera();

      float width = camera_->getViewport()->getActualWidth();
      float height = camera_->getViewport()->getActualHeight();

      float scale = scale_property_->getFloat();
      float ortho_width = width / scale / 2;
      float ortho_height = height / scale / 2;
      Ogre::Matrix4 projection = rviz_rendering::buildScaledOrthoMatrix(
          -ortho_width, ortho_width, -ortho_height, ortho_height,
          camera_->getNearClipDistance(), camera_->getFarClipDistance());
      camera_->setCustomProjectionMatrix(true, projection);

      // For Z, we use a value that seems to work very well in the past. It once was connected to
      // half the far_clip_distance.
      auto camera_parent = getCameraParent(camera_);
      camera_parent->setPosition(
          Ogre::Vector3(
              x_property_->getFloat(), y_property_->getFloat(), ORTHO_VIEW_CONTROLLER_CAMERA_Z));
    }

    Ogre::SceneNode *TouchScreenTopDownOrtho::getCameraParent(Ogre::Camera *camera)
    {
      auto camera_parent = camera->getParentSceneNode();

      if (!camera_parent)
      {
        throw std::runtime_error("camera's parent scene node pointer unexpectedly nullptr");
      }
      return camera_parent;
    }

    void TouchScreenTopDownOrtho::setPosition(const Ogre::Vector3 &pos_rel_target)
    {
      x_property_->setFloat(pos_rel_target.x);
      y_property_->setFloat(pos_rel_target.y);
    }

    void TouchScreenTopDownOrtho::move(float dx, float dy)
    {
      float angle = angle_property_->getFloat();
      x_property_->add(dx * cos(angle) - dy * sin(angle));
      y_property_->add(dx * sin(angle) + dy * cos(angle));
    }

  } // namespace view_controllers
} // namespace rviz2_touch_screen_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(
    rviz2_touch_screen_plugins::view_controllers::TouchScreenTopDownOrtho,
    rviz_common::ViewController)