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

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/ogre_helpers/ogre_vector.h>
#include <OgreViewport.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/orthographic.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/viewport_mouse_event.h>

#include "rviz_touch_screen_plugins/view_controllers/ortho/touch_screen_ortho_view_controller.hpp"

namespace rviz_touch_screen_plugins
{
  namespace view_controllers
  {
    TouchScreenTopDownOrtho::TouchScreenTopDownOrtho() : dragging_(false)
    {
      scale_property_ =
          new rviz::FloatProperty("Scale", 10, "How much to scale up the size of things in the scene.", this);
      angle_property_ = new rviz::FloatProperty("Angle", 0, "Angle around the Z axis to rotate.", this);
      x_property_ = new rviz::FloatProperty("X", 0, "X component of camera position.", this);
      y_property_ = new rviz::FloatProperty("Y", 0, "Y component of camera position.", this);
      track_ori_property_ = new rviz::BoolProperty(
          "Track Orientation", false, "Set if orientation of target frame is tracked.", this);
      controls_width_property_ = new rviz::FloatProperty(
          "Control field width", 0.05, "Sets the width of the control fields (relative to width / height of 3D viewer).", this);
      controls_width_property_->setMin(0.0);
      controls_width_property_->setMax(1.0);
      en_move_property_ = new rviz::BoolProperty(
          "Enable movement", true, "Set if movement is enabled.", this);
      en_scale_property_ = new rviz::BoolProperty(
          "Enable scaling", true, "Set if scaling is enabled.", this);
      en_rot_property_ = new rviz::BoolProperty(
          "Enable rotation", true, "Set if rotation is enabled.", this);
      en_reset_property_ = new rviz::BoolProperty(
          "Enable reset", true, "Set if reset is enabled.", this);
    }

    TouchScreenTopDownOrtho::~TouchScreenTopDownOrtho()
    {
    }

    void TouchScreenTopDownOrtho::onInitialize()
    {
      FramePositionTrackingViewController::onInitialize();

      camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
      camera_->setFixedYawAxis(false);
      invert_z_->hide();
    }

    void TouchScreenTopDownOrtho::reset()
    {
      scale_property_->setFloat(10);
      angle_property_->setFloat(0);
      x_property_->setFloat(0);
      y_property_->setFloat(0);
    }

    void TouchScreenTopDownOrtho::handleMouseEvent(rviz::ViewportMouseEvent &event)
    {
      // Get the actual dimensions of the viewport
      int viewport_width = camera_->getViewport()->getActualWidth();
      int viewport_height = camera_->getViewport()->getActualHeight();

      bool moved = false;

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
        moved = true;
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

      if (moved)
      {
        context_->queueRender();
        emitConfigChanged();
      }
    }

    void TouchScreenTopDownOrtho::orientCamera()
    {
      camera_->setOrientation(
          Ogre::Quaternion(Ogre::Radian(angle_property_->getFloat()), Ogre::Vector3::UNIT_Z));
    }

    void TouchScreenTopDownOrtho::mimic(ViewController *source_view)
    {
      FramePositionTrackingViewController::mimic(source_view);

      if (TouchScreenTopDownOrtho *source_ortho =
              qobject_cast<TouchScreenTopDownOrtho *>(source_view))
      {
        scale_property_->setFloat(source_ortho->scale_property_->getFloat());
        angle_property_->setFloat(source_ortho->angle_property_->getFloat());
        x_property_->setFloat(source_ortho->x_property_->getFloat());
        y_property_->setFloat(source_ortho->y_property_->getFloat());
      }
      else
      {
        Ogre::Camera *source_camera = source_view->getCamera();
        setPosition(source_camera->getPosition());
      }
    }

    void TouchScreenTopDownOrtho::update(float dt, float ros_dt)
    {
      FramePositionTrackingViewController::update(dt, ros_dt);
      updateCamera();
    }

    void TouchScreenTopDownOrtho::lookAt(const Ogre::Vector3 &point)
    {
      setPosition(point - target_scene_node_->getPosition());
    }

    void TouchScreenTopDownOrtho::onTargetFrameChanged(
        const Ogre::Vector3 &old_reference_position,
        const Ogre::Quaternion & /*old_reference_orientation*/)
    {
      move(old_reference_position.x - reference_position_.x,
           old_reference_position.y - reference_position_.y);
    }

    void TouchScreenTopDownOrtho::updateCamera()
    {
      orientCamera();

      float width = camera_->getViewport()->getActualWidth();
      float height = camera_->getViewport()->getActualHeight();

      float scale = scale_property_->getFloat();
      Ogre::Matrix4 proj;
      rviz::buildScaledOrthoMatrix(proj, -width / scale / 2, width / scale / 2, -height / scale / 2,
                             height / scale / 2, camera_->getNearClipDistance(),
                             camera_->getFarClipDistance());
      camera_->setCustomProjectionMatrix(true, proj);

      // For Z, we use half of the far-clip distance set in
      // selection_context.cpp, so that the shader program which computes
      // depth can see equal distances above and below the Z=0 plane.
      camera_->setPosition(x_property_->getFloat(), y_property_->getFloat(), 500);
    }

    void TouchScreenTopDownOrtho::setPosition(const Ogre::Vector3 &pos_rel_target)
    {
      x_property_->setFloat(pos_rel_target.x);
      y_property_->setFloat(pos_rel_target.y);
    }

    void TouchScreenTopDownOrtho::move(float dx, float dy)
    {
      float angle = angle_property_->getFloat();
      x_property_->add(dx * std::cos(angle) - dy * std::sin(angle));
      y_property_->add(dx * std::sin(angle) + dy * std::cos(angle));
    }

  }
}
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rviz_touch_screen_plugins::view_controllers::TouchScreenTopDownOrtho,
    rviz::ViewController)