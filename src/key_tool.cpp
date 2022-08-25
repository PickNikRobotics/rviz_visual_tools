/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include <OgreRay.h>
#include <OgreVector3.h>

// Rviz
#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>

// this package
#include "key_tool.h"

// C++
#include <sstream>

namespace rviz_visual_tools
{
KeyTool::KeyTool() = default;

KeyTool::~KeyTool() = default;

void KeyTool::onInitialize()
{
  move_tool_.initialize(context_);
}

void KeyTool::activate()
{
}

void KeyTool::deactivate()
{
}

int KeyTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  // move forward / backward
  switch (event->key())
  {
    case Qt::Key_N:
      remote_reciever_.publishNext();
      return 1;
    case Qt::Key_A:
    case Qt::Key_C:
      remote_reciever_.publishContinue();
      return 1;
    case Qt::Key_B:
      remote_reciever_.publishBreak();
      return 1;
    case Qt::Key_S:
      remote_reciever_.publishStop();
      return 1;
  }

  return move_tool_.processKeyEvent(event, panel);
}

int KeyTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  move_tool_.processMouseEvent(event);
  setCursor(move_tool_.getCursor());

  return flags;
}

}  // namespace rviz_visual_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::KeyTool, rviz::Tool)
