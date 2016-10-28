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

#ifndef RVIZ_VISUAL_TOOLS_KEY_TOOL_H
#define RVIZ_VISUAL_TOOLS_KEY_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <rviz/tool.h>
#include <rviz/default_plugin/tools/move_tool.h>

#include <QCursor>
#include <QObject>
#endif

namespace rviz_visual_tools
{
//! The Point Tool allows the user to click on a point which
//! gets published as a Joy message.
class KeyTool : public rviz::Tool
{
  Q_OBJECT
public:
  KeyTool();
  virtual ~KeyTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();
  void moveNext();
  void moveAuto();

  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

public Q_SLOTS:

protected:
  ros::NodeHandle nh_;
  ros::Publisher joy_publisher_;
  rviz::MoveTool move_tool_;
};
}

#endif
