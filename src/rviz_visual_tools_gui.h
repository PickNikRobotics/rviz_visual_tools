/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

// TODO(dave): convert to flow layout:
// http://doc.qt.io/qt-5/qtwidgets-layouts-flowlayout-example.html

#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>

#include <rviz_visual_tools/remote_reciever.h>

class QLineEdit;
class QSpinBox;

namespace rviz_visual_tools
{
class RvizVisualToolsGui : public rviz::Panel
{
  Q_OBJECT
public:
  explicit RvizVisualToolsGui(QWidget* parent = nullptr);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

public Q_SLOTS:

protected Q_SLOTS:

  void moveNext();

  void moveAuto();

  void moveFullAuto();

  void moveStop();

protected:
  QPushButton* btn_next_;
  QPushButton* btn_auto_;
  QPushButton* btn_full_auto_;
  QPushButton* btn_stop_;

  RemoteReciever remote_reciever_;
};

}  // end namespace rviz_visual_tools
