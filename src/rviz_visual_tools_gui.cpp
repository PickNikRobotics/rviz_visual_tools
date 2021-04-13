// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

#include <cstdio>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_visual_tools/rviz_visual_tools_gui.hpp>

namespace rviz_visual_tools
{
RvizVisualToolsGui::RvizVisualToolsGui(QWidget* parent)
  : rviz_common::Panel(parent), remote_reciever_("rviz_visual_tools_gui")
{
  // Create a push button
  btn_next_ = new QPushButton(this);
  btn_next_->setText("Next");
  connect(btn_next_, SIGNAL(clicked()), this, SLOT(moveNext()));

  // Create a push button
  btn_auto_ = new QPushButton(this);
  btn_auto_->setText("Continue");
  connect(btn_auto_, SIGNAL(clicked()), this, SLOT(moveAuto()));

  // Create a push button
  btn_full_auto_ = new QPushButton(this);
  btn_full_auto_->setText("Break");
  connect(btn_full_auto_, SIGNAL(clicked()), this, SLOT(moveFullAuto()));

  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(moveStop()));

  // Horizontal Layout
  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_next_);
  hlayout1->addWidget(btn_auto_);
  hlayout1->addWidget(btn_full_auto_);
  hlayout1->addWidget(btn_stop_);

  // Verticle layout
  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  setLayout(layout);

  btn_next_->setEnabled(true);
  btn_auto_->setEnabled(true);
  btn_full_auto_->setEnabled(true);
}

void RvizVisualToolsGui::moveNext()
{
  remote_reciever_.publishNext();
}

void RvizVisualToolsGui::moveAuto()
{
  remote_reciever_.publishContinue();
}

void RvizVisualToolsGui::moveFullAuto()
{
  remote_reciever_.publishBreak();
}

void RvizVisualToolsGui::moveStop()
{
  remote_reciever_.publishStop();
}

void RvizVisualToolsGui::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void RvizVisualToolsGui::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}
}  // end namespace rviz_visual_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsGui, rviz_common::Panel)
