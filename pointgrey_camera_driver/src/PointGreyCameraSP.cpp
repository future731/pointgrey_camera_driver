/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie
Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution
Unlimited).
This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*-*-C++-*-*/
/**
   @file PointGreyCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "pointgrey_camera_driver/PointGreyCameraSpinnaker.h"

#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <limits>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

// Once Exposure End Event is detected, the OnDeviceEvent function will be called
class DeviceEventHandler : public DeviceEvent
{
public:
  DeviceEventHandler(PointGreyCameraSP* sp_ptr)
  {
    ptr_ = sp_ptr;
  };
  ~DeviceEventHandler()
  {
    ptr_ = 0;
  };

  // this is callback for each image capture process
  void OnDeviceEvent(GenICam::gcstring eventName)
  {
    // When this event is not for capturing an image
    if (GetDeviceEventId() != 40003)
    {
      std::cerr << "Got Device Event with " << eventName << " and ID=" << GetDeviceEventId() << std::endl;
    }

    ros::Time sys_tm = ros::Time::now();
    if (!!ptr_)
    {
      ptr_->setTime(sys_tm);
    }
  }

private:
  PointGreyCameraSP* ptr_;
};

void PointGreyCameraSP::setTime(ros::Time& sys_tm)
{
  sys_tm_ = sys_tm;
}

PointGreyCameraSP::PointGreyCameraSP()
{
  serial_ = 0;
  captureRunning_ = false;
  cam_ptr_ = 0;
  is_est_tm_initialized_ = false;
  is_cam_tm_initialized_ = false;
}

PointGreyCameraSP::~PointGreyCameraSP()
{
  system_->ReleaseInstance();
  captureRunning_ = false;
  cam_ptr_ = 0;
  is_est_tm_initialized_ = false;
  is_cam_tm_initialized_ = false;
}

// {{{ setFrameRate
void PointGreyCameraSP::setFrameRate(double& value)
{
  INodeMap& nodeMap = cam_ptr_->GetNodeMap();

  try
  {
    double setrate = value;
#if 0
    NodeList_t entries;
    nodeMap.GetNodes(entries);
    for(int i = 0; i < entries.size(); i++) {
      std::cerr << i << ": " << entries.at(i)->GetName(true) << std::endl;
    }
#endif
    CFloatPtr ptrFrameRateCurr = nodeMap.GetNode("AcquisitionResultingFrameRate");
    if (!IsAvailable(ptrFrameRateCurr) || !IsReadable(ptrFrameRateCurr))
    {
      std::cerr << "Unable to read actual frame rate. Aborting..." << std::endl;
      return;
    }
    value = ptrFrameRateCurr->GetValue();
    std::cerr << "Measured FrameRate: " << value << std::endl;

    CIntegerPtr ptrThroughLimit = nodeMap.GetNode("DeviceLinkThroughputLimit");
    if (!IsAvailable(ptrThroughLimit) || !IsWritable(ptrThroughLimit))
    {
      std::cerr << "Unable to set throughput limit. Aborting..." << std::endl;
      return;
    }
    int pmax = ptrThroughLimit->GetMax();
    std::cerr << "pmax: " << pmax << std::endl;
    int p = ptrThroughLimit->GetValue();
    std::cerr << "p: " << p << std::endl;

    CBooleanPtr ptrFrameRateEnable = nodeMap.GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(ptrFrameRateEnable) || !IsWritable(ptrFrameRateEnable))
    {
      std::cerr << "Unable to set frame rate enable. Aborting..." << std::endl;
      return;
    }
    ptrFrameRateEnable->SetValue(true);

    CFloatPtr ptrFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
    if (!IsAvailable(ptrFrameRate) || !IsWritable(ptrFrameRate))
    {
      std::cerr << "Unable to set frame rate. Aborting..." << std::endl;
      return;
    }
    const double maxrate = ptrFrameRate->GetMax();
    std::cerr << "Hardware MaxFrameRate: " << maxrate << std::endl;
    if (setrate > maxrate)
    {
      setrate = maxrate;
    }
    ptrFrameRate->SetValue(setrate);
    value = setrate;
    std::cerr << "FrameRate set to " << ptrFrameRate->GetValue() << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cerr << "Error(FrameRate): " << e.what() << std::endl;
  }
}
// }}}

// {{{ setExposure
void PointGreyCameraSP::setExposure(bool& autoset, double& value)
{
  INodeMap& nodeMap = cam_ptr_->GetNodeMap();

  try
  {
    if (autoset)
    {
      CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
      if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
      {
        std::cerr << "Unable to enable automatic exposure (node retrieval). Aborting..." << std::endl;
        return;
      }
      CEnumEntryPtr ptrExposureAutoOn = ptrExposureAuto->GetEntryByName("Continuous");
      if (!IsAvailable(ptrExposureAutoOn) || !IsReadable(ptrExposureAutoOn))
      {
        std::cerr << "Unable to enable automatic exposure (enum entry retrieval). Aborting..." << std::endl;
        return;
      }
      ptrExposureAuto->SetIntValue(ptrExposureAutoOn->GetValue());

      CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
      if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime))
      {
        std::cerr << "Unable to read exposure time. Aborting..." << std::endl;
        return;
      }
      double val = ptrExposureTime->GetValue();
      value = val / 1000000.0;
    }
    else
    {
      CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
      if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
      {
        std::cerr << "Unable to disable automatic exposure (node retrieval). Aborting..." << std::endl;
        return;
      }
      CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
      if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
      {
        std::cerr << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << std::endl;
        return;
      }
      ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

      CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
      if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
      {
        std::cerr << "Unable to set exposure time. Aborting..." << std::endl;
        return;
      }
      // Ensure desired exposure time does not exceed the maximum
      const double exposureTimeMax = ptrExposureTime->GetMax();
      // double exposureTimeToSet = 1.0;
      double exposureTimeToSet = value * 1000000.0;
      if (exposureTimeToSet > exposureTimeMax)
      {
        exposureTimeToSet = exposureTimeMax;
      }
      value = exposureTimeToSet / 1000000.0;

      ptrExposureTime->SetValue(exposureTimeToSet);
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cerr << "Error(Exposure): " << e.what() << std::endl;
  }
}
// }}}

// {{{ setGamma
void PointGreyCameraSP::setGamma(double& value)
{
  INodeMap& nodeMap = cam_ptr_->GetNodeMap();

  // TODO gamma is always enabled; implementation is not enough
  try
  {
    CBooleanPtr ptrGammaEnable = nodeMap.GetNode("GammaEnable");
    if (!IsAvailable(ptrGammaEnable) || !IsWritable(ptrGammaEnable))
    {
      std::cerr << "Unable to disable gamma (node retrieval). Aborting..." << std::endl;
      return;
    }
    ptrGammaEnable->SetValue(true);

    CFloatPtr ptrGammaValue = nodeMap.GetNode("Gamma");
    if (!IsAvailable(ptrGammaValue) || !IsWritable(ptrGammaValue))
    {
      std::cerr << "Unable to set gamma time. Aborting..." << std::endl;
      return;
    }
    const double maxgamma = ptrGammaValue->GetMax();
    double setgamma = value;
    if (setgamma > maxgamma)
    {
      setgamma = maxgamma;
    }
    ptrGammaValue->SetValue(setgamma);
    value = setgamma;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cerr << "Error(Gamma): " << e.what() << std::endl;
  }
}
// }}}

// {{{ setGain
void PointGreyCameraSP::setGain(bool& autoset, double& value)
{
  INodeMap& nodeMap = cam_ptr_->GetNodeMap();

  try
  {
    if (autoset)
    {
      CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
      if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
      {
        std::cerr << "Unable to enable automatic gain (node retrieval). Aborting..." << std::endl;
        return;
      }
      // NodeList_t entries;
      // ptrGainAuto->GetEntries(entries);
      // for(int i = 0; i < entries.size(); i++) {
      // std::cerr << "i: " << entries.at(i)->GetName() << std::endl;
      //}
      CEnumEntryPtr ptrGainAutoOn = ptrGainAuto->GetEntryByName("Continuous");
      if (!IsAvailable(ptrGainAutoOn) || !IsReadable(ptrGainAutoOn))
      {
        std::cerr << "Unable to enable automatic gain (enum entry retrieval). Aborting..." << std::endl;
        return;
      }
      ptrGainAuto->SetIntValue(ptrGainAutoOn->GetValue());

      CFloatPtr ptrGainValue = nodeMap.GetNode("Gain");
      if (!IsAvailable(ptrGainValue) || !IsReadable(ptrGainValue))
      {
        std::cerr << "Unable to read gain. Aborting..." << std::endl;
        return;
      }
      double val = ptrGainValue->GetValue();
      value = val;
    }
    else
    {
      CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
      if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
      {
        std::cerr << "Unable to disable automatic gain (node retrieval). Aborting..." << std::endl;
        return;
      }
      CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
      if (!IsAvailable(ptrGainAutoOff) || !IsReadable(ptrGainAutoOff))
      {
        std::cerr << "Unable to disable automatic gain (enum entry retrieval). Aborting..." << std::endl;
        return;
      }
      ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());

      CFloatPtr ptrGainValue = nodeMap.GetNode("Gain");
      if (!IsAvailable(ptrGainValue) || !IsWritable(ptrGainValue))
      {
        std::cerr << "Unable to set gain time. Aborting..." << std::endl;
        return;
      }
      const double maxgain = ptrGainValue->GetMax();
      double setgain = value;
      if (setgain > maxgain)
      {
        setgain = maxgain;
      }
      ptrGainValue->SetValue(setgain);
      value = setgain;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cerr << "Error(Gain): " << e.what() << std::endl;
  }
}
// }}}

// {{{ setExternalTrigger
void PointGreyCameraSP::setExternalTrigger(bool& enable, std::string& source, int& trigger_polarity)
{
  INodeMap& nodeMap = cam_ptr_->GetNodeMap();

  try
  {
    bool is_hardware;
    // TODO implement GPIO1, 2, 3
    if (source == "GPIO0")
    {
      is_hardware = true;
    }
    else if (source == "GPIO1")
    {
      is_hardware = false;
    }
    else if (source == "GPIO2")
    {
      is_hardware = false;
    }
    else if (source == "GPIO3")
    {
      is_hardware = false;
    }
    else
    {
      std::cerr << "Unable to set External Trigger Mode. Choose GPIO pin" << std::endl;
      std::cerr << source << std::endl;
      return;
    }
    bool is_falling_edge;
    if (trigger_polarity == 0)
    {
      is_falling_edge = true;
    }
    else if (trigger_polarity == 1)
    {
      is_falling_edge = false;
    }
    else
    {
      std::cerr << "Unable to set External Trigger Polarity. Choose 0(Low) or 1(High)" << std::endl;
      return;
    }
    // Ensure trigger mode is off
    CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
    if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
    {
      std::cerr << "Unable to disable trigger mode (node retrieval). Aborting..." << std::endl;
      return;
    }

    CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
    if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
    {
      std::cerr << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << std::endl;
      return;
    }
    ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
    if (not enable)
    {
      return;
    }
    CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
    if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
    {
      std::cerr << "Unable to set trigger mode (node retrieval). Aborting..." << std::endl;
      return;
    }
    if (is_hardware)
    {
      // Set trigger mode to hardware ('Line0')
      // TODO: There are other possibilities of Trigger Line other than Line0
      CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line0");
      if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
      {
        std::cerr << "Unable to set trigger mode (enum entry retrieval). Aborting..." << std::endl;
        return;
      }

      ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());
      std::cerr << "Trigger Mode set to Hardware(GPIO0). Mode is " << ptrTriggerSource->GetIntValue() << std::endl;
    }
    else
    {
      // Set trigger mode to software
      CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
      if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
      {
        std::cerr << "Unable to set trigger mode (enum entry retrieval). Aborting..." << std::endl;
        return;
      }

      ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
      std::cerr << "Trigger Mode set to Software. Mode is " << ptrTriggerSource->GetIntValue() << std::endl;
    }
    if (is_falling_edge)
    {
      // Set trigger mode to falling edge
      CEnumerationPtr ptrTriggerEdge = nodeMap.GetNode("TriggerActivation");
      CEnumEntryPtr ptrTriggerFallingEdge = ptrTriggerEdge->GetEntryByName("FallingEdge");
      if (!IsAvailable(ptrTriggerFallingEdge) || !IsReadable(ptrTriggerFallingEdge))
      {
        std::cerr << "Unable to set trigger edge (enum entry retrieval). Aborting..." << std::endl;
        return;
      }

      ptrTriggerEdge->SetIntValue(ptrTriggerFallingEdge->GetValue());
    }
    else
    {
      // Set trigger mode to rising edge
      CEnumerationPtr ptrTriggerEdge = nodeMap.GetNode("TriggerActivation");
      CEnumEntryPtr ptrTriggerRisingEdge = ptrTriggerEdge->GetEntryByName("RisingEdge");
      if (!IsAvailable(ptrTriggerRisingEdge) || !IsReadable(ptrTriggerRisingEdge))
      {
        std::cerr << "Unable to set trigger edge (enum entry retrieval). Aborting..." << std::endl;
        return;
      }

      ptrTriggerEdge->SetIntValue(ptrTriggerRisingEdge->GetValue());
    }
    CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
    if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
    {
      std::cerr << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << std::endl;
      return;
    }
    ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());
  }
  catch (Spinnaker::Exception& e)
  {
    std::cerr << "Error(Trigger): " << e.what() << std::endl;
  }
}
// }}}

// {{{ setNewConfiguration
bool PointGreyCameraSP::setNewConfiguration(const int& camera_id, pointgrey_camera_driver::PointGreyConfig& config,
                                            const uint32_t& level)
{
  if (!PointGreyCameraSP::isConnected())
  {
    PointGreyCameraSP::connect(camera_id);
  }

  // Activate mutex to prevent us from grabbing images during this time
  boost::mutex::scoped_lock scopedLock(mutex_);

  // return true if we can set values as desired.
  bool retVal = true;

  // Check video mode

  // Only change video mode if we have to.
  // dynamic_reconfigure will report anything other than LEVEL_RECONFIGURE_RUNNING if we need to change videomode.
  if (level != PointGreyCameraSP::LEVEL_RECONFIGURE_RUNNING)
  {
    bool wasRunning = PointGreyCameraSP::stop();  // Check if camera is running, and if it is, stop it.

    // Restart the camera if it was running
    if (wasRunning)
    {
      PointGreyCameraSP::start();
    }
  }

  // Set frame rate
  // retVal &= PointGreyCameraSP::setProperty(FRAME_RATE, false, config.frame_rate);
  // We don't need this when enable_trigger is true
  if (not config.enable_trigger)
  {
    setFrameRate(config.frame_rate);
  }

  // Set exposure
  // retVal &= PointGreyCameraSP::setProperty(AUTO_EXPOSURE, config.auto_exposure, config.exposure);
  setExposure(config.auto_exposure, config.exposure);

  // Set shutter time
  // double shutter = 1000.0 * config.shutter_speed; // Needs to be in milliseconds
  // retVal &= PointGreyCameraSP::setProperty(SHUTTER, config.auto_shutter, shutter);
  // config.shutter_speed = shutter / 1000.0; // Needs to be in seconds

  // Set gain
  // retVal &= PointGreyCameraSP::setProperty(GAIN, config.auto_gain, config.gain);
  setGain(config.auto_gain, config.gain);
  setGamma(config.gamma);
  setExternalTrigger(config.enable_trigger, config.trigger_source, config.trigger_polarity);

  return retVal;
}
// }}}

void PointGreyCameraSP::setGain(double& gain)
{
  // PointGreyCameraSP::setProperty(GAIN, false, gain);
}

void PointGreyCameraSP::setBRWhiteBalance(bool auto_white_balance, uint16_t& blue, uint16_t& red)
{
  // PointGreyCameraSP::setWhiteBalance(auto_white_balance, blue, red);
}

void PointGreyCameraSP::setTimeout(const double& timeout)
{
}

float PointGreyCameraSP::getCameraTemperature()
{
  INodeMap& nodeMap = cam_ptr_->GetNodeMap();
  CFloatPtr ptrDeviceTemperature = nodeMap.GetNode("DeviceTemperature");
  if (!IsAvailable(ptrDeviceTemperature) || !IsReadable(ptrDeviceTemperature))
  {
    std::cerr << "Unable to read actual temperature. Aborting..." << std::endl;
    return -273.15f;
  }
  double value = ptrDeviceTemperature->GetValue();
  return value;
}

void PointGreyCameraSP::connect(const int& camera_id)
{
  boost::mutex::scoped_lock scopedLock(mutex_);

  std::cerr << "CONNECT" << std::endl;

  system_ = System::GetInstance();
  cam_list_ = system_->GetCameras();
  unsigned int numCameras = cam_list_.GetSize();

  std::cerr << "Number of cameras detected: " << numCameras << std::endl;

  if (numCameras == 0)
  {
    return;
  }

  // Choose camera
  cam_ptr_ = cam_list_.GetByIndex(camera_id);

  try
  {
    INodeMap& nodeMapTLDevice = cam_ptr_->GetTLDeviceNodeMap();

    // int result = PrintDeviceInfo(nodeMapTLDevice);
    FeatureList_t features;
    CCategoryPtr category = nodeMapTLDevice.GetNode("DeviceInformation");
    if (IsAvailable(category) && IsReadable(category))
    {
      category->GetFeatures(features);

      FeatureList_t::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it)
      {
        CNodePtr pfeatureNode = *it;
        std::cerr << pfeatureNode->GetName() << " : ";
        CValuePtr pValue = (CValuePtr)pfeatureNode;
        std::cerr << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
        std::cerr << std::endl;
      }
    }
    else
    {
      std::cerr << "Device control information not available." << std::endl;
    }

    // Initialize camera
    cam_ptr_->Init();

    INodeMap& nodeMap = cam_ptr_->GetNodeMap();

    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
      std::cerr << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << std::endl;
      return;
    }

    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
      std::cerr << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << std::endl;
      return;
    }

    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
    std::cerr << "Acquisition mode set to continuous..." << std::endl;

    gcstring deviceSerialNumber("");
    CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
    {
      deviceSerialNumber = ptrStringSerial->GetValue();

      std::cerr << "Device serial number retrieved as " << deviceSerialNumber << "..." << std::endl;
    }

    std::cerr << "Device temperature is " << getCameraTemperature() << " Degrees Celcius..." << std::endl;
#if 1
    // test handler
    {
      CEnumerationPtr ptrEventSelector = nodeMap.GetNode("EventSelector");
      if (!IsAvailable(ptrEventSelector) || !IsReadable(ptrEventSelector))
      {
        std::cerr << "Unable to retrieve event selector entries. Aborting..." << std::endl;
        return;
      }
      CEnumEntryPtr ptrEnumEntry = ptrEventSelector->GetEntryByName("ExposureEnd");
      if (!IsAvailable(ptrEnumEntry) || !IsReadable(ptrEnumEntry))
      {
        std::cerr << "Unable to retrieve event selector entries. Aborting..." << std::endl;
        return;
      }
      ptrEventSelector->SetIntValue(ptrEnumEntry->GetValue());

      CEnumerationPtr ptrEventNotification = nodeMap.GetNode("EventNotification");
      if (!IsAvailable(ptrEventNotification) || !IsWritable(ptrEventNotification))
      {
        std::cerr << "Unable to retrieve event selector entries. Aborting..." << std::endl;
        return;
      }
      ptrEventNotification->SetIntValue(1);

      DeviceEventHandler* allDeviceEventHandler = new DeviceEventHandler(this);
      cam_ptr_->RegisterEvent(*allDeviceEventHandler);
    }
#endif
    std::cerr << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

void PointGreyCameraSP::disconnect()
{
  std::cerr << "disconnect:" << std::endl;
  boost::mutex::scoped_lock scopedLock(mutex_);
  captureRunning_ = false;
  if (isConnected())
  {
  }
}

void PointGreyCameraSP::start()
{
  std::cerr << "start:" << std::endl;
  if (isConnected() && !captureRunning_)
  {
    std::cerr << "Acquiring images..." << std::endl;
    cam_ptr_->BeginAcquisition();
    captureRunning_ = true;
    is_est_tm_initialized_ = false;
    is_cam_tm_initialized_ = false;
  }
}

bool PointGreyCameraSP::stop()
{
  std::cerr << "stop:" << std::endl;
  if (isConnected() && captureRunning_)
  {
    std::cerr << "Acquiring images..." << std::endl;
    cam_ptr_->EndAcquisition();
    // Stop capturing images
    captureRunning_ = false;
    is_est_tm_initialized_ = false;
    is_cam_tm_initialized_ = false;
    return true;
  }
  return false;
}

void PointGreyCameraSP::grabImage(sensor_msgs::Image& image, const std::string& frame_id)
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  if (isConnected() && captureRunning_)
  {
    ImagePtr pResultImage;
    pResultImage = cam_ptr_->GetNextImage();
    size_t width = pResultImage->GetWidth();
    size_t height = pResultImage->GetHeight();
    // in nano sec
    uint64_t tm_result = pResultImage->GetTimeStamp();
    // spinnaker reference was wrong; it said the timestamp is in nanosecond, but actually it is in microsecond
    long tm_sec = tm_result / 1000000;
    long tm_nsec = tm_result % 1000000 * 1000;
    ros::Time cam_tm(tm_sec, tm_nsec);
    if (not is_cam_tm_initialized_)
    {
      last_cam_tm_ = cam_tm;
      is_cam_tm_initialized_ = true;
    }
    if (not is_est_tm_initialized_)
    {
      std::cerr << frame_id << " init timestamp is " << cam_tm.sec << "." << std::setw(9) << std::setfill('0')
                << cam_tm.nsec << std::setfill(' ') << std::endl;
      est_tm_ = cam_tm;
      is_est_tm_initialized_ = true;
    }
    ros::Duration cam_diff = cam_tm - last_cam_tm_;

    // estimating image header timestamp by P control
    est_tm_ = est_tm_ + cam_diff;
    est_tm_ = est_tm_ + static_cast<ros::Duration>((sys_tm_ - est_tm_) * 0.3);

    ros::Time tm_now;
    tm_now = est_tm_;

    // std::cerr << frame_id.at(frame_id.size() - 1)                                                         //
    // << ", cam: " << cam_tm.sec % 10 << "." << std::setw(9) << std::setfill('0') << cam_tm.nsec  //
    // << ", last_cam_tm_: " << last_cam_tm_.sec << "." << std::setw(9) << std::setfill('0') <<
    // last_cam_tm_.nsec  //
    // << ", cam_diff: " << cam_diff.sec << "." << std::setw(9) << std::setfill('0') << cam_diff.nsec  //
    // << ", cam_tm: " << cam_tm.sec << "." << std::setw(9) << std::setfill('0') << cam_tm.nsec     //
    // << ", sys: " << sys_tm_.sec % 10 << "." << std::setw(9) << std::setfill('0') << sys_tm_.nsec  //
    // << ", est: " << est_tm_.sec % 10 << "." << std::setw(9) << std::setfill('0') << est_tm_.nsec  //
    // << std::endl;

    last_cam_tm_ = cam_tm;
    image.header.stamp = tm_now;
    // std::cerr << tm_sec << "/" << tm_nano << " : " << width << " - " << height << ", sz = " << imsize << ", bpp = "
    // << bpp << std::endl;
    // ROS_INFO("%ld %ld", tm_sec, tm_nano);
    // ImagePtr convertedImage = pResultImage->Convert(PixelFormat_RGB8, HQ_LINEAR);
    // ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
    // DEFAULT
    // NO_COLOR_PROCESSING
    // NEAREST_NEIGHBOR
    // RIGOROUS
    // DIRECTIONAL_FILTER
    // ImagePtr convertedImage = pResultImage;
    // ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, DEFAULT);
    ImagePtr convertedImage = pResultImage->Convert(PixelFormat_RGB8, DEFAULT);

    std::string imageEncoding = sensor_msgs::image_encodings::RGB8;
    // std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
    // std::string imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    width = convertedImage->GetWidth();
    height = convertedImage->GetHeight();
    size_t stride = convertedImage->GetStride();
    size_t imsize = convertedImage->GetImageSize();
    int bpp = convertedImage->GetBitsPerPixel();
    // std::cerr << width << " - " << height << ", st = " << stride << std::endl;

    last_ = now_;
    now_ = std::chrono::system_clock::now();
    double diff = std::chrono::duration_cast<std::chrono::microseconds>(now_ - last_).count() / 1000000.0;
    if (count_test_ == 100)
    {
      count_test_ = 0;
      sum_test_ = 0;
    }
    count_test_++;
    sum_test_ += diff;
    if (count_test_ % 100 == 0)
    {
      if (diff > 10e-5)
      {
        std::cerr << std::fixed << std::setw(10) << std::right << 1.0 / (sum_test_ / 100.0) << " " << frame_id
                  << std::endl;
      }
      else
      {
        std::cerr << "interval too short" << std::endl;
      }
    }
    fillImage(image, imageEncoding, convertedImage->GetHeight(), convertedImage->GetWidth(),
              convertedImage->GetStride(), convertedImage->GetData());
    image.header.frame_id = frame_id;
  }
  else if (isConnected())
  {
    throw CameraNotRunningException("PointGreyCameraSP::grabImage: Camera is currently not running.  Please start the "
                                    "capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCameraSP::grabImage not connected!");
  }
}

void PointGreyCameraSP::grabStereoImage(sensor_msgs::Image& image, const std::string& frame_id,
                                        sensor_msgs::Image& second_image, const std::string& second_frame_id)
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  if (isConnected() && captureRunning_)
  {
  }
  else if (isConnected())
  {
    throw CameraNotRunningException("PointGreyCameraSP::grabStereoImage: Camera is currently not running.  Please "
                                    "start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCameraSP::grabStereoImage not connected!");
  }
}

uint PointGreyCameraSP::getGain()
{
  return 0;
}

uint PointGreyCameraSP::getShutter()
{
  return 0;
}

uint PointGreyCameraSP::getBrightness()
{
  return 0;
}

uint PointGreyCameraSP::getExposure()
{
  return 0;
}

uint PointGreyCameraSP::getWhiteBalance()
{
  return 0;
}

uint PointGreyCameraSP::getROIPosition()
{
  return 0;
}

void PointGreyCameraSP::setDesiredCamera(const uint32_t& id)
{
  serial_ = id;
}

std::vector<uint32_t> PointGreyCameraSP::getAttachedCameras()
{
  std::vector<uint32_t> cameras;
  unsigned int num_cameras;
#if 0
  Error error = busMgr_.GetNumOfCameras(&num_cameras);
  PointGreyCameraSP::handleError("PointGreyCameraSP::getAttachedCameras: Could not get number of cameras", error);
  for(unsigned int i = 0; i < num_cameras; i++)
  {
    unsigned int this_serial;
    error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
    PointGreyCameraSP::handleError("PointGreyCameraSP::getAttachedCameras: Could not get get serial number from index", error);
    cameras.push_back(this_serial);
  }
#endif
  return cameras;
}

bool PointGreyCameraSP::isConnected()
{
  return (cam_ptr_ != 0);
}
