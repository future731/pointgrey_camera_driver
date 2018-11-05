#include "pointgrey_camera_driver/PointGreyCameraSpinnaker.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

int main()
{
  SystemPtr system = System::GetInstance();
  CameraList cam_list = system->GetCameras();
  unsigned int cam_list_size = cam_list.GetSize();
  if (cam_list_size == 0)
  {
    cam_list.Clear();
    system->ReleaseInstance();
    std::cerr << "Detected 0 camera. Abort." << std::endl;
    return EXIT_FAILURE;
  }
  for (unsigned int i = 0; i < cam_list_size; i++)
  {
    CameraPtr cam = cam_list.GetByIndex(i);
    cam->Init();
    INodeMap& node_map = cam->GetNodeMap();
    CCommandPtr ptrDeviceResetCommand = node_map.GetNode("DeviceReset");
    if (not IsAvailable(ptrDeviceResetCommand) or not IsWritable(ptrDeviceResetCommand))
    {
      std::cerr << "Unable to Execute Device Reset. Aborting..." << std::endl;
      return EXIT_FAILURE;
    }
    ptrDeviceResetCommand->Execute();
  }

  cam_list.Clear();
  system->ReleaseInstance();

  return EXIT_SUCCESS;
}
