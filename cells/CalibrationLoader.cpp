#include <ecto/ecto.hpp>
#include <image_pipeline/pinhole_camera_model.h>

using ecto::tendrils;
namespace image_pipeline
{
  struct PinholeModelLoader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("filename", "The camera calibration file.", "camera.yml");
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare<PinholeCameraModel>("model", "The camera model loaded.");
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      std::string filename;
      params["filename"] >> filename;
      camera->readCalibration(filename);
    }
    ecto::spore<PinholeCameraModel> camera;
  };
}

ECTO_CELL(base, image_pipeline::PinholeModelLoader, "PinholeModelLoader",
          "This reads a monocular camera calibration file and puts the results on the output.");
