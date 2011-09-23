#include <ecto/ecto.hpp>
#include <image_pipeline/pinhole_camera_model.h>

using ecto::tendrils;

namespace image_pipeline
{
  struct CalibrationLoader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("filename", "The camera calibration file.", "camera.yml");
    }
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare<PinholeCameraModel>("camera", "The camera model loaded.");
    }
    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      camera = out["camera"]; // this is like a smart pointer to the output.
      std::string filename;
      params["filename"] >> filename;
      //TODO load the camer from file.
      //load(*camera, filename);
    }
    ecto::spore<PinholeCameraModel> camera;
  };
}

ECTO_CELL(image_pipeline, image_pipeline::CalibrationLoader, "CalibrationLoader",
          "This reads a camera calibration file and puts the results on the outputs.");
