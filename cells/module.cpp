#include <ecto/ecto.hpp>
#include <ecto_image_pipeline/pinhole_camera_model.h>
#include <ecto_image_pipeline/enums.hpp>

using namespace image_pipeline;

ECTO_DEFINE_MODULE(base){
  boost::python::enum_<InterpolationMode>("InterpolationMode")
    .value("CV_INTER_NN",image_pipeline::CV_INTER_NN)
    .value("CV_INTER_LINEAR",image_pipeline::CV_INTER_LINEAR)
    .value("CV_INTER_CUBIC",image_pipeline::CV_INTER_CUBIC)
    .value("CV_INTER_AREA",image_pipeline::CV_INTER_AREA)
    .export_values()
    ;
}
