#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
using ecto::tendrils;
namespace image_pipeline
{
  template<typename T>
  struct Latch
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<bool>("init", "Set initial value from input.", false);
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&Latch::in_, "input", "The input to copy to the output..").required(true);
      in.declare(&Latch::set_, "set", "The latch a value.", false);
      in.declare(&Latch::reset_, "reset", "The latch a value.", false);
      out.declare(&Latch::out_, "output", "A copy of the input.");
    }

    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      params["init"] >> init_;
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      if (init_)
	{
	  init_ = false;
	  *out_ = *in_;
	}


      if (*reset_)
      {
        *reset_ = false;
        *set_ = false;
        *out_ = T();
      }
      if (*set_)
        *out_ = *in_;
      return ecto::OK;
    }
    ecto::spore<T> in_, out_;
    ecto::spore<bool> set_, reset_;
    bool init_;
  };
}

ECTO_CELL(base, image_pipeline::Latch<bool>, "LatchBool", "Latch a bool.");

ECTO_CELL(base, image_pipeline::Latch<cv::Mat>, "LatchMat", "Latch a cv::Mat.");

