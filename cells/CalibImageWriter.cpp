#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/select.h>

#if 0
       int select(int nfds, fd_set *readfds, fd_set *writefds,
                  fd_set *exceptfds, struct timeval *timeout);

       void FD_CLR(int fd, fd_set *set);
       int  FD_ISSET(int fd, fd_set *set);
       void FD_SET(int fd, fd_set *set);
       void FD_ZERO(fd_set *set);

       #include <sys/select.h>

       int pselect(int nfds, fd_set *readfds, fd_set *writefds,
                   fd_set *exceptfds, const struct timespec *timeout,
                   const sigset_t *sigmask);
#endif


using ecto::tendrils;

namespace image_pipeline
{
  struct CalibImageWriter
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string> ("filename", "Output file name", "rgb_00.png");
    }
    static void

    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat> ("image",
			   "The grayscale image with a calibration pattern");
      in.declare<bool> ("found", "Whether or not a pattern was found...");
    }
    void

    configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      image_in = in["image"];
      found = in["found"];
      params["filename"] >> filename;

    }

    int process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      if (*found)
	std::cout << "Found a pattern" << std::endl;
      else
	std::cout << "Pattern not found" << std::endl;

      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 0;
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(0,&readfds);

      if (select(1,&readfds,0,0,&timeout))
	{
	  std::cout << "Keypress" << std::endl;	
	  if (*found)
	    {
	      std::cout << "Writing file to " << filename << std::endl;	
	      cv::imwrite(filename, *image_in);
	      return ecto::QUIT;
	    }
	}

      return ecto::OK;
    }

    ecto::spore<bool> found;
    ecto::spore<cv::Mat> image_in;
    std::string filename;

  };
}

ECTO_CELL(image_pipeline, image_pipeline::CalibImageWriter, "CalibImageWriter",
          "Write out a calibration image to a file, when triggered by an input.");
