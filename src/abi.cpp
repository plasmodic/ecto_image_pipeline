#include <ecto_image_pipeline/abi.hpp>
#include <stdexcept>

namespace  image_pipeline {

  abi_checker::abi_checker(unsigned running_version)
  {
    if (running_version != abi_version)
      throw incompatible_version(running_version, abi_version);
  };

  incompatible_version::incompatible_version(unsigned c, unsigned r)
    : compiled_version(c), running_version(r)
  { }

  const char* incompatible_version::what() const throw() {
    return "Runtime version ofimage_pipeline is incompatible with"
           " the compiletime version of  image_pipeline.  Check "
           "the contents of your runtime link path.";
  }
}
