#pragma once
#include <stdexcept>
namespace image_pipeline
{
  const static unsigned abi_version = 0; //ABI version

  // exception to be thrown in case of detected incompatibility
  struct incompatible_version : std::exception
  {
    unsigned compiled_version, running_version;
    incompatible_version(unsigned compiledwith, unsigned runningwith);
    virtual const char* what() const throw();
  };

  // object that checks ABI version in its constructor
  struct abi_checker {
    abi_checker(unsigned version);
  };

// CMake will define foo_EXPORTS when compiling foo itself (and not
// when compiling *against* Foo.  So this will save us a very small amount
// of space inside libfoo itself, but it doesn't hurt if they are there.
#ifndef image_pipeline_EXPORTS
  // an instance of this object... there will be one in each
  // translation unit that compiles against this header
  namespace {
    abi_checker checker(abi_version);
  }
#endif //image_pipeline_EXPORTS
}
