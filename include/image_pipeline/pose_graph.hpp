#pragma once
#include <image_pipeline/abi.hpp>
#include <image_pipeline/pose.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace image_pipeline
{
  struct PoseGraph: boost::noncopyable
  {
    typedef std::string frame_id;
    typedef Eigen::Affine3d transform;
    PoseGraph();
    /**
     * Set the transform between frame a and b.  No need to set the opposite transform, will be done for you.
     * @param a
     * @param b
     * @param t_ab the transform to use.
     */
    void
    set(const frame_id& a, const frame_id& b, const transform & t_ab);

    /**
     * Find the transform from a to b.
     * @param a
     * @param b
     * @param transform The output, will be meaningless if the return value is false.
     * @return False if no transform is available.
     */
    bool
    lookup(const frame_id& a, const frame_id& b, transform& transform) const;

    /**
     * Looks up the transform between a and b and returns the result, may be computed.
     * @param a
     * @param b
     * @return
     */
    transform
    operator()(const frame_id& a, const frame_id& b) const;

  private:
    struct impl;
    boost::shared_ptr<impl> impl_;
  };

}

