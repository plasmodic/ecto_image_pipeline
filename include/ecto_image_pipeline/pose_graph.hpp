#pragma once
#include "abi.hpp"
#include "pose.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace image_pipeline
{
  /**
   * A simple pose graph for keeping track of frames and their relative poses.
   * Uses the Eigen::Affine3d for the transform, string ids for the frames, and internally boost graph for the tree
   * structure.
   */
  struct PoseGraph: boost::noncopyable
  {
    typedef std::string frame_id;
    class transform : public Eigen::Affine3d {
    public:
      transform() {
        // This is needed has m_matrix is otherwise uninitialized and some Eigen warning appear on 32 bits
        // and are perceived as an error with -Werror
        this->m_matrix.setIdentity();
      }
      transform(const Eigen::Affine3d & other) {
        this->m_matrix = other.matrix();
      }
      transform & operator=(const transform & other) {
        m_matrix = other.m_matrix;
        return *this;
      }
      transform & operator=(const Eigen::Affine3d & other) {
        m_matrix = other.matrix();
        return *this;
      }
    };
    PoseGraph();
    /**
     * Explicitly set the transform between frame a and b. This creates a "link" between the two.
     * No need to set the opposite transform, will be done for you.
     * @param a
     * @param b
     * @param t_ab the transform that goes from a to b.
     */
    void
    set(const frame_id& a, const frame_id& b, const transform & t_ab);

    /**
     * Find the transform from a to b.
     * @param a
     * @param b
     * @param transform The output, will be meaningless if the return value is false.
     * @return False if no transform exists between the two frames.
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

