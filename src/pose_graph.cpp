#include <ecto_image_pipeline/pose_graph.hpp>
#define BOOST_NO_HASH

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <map>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/format.hpp>
namespace image_pipeline
{
  using namespace boost;

  struct PoseGraph::impl
  {
    typedef adjacency_list<setS, // OutEdgeList, set so that parallel edges aren't added.
        vecS, // VertexList
        directedS, //directional, so that its easy to compute poses by just following the shortest path.
                   //any connected nodes will be associated by two edges, a~>b, b~>a, each with an inverted pose.
        frame_id, // vertex property, frame id, string
        property<edge_weight_t, float, //weight, for shortest path
            transform //edge property, holds the transform.
        > //end of edge properties
    > graph_t;

    typedef graph_traits<graph_t>::vertex_descriptor vd_t;
    typedef graph_traits<graph_t>::edge_descriptor ed_t;
    typedef std::map<frame_id, vd_t> map_t;

    graph_t graph_;
    map_t frame_map; //reverse lookup of vertex descriptor, using string name.

    /**
     * Get a frame vertex descriptor
     * @param x
     * @param vd
     * @return
     */
    bool
    frame(const frame_id& x, vd_t& vd) const
    {
      map_t::const_iterator it = frame_map.find(x);
      if (it == frame_map.end())
        return false;
      vd = it->second;
      return true;
    }

    vd_t
    add_frame(const frame_id& x)
    {
      vd_t vd;
      if (frame(x, vd))
        return vd;
      vd = boost::add_vertex(x, graph_);
      frame_map[x] = vd;
      return vd;
    }
  };

  PoseGraph::PoseGraph()
      :
        impl_(new impl)
  {

  }
  void
  PoseGraph::set(const frame_id& a, const frame_id& b, const transform & t_ab)
  {
    impl::vd_t u, v;
    u = impl_->add_frame(a);
    v = impl_->add_frame(b);
    //todo figure out a good weight for computing shortest path.. use distance and number of links?
    float norm = t_ab.translation().norm() + 1.0; //add a constant per link.
    impl::ed_t ed;
    bool added;
    tie(ed, added) = boost::add_edge(u, v, impl_->graph_);
    impl_->graph_[ed] = t_ab;
    put(edge_weight, impl_->graph_, ed, norm); //use the translation norm as the edge weight

    //reverse the transform
    tie(ed, added) = boost::add_edge(v, u, impl_->graph_);
    impl_->graph_[ed] = t_ab.inverse();
    put(edge_weight, impl_->graph_, ed, norm);

  }

  bool
  PoseGraph::lookup(const frame_id& a, const frame_id& b, transform& t_out) const
  {
    bool found = false;
    impl::vd_t to, from;
    found = impl_->frame(b, to);
    if (!found)
      return found;
    found = impl_->frame(a, from);
    if (!found)
      return found;
    std::vector<impl::vd_t> parents(num_vertices(impl_->graph_));
    std::vector<float> distances(num_vertices(impl_->graph_));
    dijkstra_shortest_paths(impl_->graph_, to, predecessor_map(&parents[0]).distance_map(&distances[0]));
    impl::ed_t ed;
    t_out.setIdentity();
    while (from != to)
    {
      bool link_found = false;
      tie(ed, link_found) = edge(from, parents[from], impl_->graph_);
      if (link_found)
      {
        transform t_ab = get(edge_bundle, impl_->graph_, ed);
        t_out = t_out * t_ab; //accumulate the transform.
        from = parents[from];
      }
      else
      {
        return false;
      }
    }
    return found;
  }
  PoseGraph::transform
  PoseGraph::operator()(const frame_id& a, const frame_id& b) const
  {
    transform t;
    if (!lookup(a, b, t))
      throw std::runtime_error(boost::str(boost::format("Could not find a transform between %s and %s") % a % b));
    return t;
  }

}

