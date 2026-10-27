#include "panda_ompl/planning_engine.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>

namespace panda_ompl {
std::vector<Q> PlanningEngine::prm_astar_path(const Q &start,const Q &goal,double timeout,PlanResult &result) {
  using Clock=std::chrono::steady_clock;
  const auto began=Clock::now();
  const auto elapsed=[&]{return std::chrono::duration<double>(Clock::now()-began).count();};
  const auto check=[&]{if(elapsed()>=timeout)throw std::runtime_error("prm_astar 搜索超时（含首次建图）");};
  const auto dist=[](const Q &a,const Q &b){double x=0;for(int j=0;j<7;++j)x+=(a[j]-b[j])*(a[j]-b[j]);return std::sqrt(x);};
  constexpr std::size_t count=1800, neighbours=20;
  constexpr double radius=2.5;
  result.roadmap_reused=!roadmap_points_.empty();
  // Candidate graph only: collision status is deliberately NOT persistent.
  // Failed builds commit nothing. Sampling is independent of each query.
  if(!result.roadmap_reused) {
    std::vector<Q> points;points.reserve(count);
    const Q home{0,-.785398,0,-2.356194,0,1.570796,.785398};
    std::mt19937 rng(42);std::normal_distribution<double> normal(0,1.0);
    for(std::size_t i=0;i<count;++i) {
      check();Q q(7);
      for(int j=0;j<7;++j) {
        if(i%2==0) { // Truncated normal, not clamping mass onto joint limits.
          do {check();q[j]=home[j]+normal(rng);}while(q[j]<lower_[j]||q[j]>upper_[j]);
        } else q[j]=std::uniform_real_distribution<double>(lower_[j],upper_[j])(rng);
      }
      points.push_back(q);
    }
    std::vector<std::vector<std::size_t>> edges(count);
    for(std::size_t i=0;i<count;++i) {
      check();std::vector<std::pair<double,std::size_t>> near;
      for(std::size_t j=0;j<count;++j)if(i!=j) {const double d=dist(points[i],points[j]);if(d<=radius)near.emplace_back(d,j);}
      const auto n=std::min(neighbours,near.size());std::partial_sort(near.begin(),near.begin()+n,near.end());
      for(std::size_t k=0;k<n;++k){edges[i].push_back(near[k].second);edges[near[k].second].push_back(i);}
    }
    for(auto &adj:edges){std::sort(adj.begin(),adj.end());adj.erase(std::unique(adj.begin(),adj.end()),adj.end());}
    roadmap_points_=std::move(points);roadmap_edges_=std::move(edges);
    result.roadmap_build_seconds=elapsed();
  }
  result.roadmap_nodes=roadmap_points_.size();
  for(const auto &adj:roadmap_edges_)result.roadmap_edges+=adj.size();result.roadmap_edges/=2;
  const auto query_begin=Clock::now();
  auto points=roadmap_points_;auto edges=roadmap_edges_;
  const auto source=points.size(),target=source+1;points.push_back(start);points.push_back(goal);edges.resize(points.size());
  for(auto id:{source,target}) {
    std::vector<std::pair<double,std::size_t>> near;
    for(std::size_t j=0;j<source;++j){check();const double d=dist(points[id],points[j]);if(d<=radius)near.emplace_back(d,j);}
    const auto n=std::min(neighbours,near.size());std::partial_sort(near.begin(),near.begin()+n,near.end());
    for(std::size_t k=0;k<n;++k){edges[id].push_back(near[k].second);edges[near[k].second].push_back(id);}
  }
  // Same local connector as roadmap edges, including a possible direct route.
  if(dist(start,goal)<=radius){edges[source].push_back(target);edges[target].push_back(source);}
  std::set<std::pair<std::size_t,std::size_t>> blocked,validated;
  auto key=[](std::size_t a,std::size_t b){return std::make_pair(std::min(a,b),std::max(a,b));};
  auto valid_connection=[&](std::size_t a,std::size_t b) {
    const int n=std::max(1,static_cast<int>(std::ceil(dist(points[a],points[b])/.02)));
    for(int i=0;i<=n;++i){check();Q q(7);for(int j=0;j<7;++j)q[j]=points[a][j]+(points[b][j]-points[a][j])*i/n;
      ++result.validity_queries;if(!valid(q))return false;}
    return true;
  };
  // Lazy PRM: A* first searches optimistic candidate edges, then validates the
  // candidate path. Block invalid edges and rerun until a fully valid path.
  while(true) {
    check();const auto size=points.size();const double inf=std::numeric_limits<double>::infinity();
    std::vector<double> cost(size,inf);std::vector<std::size_t> parent(size,size);
    using Entry=std::pair<double,std::size_t>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
    cost[source]=0;open.emplace(dist(start,goal),source);
    bool found=false;
    while(!open.empty()) {
      check();auto [f,a]=open.top();open.pop();if(f>cost[a]+dist(points[a],goal)+1e-10)continue;
      if(a==target){found=true;break;}
      for(auto b:edges[a]) {
        if(blocked.count(key(a,b)))continue;
        const double g=cost[a]+dist(points[a],points[b]);
        if(g<cost[b]){cost[b]=g;parent[b]=a;open.emplace(g+dist(points[b],goal),b);}
      }
    }
    if(!found)throw std::runtime_error("prm_astar 当前路线图无可行路径；不代表连续空间不可达");
    std::vector<std::size_t> path;
    for(auto id=target;id!=source;id=parent[id])path.push_back(id);
    path.push_back(source);std::reverse(path.begin(),path.end());
    bool ok=true;
    for(std::size_t i=1;i<path.size();++i) {
      const auto edge=key(path[i-1],path[i]);if(validated.count(edge))continue;
      if(!valid_connection(path[i-1],path[i])){blocked.insert(edge);ok=false;break;}
      validated.insert(edge);
    }
    if(ok) {
      result.roadmap_query_seconds=std::chrono::duration<double>(Clock::now()-query_begin).count();
      result.search_seconds=elapsed();result.first_solution_seconds=result.search_seconds;
      std::clog<<"prm_astar build_s="<<result.roadmap_build_seconds<<" query_s="<<result.roadmap_query_seconds
        <<" reused="<<result.roadmap_reused<<" nodes="<<result.roadmap_nodes<<" edges="<<result.roadmap_edges<<'\n';
      std::vector<Q> answer;for(auto id:path)answer.push_back(points[id]);return answer;
    }
  }
}
} // namespace panda_ompl
