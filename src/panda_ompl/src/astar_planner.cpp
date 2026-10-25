#include "panda_ompl/planning_engine.hpp"
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>

namespace panda_ompl {
std::vector<Q> PlanningEngine::astar_path(const Q &start,const Q &goal,double timeout,PlanResult &result) {
  // Start-anchored 7-D lattice: 14 axis neighbours, Euclidean edge costs.
  // Exact goal is an extra vertex connected to collision-free nodes within
  // 0.30 rad. Optimality applies to this discrete graph, not continuous space.
  constexpr double step=.15, goal_radius=.30, resolution=.02;
  constexpr std::size_t max_nodes=500000;
  using Key=std::array<int,7>;
  const Key goal_key{std::numeric_limits<int>::max(),0,0,0,0,0,0};
  struct Node { double g=std::numeric_limits<double>::infinity();Key parent{};bool closed=false; };
  struct Entry {double f,g;Key key;};
  struct Greater {bool operator()(const Entry &a,const Entry &b)const {
    if(a.f!=b.f)return a.f>b.f;return a.key>b.key;
  }};
  std::map<Key,Node> nodes;
  std::priority_queue<Entry,std::vector<Entry>,Greater> open;
  auto norm=[](const Q &a,const Q &b) {double s=0;for(int j=0;j<7;++j)s+=(a[j]-b[j])*(a[j]-b[j]);return std::sqrt(s);};
  auto state=[&](const Key &k) {if(k==goal_key)return goal;Q q=start;for(int j=0;j<7;++j)q[j]+=step*k[j];return q;};
  const auto began=std::chrono::steady_clock::now();
  auto check_time=[&] {
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>=timeout)
      throw std::runtime_error("astar 搜索超时：七维网格未找到精确路径");
  };
  auto edge=[&](const Q &a,const Q &b) {
    const int count=std::max(1,static_cast<int>(std::ceil(norm(a,b)/resolution)));
    for(int i=1;i<=count;++i) {
      check_time();Q q(7);for(int j=0;j<7;++j)q[j]=a[j]+(b[j]-a[j])*i/count;
      ++result.validity_queries;if(!valid(q))return false;
    }
    return true;
  };
  Key root{};nodes[root].g=0;open.push({norm(start,goal),0,root});
  while(!open.empty()) {
    check_time();const auto current=open.top();open.pop();
    auto &node=nodes.at(current.key);
    if(node.closed||current.g!=node.g)continue;
    if(current.key==goal_key) {
      std::vector<Q> path;Key k=goal_key;
      while(k!=root) {path.push_back(state(k));k=nodes.at(k).parent;}
      path.push_back(start);std::reverse(path.begin(),path.end());
      result.search_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
      result.first_solution_seconds=result.search_seconds;
      return path;
    }
    node.closed=true;const Q q=state(current.key);
    auto relax=[&](const Key &next,const Q &to) {
      const double cost=node.g+norm(q,to);
      auto it=nodes.find(next);
      if(it!=nodes.end()&&cost>=it->second.g)return;
      for(int j=0;j<7;++j)if(to[j]<lower_[j]||to[j]>upper_[j])return;
      if(!edge(q,to))return;
      if(it==nodes.end()&&nodes.size()>=max_nodes)
        throw std::runtime_error("astar 达到 500000 节点内存限制");
      auto &n=nodes[next];n.g=cost;n.parent=current.key;n.closed=false;
      open.push({cost+norm(to,goal),cost,next});
    };
    if(norm(q,goal)<=goal_radius)relax(goal_key,goal);
    for(int j=0;j<7;++j)for(int sign:{-1,1}) {
      Key next=current.key;next[j]+=sign;relax(next,state(next));
    }
  }
  throw std::runtime_error("astar 网格搜索耗尽：当前离散图无可行路径");
}
} // namespace panda_ompl
