// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <cmath>
#include <ctime>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <deque>

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"

#include "pnc/map/map_lib.h"
#include "pnc/simulation/vehicle_agent.h"
#include "pnc/simulation/vehicle_agent_factory.h"


namespace jcvb {

const double pi = 3.141592653589793238462643383;
const double speed_limit = 13.888888889;
const double hard_speed_limit = 13.888888889/50*55;
const double max_deceleration = 10.58822;
const double time_to_reach_max_deceleration_upper_bound = 0.4;
const double time_to_reach_zero_acc = 0.5;

const int STRAIGHT = 1;
const int RIGHT = 2;
const int LEFT = 3;

int sgn(double x){
	if (x>0)return 1;
	else return -1;
}

bool in_between(double a,double l, double r, double eps = 0.5) {
	return (a>l-eps && a< r+eps) || (a>r-eps && a< l+eps);
}

inline interface::geometry::Point3D ToPoint3D (interface::geometry::Point2D p){
		  interface::geometry::Point3D pt;
		  pt.set_x(p.x());
		  pt.set_y(p.y());
		  pt.set_z(0);
		  return pt;
}
inline interface::geometry::Point3D Midpoint (interface::geometry::Point3D p, interface::geometry::Point3D q){
		  interface::geometry::Point3D pt;
		  pt.set_x(0.5*(p.x()+q.x()));
		  pt.set_y(0.5*(p.y()+q.y()));
		  return pt;
}
inline interface::geometry::Point3D ToPoint3D (interface::geometry::Vector3d p){
		  interface::geometry::Point3D pt;
		  pt.set_x(p.x());
		  pt.set_y(p.y());
		  pt.set_z(0);
		  return pt;
}
inline interface::geometry::Point2D ToPoint2D (interface::geometry::Point3D p){
		  interface::geometry::Point2D pt;
		  pt.set_x(p.x());
		  pt.set_y(p.y());
		  return pt;
}

class JcvbVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit JcvbVehicleAgent(const std::string& name) : VehicleAgent(name) {
	  my_name_ = name;
  }

  void Initialize(const interface::agent::AgentStatus&  agent_status ) override {
	  LoadMap();
	  //cur_rou_ = FindRoute(ToPoint3D(agent_status.vehicle_status().position()), agent_status.route_status().destination());
  }

  interface::control::ControlCommand RunOneIteration(
		  const interface::agent::AgentStatus& agent_status) override {
	  interface::control::ControlCommand command;

	  cur_pos_ = agent_status.vehicle_status().position();
	  cur_dest_ = agent_status.route_status().destination();
	  cur_velo_ = agent_status.vehicle_status().velocity();
	  double v = CalcVelocity(cur_velo_);
	  double dv = UpdateVelocityAndGetDerivative(v);
	  cur_acc_ = agent_status.vehicle_status().acceleration_vcs();
	  nowtime_ = agent_status.simulation_status().simulation_time();
	  UpdateObstacles(agent_status);

	  if (agent_status.route_status().is_new_request()) {
	  	  //if (my_name_ == "jcvb1") right_ = true;
	  	  /*if (std::abs(cur_pos_.x() - 227)<1 && std::abs(cur_pos_.y() + 2.5)<1 ) {
			  //my_name_ = "jcvb2";
		  }*/
		  cur_rou_ = FindRoute();
	  }



	  double to_dest = DistanceToDestination();
	  if (to_dest < 35) right_ = false;

	  double brake_dist = GetBrakeDistance(v, cur_acc_.x());

	  auto lightinfo = GetLightInfo(agent_status);
	  double to_red = lightinfo.dist;
	  if (v<5.5 && lightinfo.last_state == interface::map::Bulb::RED) {
	  	  to_red = 1e9;
	  }
	  if (LongLane(cur_rou_.back().lane_ind)) {
		if (CalcDistance(cur_pos_, LoadLanePoint(GetEnd(cur_rou_.back()) )) < 15) {
				if (lightinfo.this_state != -1 && to_red > 1e8 && opposite_light_[cur_rou_.back().lane_ind]!="") {
					at_crossing_ = true;
				}
		} else {
			at_crossing_ = false;
		}
	  }

	  if (at_crossing_) {
	  	  int opposite_light_lane = -1;
	  	  for (int i=0;i<lane_light_.size();i++) {
	  	  	  if (lane_light_[i] == opposite_light_[cur_rou_.back().lane_ind]) {
	  	  	  	  opposite_light_lane = i;
	  	  	  	  break;
			  }
		  }
		  PublishVariable("atcross", "at", utils::display::Color::Red());
		  PublishVariable("next", std::to_string(NextTurnType()), utils::display::Color::Red());


	  }
	  //TODO obs at lane ?


	  double to_front = GetFrontObstacleSafeDistance();

	  double dist_to = std::min(std::min(to_dest, to_red), to_front);
	  if(dist_to<1e-10) dist_to = 1e-10;
	  //dist_to = 1e9;//


	  if (dist_to < brake_dist){
	  	  double del = (v*v/(-cur_acc_.x())/2) / dist_to;
	  	  if (del<0.0) del = 2.0;
		  if (v > 2.5) {
			  del = std::min(del, 2.00);
			  del = std::max(del, 0.60);
			  command.set_brake_ratio(0.263 * del);
		  } else {
			  del = std::min(del, 2.00);
			  del = std::max(del, 0.50);
			  command.set_brake_ratio(0.13 * del);
		  }
	  } else {
		  if (v < GetSpeedLimit() - 0.55) {
			  command.set_throttle_ratio(GetThrottleForAcceleration(v));
		  } else if (v > GetSpeedLimit() + 0.45) {
			  command.set_brake_ratio(0.3);
		  }
	  }

	  /*if (agent_status.simulation_status().simulation_time() > 4.4) {
			  command.set_brake_ratio(1.0);
			  command.set_throttle_ratio(0.0);
	  }*/

	  command.set_steering_angle(GetSteeringAngle());


	  //if (ObstaclesOnRoute(GetBrakeDistanceForPedestrian(v), 1.5+ 1.66667*(time_to_reach_max_deceleration_upper_bound + v/max_deceleration))) {
	  if (CheckHardBrake(dist_to)) {
			  command.set_brake_ratio(1.0); // hard brake
			  command.set_throttle_ratio(0.0);
			  //std::cout<<"hard"<<std::endl;
			  PublishVariable("hard", "hard", utils::display::Color::Red());
	  } else {


	  }

	  //std::cout<<command.brake_ratio()<<" "<<command.throttle_ratio()<<std::endl;

	  PublishVariable("key1", map_.lane()[cur_rou_.back().lane_ind].id().id());

	  /*if (my_name_ == "jcvb12") {
		  if (obstacle_id_map_.count("Cjcvb15")) {
			  if(obstacle_history_[obstacle_id_map_["Cjcvb15"]].size()>0) {
				  auto po = obstacle_history_[obstacle_id_map_["Cjcvb15"]].back();
				  std::string dd = std::to_string(CalcDistance(cur_pos_, po.center()));
				  PublishVariable("distto15", dd, utils::display::Color::Red());
				  std::cout<<dd<<std::endl;
			  }
		  }
	  }*/
	  //std::cout<<my_name_<<" "<<agent_status.vehicle_status().position().x()<<" "<<agent_status.vehicle_status().position().y()<<std::endl;
	  //std::cout<< agent_status.route_status().destination().x() <<' '<< agent_status.route_status().destination().y()<<'\n';

	  return command;
  }

 private:
  double GetSpeedLimit() {
	 return hard_speed_limit - 0.5;
  }
  class Polygon {
  public:
	  std::vector<interface::geometry::Point3D> points;

	  interface::geometry::Point3D center() const {
		interface::geometry::Point3D p;
		double x=0.0,y=0.0;
		int cnt=0;
		for (auto t: points) {
			x +=  t.x();
			y +=  t.y();
			cnt ++;
		}
		p.set_x(x/cnt);
		p.set_y(y/cnt);

		return p;
	  }
  };
  bool CheckHardBrake(double front_dist) { // front_dist + pedes

	  double v = CalcVelocity(cur_velo_);
	  double acc = cur_acc_.x();

  	  auto points = GetNextPointsFromRoute();
	  for (int i=0;i<obstacle_id_map_.size();i++)  if(!obstacle_history_[i].empty() && pede_speed_[i] > -0.5) {
		  auto q = obstacle_history_[i].back().center();


		  if (v>0.1 && cur_velo_.x()*(cur_pos_.x()-q.x()) + cur_velo_.y()*(cur_pos_.y()-q.y()) >0) continue;

		  double cumu_dist=0.0;
		  double mindist = 1e9;
		  double cumu_dist_at_min = 0.0;
		  for (int j = 0; j < points.size();j ++) {
			  double dist;
			  if (j>1) {
				  auto mid = Midpoint(points[j-1], points[j]);
				  auto m1 = Midpoint(points[j-1],mid);
				  auto m2 = Midpoint(points[j],mid);
				  double thisdist = CalcDistance(points[j-1], points[j]);

				  cumu_dist += 0.25 * thisdist;
				  dist = CalcDistance(m1, q);
				  if (dist<mindist) {
					  mindist = dist;
					  cumu_dist_at_min = cumu_dist;
				  }
				  cumu_dist += 0.25 * thisdist;
				  dist = CalcDistance(mid, q);
				  if (dist<mindist) {
					  mindist = dist;
					  cumu_dist_at_min = cumu_dist;
				  }
				  cumu_dist += 0.25 * thisdist;
				  dist = CalcDistance(m2, q);
				  if (dist<mindist) {
					  mindist = dist;
					  cumu_dist_at_min = cumu_dist;
				  }
				  cumu_dist += 0.25 * thisdist;
			  }
			  dist = CalcDistance(points[j], q);
			  if (dist<mindist) {
				  mindist = dist;
				  cumu_dist_at_min = cumu_dist;
			  }
		  }
		  double vp = 6/3.6;

		  if (pede_speed_[i]>0.0) vp = pede_speed_[i];

		  double termv = v + (acc>0.8? time_to_reach_zero_acc*acc/2:0.0);

		  double safe_dist = (acc>0.03? time_to_reach_zero_acc*(v+vp+time_to_reach_zero_acc/2*acc):0.0 ) + (acc>-max_deceleration+0.03? (termv + vp) * time_to_reach_max_deceleration_upper_bound:0.0) + (termv/2 + vp) * (termv/max_deceleration) + 4.0;

		  if (cumu_dist_at_min>safe_dist) continue;

		  double safe_width = 1.3 + vp*( (acc>0.03?time_to_reach_zero_acc:0.0) + (acc>-max_deceleration+0.03?time_to_reach_max_deceleration_upper_bound:0.0) + (termv/max_deceleration) );

		  if (mindist>safe_width) continue;

		  return true;
	  }

	  double termv = v + (acc>0.8? time_to_reach_zero_acc*acc/2:0.0);
	  double safe_dist = (acc>0.03? time_to_reach_zero_acc*(v+time_to_reach_zero_acc/2*acc):0.0 ) + (acc>-max_deceleration+0.03? (termv) * time_to_reach_max_deceleration_upper_bound:0.0) + (termv/2 ) * (termv/max_deceleration); 
	  if (front_dist<safe_dist) return true;
	  return false;
  }

  double GetFrontObstacleSafeDistance() { // only car
  	  double ans = 1e9;

	  double v = CalcVelocity(cur_velo_);

  	  auto points = GetNextPointsFromRoute();
	  for (int i=0;i<obstacle_id_map_.size();i++)  if(!obstacle_history_[i].empty() && pede_speed_[i] < -0.5) {

		  auto q = obstacle_history_[i].back().center();

		  if(my_name_ == "jcvb2") {
			  std::cout<<"an obstacle: "<<q.x()<<" "<<q.y()<<std::endl;
		  }


		  if (v>0.1 && cur_velo_.x()*(cur_pos_.x()-q.x()) + cur_velo_.y()*(cur_pos_.y()-q.y()) >v * 4.8) continue;

		  double dis = 1e9;

		  double cumu_dist=0.0;
		  double mindist = 1e9;
		  double cumu_dist_at_min = 0.0;
		  for (int j = 0; j < points.size();j ++) {
			  double dist;
			  if (j>1) {
				  auto mid = Midpoint(points[j-1], points[j]);
				  auto m1 = Midpoint(points[j-1],mid);
				  auto m2 = Midpoint(points[j],mid);
				  double thisdist = CalcDistance(points[j-1], points[j]);

				  cumu_dist += 0.25 * thisdist;
				  dist = CalcDistance(m1, q);
				  if (dist<mindist) {
					  mindist = dist;
					  cumu_dist_at_min = cumu_dist;
				  }
				  cumu_dist += 0.25 * thisdist;
				  dist = CalcDistance(mid, q);
				  if (dist<mindist) {
					  mindist = dist;
					  cumu_dist_at_min = cumu_dist;
				  }
				  cumu_dist += 0.25 * thisdist;
				  dist = CalcDistance(m2, q);
				  if (dist<mindist) {
					  mindist = dist;
					  cumu_dist_at_min = cumu_dist;
				  }
				  cumu_dist += 0.25 * thisdist;
			  }
			  dist = CalcDistance(points[j], q);
			  if (dist<mindist) {
				  mindist = dist;
				  cumu_dist_at_min = cumu_dist;
			  }
		  }
		  if(my_name_ == "jcvb2") {
		  	  std::cout<<mindist<<std::endl;
		  	  std::cout<<cumu_dist_at_min<<std::endl;
		  }
		  if (mindist>4) continue;//TODO

		  if (cumu_dist_at_min<ans) {
		  	  ans=cumu_dist_at_min;
		  }
	  }
	  if (my_name_=="jcvb2") {
		  std::cout<< ans<<std::endl;
	  }
	  return ans - 12.0;
  }

  /*bool ObstaclesOnRoute(double front_dist, double safe_dist) {
  	  auto points = GetNextPointsFromRoute();
	  int next_cnt = 0;
	  double cumu_dist=0.0;
	  for (int j = 0; j < points.size();j ++) {
	  	  next_cnt ++;
		  if (j>1) {
			  cumu_dist += CalcDistance(points[j-1], points[j]);
			  if (cumu_dist > front_dist) {
			  	  break;
			  }
		  }
	  }
	  for (int i=0;i<obstacle_id_map_.size();i++)  if(!obstacle_history_[i].empty() && pede_speed_[i]<-0.5) {
		  auto q = obstacle_history_[i].back().center();
		  int meet_cnt = 0;

		  for (int j = 0; j < points.size();j ++) {
			  meet_cnt ++;
		  	  if (CalcDistance(points[j], q) < safe_dist) {
		  	  	  return true;
			  }
			  if (j>1) {
				  auto mid = Midpoint(points[j], points[j-1]);
				  auto m1 = Midpoint(points[j], mid);
				  auto m2 = Midpoint(points[j-1], mid);
				  if (CalcDistance(mid, q) < safe_dist || CalcDistance(m1, q) < safe_dist || CalcDistance(m2, q) < safe_dist) {
					  return true;
				  }
			  }
			  if (meet_cnt >= next_cnt) break;
		  }
	  }
	  return false;
  }*/
  double GetThrottleForAcceleration(double v) {
	  if (v<0.22) return 0.89;
	  else if (v<3.5) return 0.29;
	  else if (v<4.3) return 0.31;
	  else if (v<5) return 0.36;
	  else if (v<6) return 0.34;
	  else if (v<7) return 0.415;
	  else if (v<8) return 0.42;
	  else if (v<10) return 0.52;
	  else return 0.55;
  }
  double GetBrakeDistance(double v, double acc) {
	  if (v>10) return 44.0;
  	  if (v>7) return 32.0;
  	  if (v>5) return 18.0;
  	  if (v>4) return 12.0;
  	  if (v>1) return 6.0;
  	  if (v>0.2) return 1.0;
  	  return 0.5;
  }
  double GetSteeringAngle() {
  	  /*if(temp_) { 6.88888889 speed
		  if (nowtime_>10){
			  if (nowtime_<10.54) return -10*pi;
			  else if (nowtime_ < 14.1) return 10*pi;
			  else if (nowtime_ < 15.3) {
			  	  cur_rou_ = FindRoute();
			  	  return -10*pi;
			  } else {
			  	  temp_ = false;
			  }
		  }
	  }*/

	  auto next_points = GetNextPointsFromRoute();
	  int need_cnt = 30;
	  if (next_points.size() > need_cnt) {
	  	  next_points.resize(need_cnt);
	  }
	  if (next_points.size() > 0) {
		  double temp_a=0.0;
		  double weight = 0.0;
		  for (int i = 0; i < next_points.size(); i ++) {
			  auto next_point = next_points[i];
			  double dx = next_point.x() - cur_pos_.x();
			  double dy = next_point.y() - cur_pos_.y();

			  double a1 = atan2(dy,dx);
			  double a2 = atan2(cur_velo_.y(),cur_velo_.x());
			  double a = a1-a2;
			  while( a > pi) a -= 2*pi;
			  while(a <= -pi) a += 2* pi;
			  
			  double we =  1.0 - i / 40;
			  temp_a += a*we;
			  weight += we;
		  }
		  double a = temp_a / weight;
		  double maxa = 0.5;

		  a = sgn(a)*pow(abs(a),1.3)*3.5*pi;

		  if (std::abs(a)<0.02) a=0.0;
		  
		  return a;
	  }
	  return 0.0;

  }
  class LightInfo {
    public:
  	  double dist;
  	  int last_state;
  	  int this_state;
  	  double time;
  	  LightInfo(){}
  	  LightInfo(double d, int l, int thi, double r):dist(d),last_state(l),this_state(thi),time(r){}
  };

  LightInfo GetLightInfo(const interface::agent::AgentStatus& agent_status) {

	  for (auto traffic_light: agent_status.perception_status().traffic_light()) {
		  for (auto light: traffic_light.single_traffic_light_status()) {
			  auto color = light.color();
			  auto id = light.id().id();
			  if (recent_light_[id] != color) {
			  	  last_light_[id]= recent_light_[id];
			  	  recent_light_[id] = color;
			  	  light_time_[id] = 0.0;
			  }else {
			  	  light_time_[id] += 0.01;
			  }
		  }
	  }

	  int curlaneid = cur_rou_.back().lane_ind;
	  double safe_dist = 3.0;
	  double to_red = 1.1e9;
	  int color = -1, lastcolor=-1;
	  double tim=100.0;
	  if (lane_light_[curlaneid]!="") {
	  	  for (auto traffic_light: agent_status.perception_status().traffic_light()) {
			  for (auto light: traffic_light.single_traffic_light_status()) {
				  if (light.id().id() == lane_light_[curlaneid]) {
					  color = light.color();
					  lastcolor = last_light_[light.id().id()];
					  tim = light_time_[light.id().id()];
				  }
			  }
		  }
		  auto stoppoint = LoadLanePoint(GetEnd(cur_rou_.back()));
		  if (color != interface::map::Bulb::GREEN) {
		  	  to_red = CalcDistance(cur_pos_, stoppoint);
		  	  to_red -= safe_dist;
		  	  to_red = std::max(to_red,0.0);
		  }
	  }
	  return LightInfo(to_red+1e-10, lastcolor, color, tim);
  }
  class LanePoint {
	  public:
	  	  int lane_ind;
	  	  int point_ind;
	  	  LanePoint(){}
	  	  LanePoint(int lane, int point):lane_ind(lane), point_ind(point){}
	  	  int operator<(LanePoint p2)const {
	  	  	  if (lane_ind == p2.lane_ind) {
	  	  	  	  return point_ind < p2.point_ind;
			  } else {
			  	  return lane_ind < p2.lane_ind;
			  }
		  }

	  	  int operator==(LanePoint p2)const {
	  	  	  return lane_ind == p2.lane_ind && point_ind == p2.point_ind;
		  }

	  	  int operator!=(LanePoint p2)const {
	  	  	  return lane_ind != p2.lane_ind || point_ind != p2.point_ind;
		  }


  };
  LanePoint GetEnd(LanePoint p) const { // the endpoint of this lane
	return LanePoint(p.lane_ind, map_.lane()[p.lane_ind].central_line().point_size() - 1);
  }

  LanePoint FindClosestLanePoint(const interface::geometry::Point3D& point)const {
  	  double best_dist = 1e100;
  	  int reti = -1, retj = -1;
	  for (int i = 0; i < map_.lane_size(); i ++) {
	  	  for (int j = 0; j < map_.lane()[i].central_line().point_size(); j ++) {
	  	  	  double dist = CalcDistance(map_.lane()[i].central_line().point()[j], point);
	  	  	  if (dist < best_dist) {
	  	  	  	  best_dist = dist;
	  	  	  	  reti = i;
	  	  	  	  retj = j;
			  }
		  }
	  }
	  return LanePoint(reti, retj);
  }

  LanePoint FindClosestLanePoint(const interface::geometry::Point2D& point)const {
  	  return FindClosestLanePoint(ToPoint3D(point));
  }

  std::vector<LanePoint> FindRoute()const { // back is cur, front is dest
  	  auto position = ToPoint3D(cur_pos_);
  	  auto destination = cur_dest_;
	  std::cout<<position.x()<<" "<<position.y()<<std::endl;
	  std::map<LanePoint, int> dist;
	  std::map<LanePoint, bool> inqueue;
	  std::map<LanePoint, LanePoint> prev;
	  std::deque<LanePoint> q;

	  LanePoint start = FindClosestLanePoint(position);
	  LanePoint end = FindClosestLanePoint(destination);

	  dist[start] = 1;
	  inqueue[start] = true;
	  q.push_back(start);

	  while(!q.empty()) {
	  	  LanePoint u = q.front();
	  	  q.pop_front();
	  	  inqueue[u] = false;

		  std::vector<LanePoint> nex;
	  	  if (u == GetEnd(u)) {
			  for (int j: lane_successors_[u.lane_ind]) {
			  	  nex.push_back(LanePoint(j, 0));
			  }
		  } else {
			  nex.push_back(GetEnd(u));
		  }
		  for (LanePoint v: nex) {
		  	  int w = 0;
		  	  if (v.lane_ind == u.lane_ind) {
		  	  	  w = (v.point_ind - u.point_ind) * lane_curve_[v.lane_ind]; // 1:straight 2,3 curve
			  }
			  if (dist.count(v) == 0 || dist[u] + w < dist[v]) {
				  dist[v] = dist[u] + w;
				  prev[v] = u;
				  if (!inqueue[v]) {
				  	  inqueue[v] = true;
				  	  q.push_back(v);
				  }
			  }
		  }
	  }
	  std::vector<LanePoint> route;
	  route.push_back(end);
	  LanePoint cur(end.lane_ind, 0);

	  if (end != cur) {
	  	  route.push_back(cur);
	  }
	  while (prev.count(cur) > 0) {
		  cur = prev[cur];
	  	  route.push_back(cur);
	  }
	  return route;
  }
  void PopPointFromRoute() {
	  CHECK(cur_rou_.size() >= 1);
	  if (cur_rou_.size() > 1) {
		  if (cur_rou_.back() == GetEnd(cur_rou_.back()) ) {
			  cur_rou_.pop_back();
		  } else {
			  cur_rou_.back().point_ind ++;
			  if (cur_rou_.back() == cur_rou_[cur_rou_.size() - 2]) {
				  cur_rou_.pop_back();
			  }
		  }
	  }
  }

  bool Passed(LanePoint p){
  	  auto p1 = LoadLanePoint(LanePoint(p.lane_ind, 0));
  	  auto p2 = LoadLanePoint(GetEnd(p));
  	  auto p3 = ToPoint3D(cur_pos_);
  	  auto p4 = LoadLanePoint(p);
  	  return ((p2.x()-p1.x())*(p4.x()-p3.x())+(p2.y()-p1.y())*(p4.y()-p3.y()) < 0);
  }
  std::vector<interface::geometry::Point3D> GetNextPointsFromRoute() {
  	  while (cur_rou_.size() > 1) {
  	  	  auto p = cur_rou_.back();
  	  	  if (Passed(p)) {
  	  	  	  PopPointFromRoute();
		  } else {
		  	  break;
		  }
	  }

	  int cnt = 50;

	  std::vector<interface::geometry::Point3D> ret;
	  for (int j = cur_rou_.size() - 1;j>=0 &&  ret.size() < cnt; j --) {
	  	  auto temp = GetEnd(cur_rou_[j]);
	  	  for (int k = cur_rou_[j].point_ind; ret.size() < cnt && k <= temp.point_ind; k ++) {
	  	  	  if (right_) ret.push_back(LoadLanePointRight(LanePoint(temp.lane_ind, k)));
			  else ret.push_back(LoadLanePoint(LanePoint(temp.lane_ind, k)));
		  }
	  }
	  return ret;
  }
  void LoadMap() {
  	  double D = 1.0; // if distance smaller than D, connect them

	  pnc::map::MapLib lib;
	  map_ = lib.map_proto();

	  int num_lane = map_.lane_size();
	  lane_successors_.clear();
	  lane_successors_.resize(num_lane);

	  for (int i = 0; i < map_.lane_size(); i ++) {
		  for (int j = 0; j < map_.lane_size(); j ++) {
		  	  auto& lane1 = map_.lane()[i];
			  //std::cout<<lane1.central_line().point_size()<<'\n';
		  	  auto& lane2 = map_.lane()[j];
			  auto p = lane1.central_line().point()[lane1.central_line().point_size()-1];
			  auto q = lane2.central_line().point()[0];
			  if (CalcDistance(p,q) < D)  {
				  auto *ptr1 = (*map_.mutable_lane())[i].add_successor();
				  ptr1->CopyFrom(lane2.id());
				  auto *ptr2 = (*map_.mutable_lane())[j].add_predecessor();
				  ptr2->CopyFrom(lane1.id());

				  lane_successors_[i].push_back(j);
			  }
		  }
	  }

	  for (int i = 0; i < map_.lane_size(); i ++) {
		  auto& lane1 = map_.lane()[i];
		  auto p = lane1.central_line().point()[0];
		  auto q = lane1.central_line().point()[lane1.central_line().point_size()-1];
		  if (std::abs(p.x() - q.x()) < D || std::abs(p.y() - q.y()) < D) {
		  	  lane_curve_.push_back(STRAIGHT);
		  } else {
		  	  auto r = lane1.central_line().point()[lane1.central_line().point_size()/2];
		  	  double cross = (r.x()-p.x())*(q.y()-r.y()) - (r.y()-p.y())*(q.x()-r.x());
		  	  if (cross>0) lane_curve_.push_back(LEFT);
			  else lane_curve_.push_back(RIGHT);
		  }
	  }


	  lane_light_.clear();
	  last_light_.clear();
	  recent_light_.clear();
	  light_time_.clear();

	  int tot_cnt = 0;
	  for (int i = 0; i < map_.lane_size(); i ++) {
		  auto& lane1 = map_.lane()[i];
		  auto q = lane1.central_line().point()[lane1.central_line().point_size()-1];
		  std::string ans = "";
		  std::string anso = "";
		  int cnt = 0;
		  for (auto light: map_.traffic_light()) {
			  auto p1 = light.stop_line().point()[0], p2 =  light.stop_line().point()[1];
			  interface::geometry::Point3D po;
			  po.set_x((p1.x()+p2.x())*0.5);
			  po.set_y((p1.y()+p2.y())*0.5);
			  if (CalcDistance(po, q) < 2) {
			  	  ans = light.id().id();
			  	  cnt ++ ;
			  	  tot_cnt ++;
			  } else if (std::abs(CalcDistance(po,q) - 34) < 4) {
			  	  anso = light.id().id();
			  }
		  }
		  CHECK(cnt<=1);
		  if(ans=="")anso="";
		  lane_light_.push_back(ans);
		  opposite_light_.push_back(anso);
		  //std::cout<<lane1.id().id()<<" "<<ans<<" "<<anso<<std::endl;
	  }
	  for (auto light: map_.traffic_light()) {
	  	  auto id = light.id().id();
		  recent_light_[id] = interface::map::Bulb::YELLOW;
		  last_light_[id] = -1;
		  light_time_[id] = 0.0;
	  }

	  CHECK(tot_cnt == map_.traffic_light_size());
  }
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination)const {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    return std::sqrt(sqr_sum);
  }

  double CalcDistance(const interface::geometry::Point2D& position,
                      const interface::geometry::Point2D& destination)const {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    return std::sqrt(sqr_sum);
  }
  double CalcDistance(const interface::geometry::Point3D& position,
                      const interface::geometry::Point3D& destination)const {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity)const {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    return std::sqrt(sqr_sum);
  }

  double UpdateVelocityAndGetDerivative(double v) { // called once in every iteration
  	  int num_history = 20;
	  recent_velocities_.push_back(v);
	  if (recent_velocities_.size() >= 2 * num_history + 1) recent_velocities_.pop_front();
	  double deri = 0.0;
	  if (recent_velocities_.size() >= 2 * num_history) {
		  for (int i = 0;i < num_history; i++)deri += + recent_velocities_[i + num_history] - recent_velocities_[i];
		  deri /= num_history;
		  deri /= 0.01 * num_history;
	  }
	  return deri;
  }

  inline interface::geometry::Point3D LoadLanePointRight (LanePoint p) const{
  	  double len = 2.0;
  	  auto q = LoadLanePoint(p);
	  if (lane_curve_[p.lane_ind] == STRAIGHT) {
	  	  auto p1 = LoadLanePoint(LanePoint(p.lane_ind, 0));
	  	  auto p2 = LoadLanePoint(GetEnd(p));
	  	  double di = CalcDistance(p1,p2);
		  interface::geometry::Point3D ret;
		  ret.set_x(q.x() + (p2.y()-p1.y())/di * 2);
		  ret.set_y(q.y() - (p2.x()-p1.x())/di * 2);
		  return ret;
	  } else {
	  	  return q;
	  }
  }
  inline interface::geometry::Point3D LoadLanePoint (LanePoint p) const{
  	  return map_.lane()[p.lane_ind].central_line().point()[p.point_ind];
  }

  /*double DistanceToNextTurn ()const {
	  return 1e9;
  }*/
  int NextTurnType ()const {
  	  for (int i=cur_rou_.size() - 1 ;i >= 0;i --) {
  	  	  if (!LongLane(cur_rou_[i].lane_ind)) {
			  return lane_curve_[cur_rou_[i].lane_ind];
		  }
	  }
	  return 1e9;
  }
  double DistanceToDestination ()const {
  	  double sum = 0.0;
  	  for (int i=cur_rou_.size() - 1 ;i >= 1;i --) {
		  auto p1 = cur_rou_[i], p2 = cur_rou_[i-1];
		  if (p1 == GetEnd(p1) && p2.point_ind == 0) continue;
		  CHECK(p1.lane_ind == p2.lane_ind);
		  if (lane_curve_[p1.lane_ind] == STRAIGHT) {
		  	  sum += CalcDistance(LoadLanePoint(p1), LoadLanePoint(p2));
		  } else {
		  	  double R = std::abs(LoadLanePoint(LanePoint(p1.lane_ind, 0)).x() - LoadLanePoint(GetEnd(p1)).x());
			  double l = CalcDistance(LoadLanePoint(p1), LoadLanePoint(p2));
			  double theta = asin(std::min((l/2)/R,1.0)) * 2;
			  sum += theta* l;
		  }
	  }
	  return sum+1e-10;
  }
  double DistanceToNextStraight ()const {
  	  for (int i=cur_rou_.size() - 1 ;i >= 0;i --) {
  	  	  if (lane_curve_[cur_rou_[i].lane_ind] == STRAIGHT) {
  	  	  	  return CalcDistance(cur_pos_, LoadLanePoint(cur_rou_[i]));
		  }
	  }
	  return 1e9;
  }

  interface::geometry::Point3D GetObstacleVelocity(std::string strid) {
  	  if (obstacle_id_map_.count(strid) == 0) {
  	  	  interface::geometry::Point3D temp;
  	  	  temp.set_x(0.0);
  	  	  temp.set_y(0.0);
  	  	  temp.set_z(0.0);
  	  	  return temp;
	  } else {
	  	  bool is_pede = false;
	  	  if (strid[0]=='P') {
	  	  	  is_pede = true;
		  }
		  int id = obstacle_id_map_[strid];
		  if (obstacle_history_[id].empty()) {
			  interface::geometry::Point3D temp;
			  temp.set_x(0.0);
			  temp.set_y(0.0);
			  temp.set_z(0.0);
			  return temp;
		  }
	  	  if (is_pede) {
			  double speed = 4.5/3.6;
			  if (pede_speed_[id]!=0.0) {
				  speed = pede_speed_[id];
			  }
			  double head = obstacle_heading_[id];
			  interface::geometry::Point3D temp;
			  temp.set_x(cos(head) * speed);
			  temp.set_y(sin(head) * speed);
			  return temp;
		  } else {
			  //TODO 
		  }

	  }
  }
  void UpdateObstacles(const interface::agent::AgentStatus &agent_status) {
  	  if(!agent_status.simulation_status().is_alive()) {
		//if(my_name_ == "jcvb12") std
		  return;
	  }
  	  int keep_cnt = 10;
	  std::set<int> appeared;
	  for (auto obstacle: agent_status.perception_status().obstacle()) {
		  auto strid = obstacle.id();
		  if (obstacle.type() == interface::perception::CAR) {
		  	  strid = std::string("C") + strid;
		  } else {
		  	  strid = std::string("P") + strid;
		  }
		  int id;
		  if (obstacle_id_map_.count(strid) == 0) {
		  	  id = obstacle_id_map_[strid] = obstacle_id_map_.size();
		  	  obstacle_heading_.resize(obstacle_id_map_.size());
		  	  obstacle_history_.resize(obstacle_id_map_.size());
		  	  pede_speed_.resize(obstacle_id_map_.size());
		  	  if (strid[0]=='C') {
		  	  	  pede_speed_[pede_speed_.size() - 1] = -1.0;
			  } else {
		  	  	  pede_speed_[pede_speed_.size() - 1] = 0.0;
			  }
		  } else id = obstacle_id_map_[strid];

		  appeared.insert(id);
		  Polygon poly;
	  	  for (auto point: obstacle.polygon_point() ){
			  poly.points.push_back(point);
		  }
		  obstacle_history_[id].push_back(poly);
		  obstacle_heading_[id] = obstacle.heading();
		  if(my_name_ == "jcvb2") std::cout<<strid<<" "<< poly.center().x()<<" "<<poly.center().y()<< " ";

		  if (obstacle_history_[id].size()>=2 && obstacle.type() == interface::perception::PEDESTRIAN) {
		  	  auto now = poly.center();
		  	  auto prev = obstacle_history_[id][obstacle_history_[id].size()-2].center();
		  	  double dist = CalcDistance(now, prev);
		  	  if (dist>1e-6) {
				  pede_speed_[id] = dist / 0.01;
			  }
		  	  //std::cout<<obstacle.heading()<<" "<< atan2(now.y()-prev.y(),now.x()-prev.x())<<std::endl;
		  }
	  }
	  if(my_name_ =="jcvb2")std::cout<<std::endl;
	  for (int i = 0; i < obstacle_id_map_.size(); i ++) {
	  	  if (!appeared.count(i)) {
	  	  	  obstacle_history_[i].clear();
		  } else {
			  while(obstacle_history_[i].size() > keep_cnt) {
			  	  obstacle_history_[i].pop_front();
			  }
		  }
	  }
  }
  bool LongLane(int id) const {
	return CalcDistance(LoadLanePoint(LanePoint(id, 0)), LoadLanePoint(GetEnd(LanePoint(id, 0)) )) > 60;
  }

  bool PointOnLane(interface::geometry::Point3D q, int id) {
	  auto p1 = LoadLanePoint(LanePoint(id,0));
	  auto p2 = LoadLanePoint(GetEnd(LanePoint(id,0)));
	  if (lane_curve_[id] == STRAIGHT) {
		  if (std::abs(p1.x()-p2.x())<2.0) {
		  	  if (std::abs(q.x()-p1.x())>3.0) return false;
		  	  if (!in_between(q.y(),p1.y(),p2.y())) return false;
		  	  return true;
		  } else {
		  	  if (std::abs(q.y()-p1.y())>3.0) return false;
		  	  if (!in_between(q.x(),p1.x(),p2.x())) return false;
		  	  return true;
		  }
	  } else {
		  double R = std::abs(p1.x()-p2.x());
	  	  auto mid = Midpoint(p1,p2);
		  interface::geometry::Point3D center;
		  if (lane_curve_[id] == LEFT) {
			  center.set_x(mid.x() -(p2.y()-mid.y()));
			  center.set_y(mid.y()+p2.x()-mid.x());
		  } else {
			  center.set_x(mid.x() +(p2.y()-mid.y()));
			  center.set_y(mid.y()-(p2.x()-mid.x()));
		  }
		  return std::abs(CalcDistance(center, q) - R) < 6.0;
	  }
  }
  bool ObstaclesOnRoute(int obsid, int laneid) {
  	  if (obstacle_history_[obsid].empty()) return false;
  	  if (!PointOnLane(obstacle_history_[obsid].back().center(), laneid)) return false;
	  double alpha = obstacle_heading_[obsid];
	  double x = cos(alpha), y= sin(alpha);
	  auto p1 = LoadLanePoint(LanePoint(laneid,0));
	  auto p2 = LoadLanePoint(GetEnd(LanePoint(laneid,0)));
	  return (p2.x()-p1.x())*x + (p2.y()-p1.y())*y > 0;
  }
  interface::map::Map map_;
  std::vector<std::vector<int> > lane_successors_;
  std::vector<int> lane_curve_; //1: straight, 2,3: curved
  std::vector<std::string> lane_light_; //id of traffic light at the end of lane or empty string
  std::vector<std::string> opposite_light_;
  std::vector<LanePoint> cur_rou_;
  std::deque<double> recent_velocities_;
  interface::geometry::Vector3d cur_pos_;
  interface::geometry::Point3D cur_dest_;
  interface::geometry::Vector3d cur_velo_;
  interface::geometry::Vector3d cur_acc_;

  std::map<std::string, int> obstacle_id_map_;
  std::vector<std::deque<Polygon> > obstacle_history_;
  std::vector<double> obstacle_heading_;
  std::vector<double> pede_speed_; //deterministic. -1 if it is a car

  std::map<std::string, int> last_light_;
  std::map<std::string, int> recent_light_;
  std::map<std::string, double> light_time_;

  std::string my_name_;
  double nowtime_;
  bool right_ = false;

  bool at_crossing_ = false;

  bool temp_ = true;
};

}  // namespace jcvb





// TODO
// zhangai de sudu, acc, pos
// duimian de deng
// how to jiasu to any speed? (qidong)
// an honglvdeng zuo zuiduanlu
// zuoyouyaobai (curva)
// changshijianbudong : replan
//  diaotou
//  sisuo
// wait at crossing // (pengzhuang )
