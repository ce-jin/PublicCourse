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

double sigmoid(double x){
	return 1/(1+exp(-x));
}
int sgn(double x){
	if (x>0)return 1;
	else return -1;
}
inline interface::geometry::Point3D ToPoint3D (interface::geometry::Point2D p){
		  interface::geometry::Point3D pt;
		  pt.set_x(p.x());
		  pt.set_y(p.y());
		  pt.set_z(0);
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
	  cur_rou_ = FindRoute(ToPoint3D(agent_status.vehicle_status().position()), agent_status.route_status().destination());
  }

  interface::control::ControlCommand RunOneIteration(
		  const interface::agent::AgentStatus& agent_status) override {

	  cur_pos_ = agent_status.vehicle_status().position();
	  cur_velo_ = agent_status.vehicle_status().velocity();
	  cur_acc_ = agent_status.vehicle_status().acceleration_vcs();

	  UpdateObstacles(agent_status);

	  if (agent_status.route_status().is_new_request()) {
		  cur_rou_ = FindRoute(ToPoint3D(agent_status.vehicle_status().position()), agent_status.route_status().destination());
		  position_reached_destination_ = false;
	  }

	  double v = CalcVelocity(cur_velo_);
	  double dv = UpdateVelocityAndGetDerivative(v);

	  interface::control::ControlCommand command;

	  double to_dest = DistanceToDestination();

	  double brake_dist = 40.0;

          if(to_dest < brake_dist) {
		  position_reached_destination_ = true;
	  }

	  double to_red = GetLightInfo(agent_status).dist;

	  if ((position_reached_destination_ || to_red < brake_dist) && (v> 0.1 || -cur_acc_.x()<-0.5)) {
	  	  double del = (v*v/(-cur_acc_.x())/2) / (std::min(to_dest, to_red)+1e-10);
	  	  if (del<0.0) del = 2.0;
		  //std::cout<<del<<std::endl;
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
	          if(std::min(to_dest, to_red)<1.0) {
			  command.set_brake_ratio(0.5);
			  command.set_throttle_ratio(0.0);
		  }else if (v < speed_limit - 0.55) {
			  command.set_throttle_ratio(GetThrottleForAcceleration(v));
		  }
	  }

	  command.set_steering_angle(GetSteeringAngle());


	  if (ObstaclesOnRoute(20, 3.0)) {
			  command.set_brake_ratio(1.0); // hard brake
			  command.set_throttle_ratio(0.0);
			  //std::cout<<"hard"<<std::endl;
			  PublishVariable("hard", "hard", utils::display::Color::Red());
	  } else {


	  }

	  //std::cout<<command.brake_ratio()<<" "<<command.throttle_ratio()<<std::endl;

	  PublishVariable("key1", map_.lane()[cur_rou_.back().lane_ind].id().id());
	  PublishVariable("key2", my_name_, utils::display::Color::Red());

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
	  //std::cout<<my_name_<<" "<<cur_pos_.x()<<" "<<cur_pos_.y()<<std::endl;
	  std::cout<<my_name_<<" "<<agent_status.vehicle_status().position().x()<<" "<<agent_status.vehicle_status().position().y()<<std::endl;
	  PublishVariable("key3", "var3", utils::display::Color::Red(), utils::display::Color::Green());
	  //std::cout<< agent_status.route_status().destination().x() <<' '<< agent_status.route_status().destination().y()<<'\n';

	  return command;
  }

 private:
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
  bool ObstaclesOnRoute(int next_cnt, double safe_dist) {
  	  auto points = GetNextPointsFromRoute();
	  
	  for (int i=0;i<obstacle_id_map_.size();i++)  if(!obstacle_history_[i].empty()) {
		  auto q = obstacle_history_[i].back().center();
		  int meet_cnt = 0;

		  for (int j = 0; j < points.size();j ++) {
			  meet_cnt ++;
		  	  if (CalcDistance(points[j], q) < safe_dist) {
		  	  	  return true;
			  }
			  if (j>1) {
				  interface::geometry::Point3D mid;
				  mid.set_x((points[j].x()+points[j-1].x())*0.5);
				  mid.set_y((points[j].y()+points[j-1].y())*0.5);
				  if (CalcDistance(mid, q) < safe_dist) {
					  return true;
				  }
			  }
			  if (meet_cnt >= next_cnt) break;
		  }
	  }
	  return false;
  }
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
  double GetSteeringAngle() {
	  auto next_points = GetNextPointsFromRoute();
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
  	  double remain_time;
  	  LightInfo(){}
  	  LightInfo(double d, int l, double r):dist(d),last_state(l),remain_time(r){}
  };

  LightInfo GetLightInfo(const interface::agent::AgentStatus& agent_status) {
	  int curlaneid = cur_rou_.back().lane_ind;
	  double safe_dist = 3.0;
	  double to_red = 1.1e9;
	  if (lane_light_[curlaneid]!="") {
	  	  int color;
	  	  for (auto traffic_light: agent_status.perception_status().traffic_light()) {
			  for (auto light: traffic_light.single_traffic_light_status()) {
				  if (light.id().id() == lane_light_[curlaneid]) {
					  color = light.color();
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
	  return LightInfo(to_red, 0, 100.0);
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

  std::vector<LanePoint> FindRoute(const interface::geometry::Point3D& position, const interface::geometry::Point3D& destination)const { // back is cur, front is dest
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
		  	  	  w = (v.point_ind - u.point_ind) * lane_curve_[v.lane_ind];
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

	  int cnt = 30;

	  std::vector<interface::geometry::Point3D> ret;
	  for (int j = cur_rou_.size() - 1;j>=0 &&  ret.size() < cnt; j --) {
	  	  auto temp = GetEnd(cur_rou_[j]);
	  	  for (int k = cur_rou_[j].point_ind; ret.size() < cnt && k <= temp.point_ind; k ++) {
	  	  	  ret.push_back(LoadLanePoint(LanePoint(temp.lane_ind, k)));
		  }
	  }
	  // TODO
	  /*while(ret.size() < cnt) {
	 	ret.push_back(ret.back());
	  }*/
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
		  	  lane_curve_.push_back(1);
		  } else {
		  	  lane_curve_.push_back(2);
		  }
	  }

	  lane_light_.clear();
	  int tot_cnt = 0;
	  for (int i = 0; i < map_.lane_size(); i ++) {
		  auto& lane1 = map_.lane()[i];
		  auto q = lane1.central_line().point()[lane1.central_line().point_size()-1];
		  std::string ans = "";
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
			  }
		  }
		  CHECK(cnt<=1);
		  lane_light_.push_back(ans);
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

  inline interface::geometry::Point3D LoadLanePoint (LanePoint p) const{
  	  return map_.lane()[p.lane_ind].central_line().point()[p.point_ind];
  }

  double DistanceToNextTurn ()const {
  	  for (int i=cur_rou_.size() - 1 ;i >= 0;i --) {
  	  	  if (lane_curve_[cur_rou_[i].lane_ind] == 2) {
  	  	  	  return CalcDistance(cur_pos_, LoadLanePoint(cur_rou_[i]));
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
		  if (lane_curve_[p1.lane_ind] == 1) {
		  	  sum += CalcDistance(LoadLanePoint(p1), LoadLanePoint(p2));
		  } else {
		  	  double R = std::abs(LoadLanePoint(LanePoint(p1.lane_ind, 0)).x() - LoadLanePoint(GetEnd(p1)).x());
			  double l = CalcDistance(LoadLanePoint(p1), LoadLanePoint(p2));
			  double theta = asin(std::min((l/2)/R,1.0)) * 2;
			  sum += theta* l;
		  }
	  }
	  return sum;
  }
  double DistanceToNextStraight ()const {
  	  for (int i=cur_rou_.size() - 1 ;i >= 0;i --) {
  	  	  if (lane_curve_[cur_rou_[i].lane_ind] == 1) {
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
		  } else id = obstacle_id_map_[strid];

		  appeared.insert(id);
		  Polygon poly;
	  	  for (auto point: obstacle.polygon_point() ){
			  poly.points.push_back(point);
		  }
		  obstacle_history_[id].push_back(poly);
		  obstacle_heading_[id] = obstacle.heading();
		  //if(my_name_ == "jcvb12") std::cout<<strid<<" "<< poly.center().x()<<" "<<poly.center().y()<< " ";

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
	  //if(my_name_ =="jcvb12")std::cout<<std::endl;
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
  // Whether vehicle's current position reaches the destination
  bool position_reached_destination_ = false;

  interface::map::Map map_;
  std::vector<std::vector<int> > lane_successors_;
  std::vector<int> lane_curve_; //1: straight, 2: curved
  std::vector<std::string> lane_light_; //id of traffic light at the end of lane or empty string
  std::vector<LanePoint> cur_rou_;
  std::deque<double> recent_velocities_;
  interface::geometry::Vector3d cur_pos_;
  interface::geometry::Vector3d cur_velo_;
  interface::geometry::Vector3d cur_acc_;

  std::map<std::string, int> obstacle_id_map_;
  std::vector<std::deque<Polygon> > obstacle_history_;
  std::vector<double> obstacle_heading_;
  std::vector<double> pede_speed_; //deterministic
  std::string my_name_;
};

}  // namespace jcvb





// TODO
// zhangai de sudu, acc, pos
// change brake dist according to current speed
// hardbrake: next an distance instead of cnts
// how to stop when close
// wait at crossing
// (pengzhuang )
// buxuyao deng de honglvdeng
// how to jiasu to any speed? (qidong)
// brake dist ~ 15m
// zuoyouyaobai (curva)
// changshijianbudong : replan
// an honglvdeng zuo zuiduanlu
// acce (tingche, shache)

