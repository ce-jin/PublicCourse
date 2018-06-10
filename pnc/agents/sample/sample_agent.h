// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"
//#include<bits/stdc++.h>
#include<iostream>
#include<unordered_map>
#include<gflags/gflags.h>
#include<glog/logging.h>
#include<QtWidgets/QApplication>
#include"common/proto/object_labeling_3d.pb.h"
#include"common/utils/display/painter_widget_base.h"
#include"common/utils/file/file.h"
#include"common/utils/file/path.h"
#include"common/utils/strings/format.h"
#include"common/proto/map_lane.pb.h"
#include"common/proto/map.pb.h"
#include"common/proto/geometry.pb.h"
#include"common/utils/file/file.h"
#include"homework5/map/map_lib.h"
#include"common/proto/route.pb.h"
#define rep(i,a,n) for(int i=(a);i<=(n);++i)
#define dep(i,a,n) for(int i=(a);i>=(n);--i)
#define LOCAL
#ifdef LOCAL
#define debug(...) fprintf(stderr, __VA_ARGS__),fflush(stderr)
#else
#define debug(...) 
#define assert(...) 
#endif
#define mp make_pair
#define pb push_back
#define X first
#define Y second
#define tm tmz
#define SZ(x) ((int)(x).size())
using namespace std;
typedef pair<double,double> pdd;
typedef double lf;

namespace xllend3 {

// A sample vehicle agent for route 1
// This agent will always run in straight. It will accelerate to the speed slightly over 5m/s,
// and keeps running with the speed over 5m/s until reaches destination.


pdd operator-=(pdd&a,const pdd &b){return a=mp(a.X-b.X,a.Y-b.Y);}
pdd operator+=(pdd&a,const pdd &b){return a=mp(a.X+b.X,a.Y+b.Y);}
pdd operator*=(pdd&a,const double &b){return a=mp(a.X*b,a.Y*b);}
pdd operator/=(pdd&a,const double &b){return a=mp(a.X/b,a.Y/b);}
pdd operator*(const pdd &a,const double &b){return mp(a.X*b,a.Y*b);}
pdd operator/(const pdd &a,const double &b){return mp(a.X/b,a.Y/b);}
pdd operator+(const pdd &a,const pdd &b){return mp(a.X+b.X,a.Y+b.Y);}
pdd operator-(const pdd &a,const pdd &b){return mp(a.X-b.X,a.Y-b.Y);}
double len(const interface::geometry::Vector3d& a) {
	return sqrt(math::Sqr(a.x())+math::Sqr(a.y())+math::Sqr(a.z()));
}
double len(double x,double y){return sqrt(math::Sqr(x)+math::Sqr(y));}
double len(pair<double,double>x){return len(x.X,x.Y);}
pdd turn90(pdd a){return mp(-a.Y,a.X);}
pdd norm(pdd a){return a/len(a);}
double cross(pdd a,pdd b){return a.X*b.Y-a.Y*b.X;}
double cross(pdd a,pdd b,pdd c){return cross(b-a,c-b);}
pdd to_pdd(interface::geometry::Point3D a){return mp(a.x(),a.y());}
pdd to_pdd(interface::geometry::Point2D a){return mp(a.x(),a.y());}
class SampleVehicleAgent : public simulation::VehicleAgent {
public:
	string myname;
	explicit SampleVehicleAgent(const std::string& name) : VehicleAgent(name) {myname=name;}
	virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
		//fprintf(stderr,"1\n");
		auto a=interface::map::Map(pnc::map::MapLib().map_proto());
		MAP=a;
		V.clear();
		//fprintf(stderr,"2\n");
		for(auto i:a.lane())for(auto j:a.lane()){
			auto I=i.central_line().point()[i.central_line().point_size()-1];
			auto J=j.central_line().point()[0];
			if(check(I,J))ae(get(I),get(J)),ae2(to_pdd(I),to_pdd(J));
		}
		for(auto i:a.lane()){
			int sz=i.central_line().point_size();
			vector<pdd>V;
			rep(j,0,sz-1)V.pb(to_pdd(i.central_line().point()[j]));
			lf l=0;
			rep(j,0,sz-2)l+=len(to_pdd(i.central_line().point()[j])-to_pdd(i.central_line().point()[j+1]));
			ae(get(i.central_line().point()[0]),get(i.central_line().point()[sz-1]),l);
			rep(j,1,sz-3)ae(get(i.central_line().point()[j]),get(i.central_line().point()[j+1]));
			rep(j,0,sz-2)ae2(to_pdd(i.central_line().point()[j]),to_pdd(i.central_line().point()[j+1]));
			rep(j,0,sz-1){
				auto p=to_pdd(i.central_line().point()[j]);
				pii_to_pdd[mp(int(p.X),int(p.Y))]=p;
			}
			/*
			dep(j,sz-2,0){
				l+=len(to_pdd(i.central_line().point()[j])-to_pdd(i.central_line().point()[j+1]));
				ae(get(i.central_line().point()[j]),get(i.central_line().point()[sz-1]),l+0.1);
			}
			l=0;
			rep(j,1,sz-1){
				l+=len(to_pdd(i.central_line().point()[j])-to_pdd(i.central_line().point()[j-1]));
				ae(get(i.central_line().point()[0]),get(i.central_line().point()[j]),l+0.1);
			}
			*/
			point_to_lane[to_pdd(i.central_line().point()[0])].pb(V);
			point_to_lane[to_pdd(i.central_line().point()[sz-1])].pb(V);
			//debug("%.9lf %.9lf\n",i.central_line().point()[0].x(),i.central_line().point()[0].y());
			//debug("%.9lf %.9lf\n",i.central_line().point()[sz-1].x(),i.central_line().point()[sz-1].y());
		}
		//fprintf(stderr,"3\n");
	}
	struct ROUTE{
		vector<pdd>V;int now_dest=0;
		void clear(){V.clear();now_dest=0;}
		void add_point(pdd x){V.pb(x);}
		int is_last_point(){return now_dest==SZ(V)-1;}
		pdd get_point(){return V[now_dest];}
		pdd dest_point(){return V[SZ(V)-1];}
		int is_last_num(int x){return now_dest>=SZ(V)-x;}
		void update_point(pdd x){while(!is_last_point()&&len(get_point()-x)<7)now_dest++;}
	}R0,R1;
	virtual void find_path(const interface::agent::AgentStatus& agent_status) {
		if(!agent_status.route_status().is_new_request())return;
		//debug("find_path begin\n");
		dis.clear();pre.clear();
		interface::geometry::Point2D st,en;
		st.set_x(agent_status.vehicle_status().position().x());
		st.set_y(agent_status.vehicle_status().position().y());
		en.set_x(agent_status.route_status().destination().x());
		en.set_y(agent_status.route_status().destination().y());
		queue<pdd>Q;
		map<pdd,lf>inq;
		for(auto i:MAP.lane()){
			int o1=0,o2=0,sz=i.central_line().point_size();
			rep(j,0,sz-1){
				if(dis2(st,get(i.central_line().point()[j]))<5)o1=1,ae(st,get(i.central_line().point()[j]));
				if(dis2(en,get(i.central_line().point()[j]))<5)o2=1,ae(get(i.central_line().point()[j]),en);
			}
			if(o1)ae(st,get(i.central_line().point()[sz-1]));
			if(o2)ae(get(i.central_line().point()[0]),en);
		}
		Q.push(tp(st));dis[tp(st)]=1;
		while(!Q.empty()){
			auto k=Q.front();Q.pop();inq[k]=0;
			for(auto i:V[k])if(dis[i.X]==0||dis[i.X]>dis[k]+i.Y){
				dis[i.X]=dis[k]+i.Y;
				pre[i.X]=k;
				if(!inq[i.X])Q.push(i.X),inq[i.X]=1;
			}
		}
		//fprintf(stderr,"%.3lf %.3lf %.3lf %.3lf\n",st.x(),st.y(),en.x(),en.y());
		R0.clear();
		for(auto i=tp(en),j=i;i!=tp(st);j=i,i=pre[i]){
			if(i==j)continue;
			//R0.add_point(i);continue;
			if(fabs(i.X-j.X)<2||fabs(i.Y-j.Y)<2){
				int px=int(fabs(i.X+i.Y-j.X-j.Y)*1.1);
				rep(_,0,px)R0.add_point((j*(px-_)+i*_)/px);
			}else{
				//debug("I: %.9lf %.9lf\n",i.X,i.Y);
				//debug("J: %.9lf %.9lf\n",j.X,j.Y);
				int ok=0;
				for(auto k:point_to_lane[i]){
					//debug("ROUTE\n");
					//for(auto l:k)debug("(%.9lf,%.9lf)\n",l.X,l.Y);
					if(k[0]==i&&k[SZ(k)-1]==j){
						dep(i,SZ(k)-1,0)R0.add_point(k[i]);
						ok=1;
					}
				}
				if(ok==0){
					debug("%.9lf %.9lf\n",i.X,i.Y);
					debug("%.9lf %.9lf\n",j.X,j.Y);
					assert(0);
				}
			}
		}
		reverse(R0.V.begin(),R0.V.end());
		//rep(i,0,SZ(R0.V)-1)debug("%.9lf %.9lf\n",R0.V[i].X,R0.V[i].Y);
		//exit(0);
	/*
		int same_line[999],n=SZ(R0.V)-2,nxt1[999],pre1[999];
		rep(i,1,n)same_line[i]=fabs(cross(R0.V[i-1],R0.V[i],R0.V[i+1]))<0.01;
		nxt1[n]=same_line[n];dep(i,n-1,1)nxt1[i]=same_line[i]?nxt1[i+1]+1:0;
		pre1[1]=same_line[1];rep(i,2,n)pre1[i]=same_line[i]?pre1[i-1]+1:0;
		R1=R0;
		rep(i,1,n)if(pre1[i]>50){
			if(nxt1[i]<=50&&nxt1[i]>=25)R0.V[i]+=norm(turn90((R0.V[i+1]-R0.V[i])))*5*(1-(50-nxt1[i])/25.0);
			else if(nxt1[i]<25)R0.V[i]+=norm(turn90((R0.V[i+1]-R0.V[i])))*5;
		}else if(i>=5&&pre1[i-5]>50)R0.V[i]+=norm(turn90((R0.V[i-4]-R0.V[i-5])))*5;
	*/
		//rep(i,1,n)printf("%d",same_line[i]);puts("");
		//fprintf(stderr,"OK\n");
		f=fopen("/home/zxy/Driving/PublicCourse/pnc/data/speed_data.txt","w");
	}
	int counter=0,count_ag=130,count_ag2=-30;
	pair<double,double> init_pos;
	double init_velo;
	double can_pass_until=-1e9;
	int check_light(double tm,int ty){
		tm-=int(tm/46)*46;
		if(ty==0)return 0.1<=tm&&tm<=25.9;
		if(ty==1)return 23.1<=tm&&tm<=46||tm<=2.9;
		assert(0);
	}
	inline lf calc_dis(lf v,lf a){
		if(a<0)a=0;
		return 0.02+0.281705276*v+0.047095533*v*v+v*(a*a*0.4);
	}
	inline lf get_colli_time(lf x){
		return min(4.0,max(0.0,+1.115613314+0.354334236*x-0.020680243*x*x+0.000520018*x*x*x));
	}
	pdd get_closest_path_point(pdd x){
		int px=int(x.X),py=int(x.Y);
		auto ans=x+mp(1e9,0);
		rep(i,px-1,px+1)rep(j,py-1,py+1)if(len(pii_to_pdd[mp(i,j)])>1e-6){
			auto p=pii_to_pdd[mp(i,j)];
			ae2(x,p);
			if(len(p-x)<len(ans-x))ans=p;
		}
		if(ans==x+mp(1e9,0)){
			debug("ERROR %.9lf %.9lf\n",x.X,x.Y);//assert(0);
		}
		return ans;
	}
	pdd get_colli_dist(pdd x,pdd y){
		auto yy=get_closest_path_point(y);
		//if(xx.X>1e6||yy.X>1e6)return mp(100,100);
		//ae2(x,xx);ae2(y,yy);
		map<pdd,pair<int,lf> >d1,d2;
		queue<pdd>Q;
		lf s=0;pdd pre=x;int cnt=0;
		rep(i,R0.now_dest,min(SZ(R0.V)-1,R0.now_dest+30)){
			s+=len(pre-R0.V[i]);
			pre=R0.V[i];
			d1[R0.V[i]]=mp(++cnt,s);
		}
		Q.push(y);d2[y]=mp(1,0);
		while(!Q.empty()){
			auto k=Q.front();Q.pop();
			if(d2[k].X>30)break;
			for(auto j:V_full[k]){
				if(d2[j.X].X==0){
					d2[j.X]=mp(d2[k].X+1,d2[k].Y+j.Y);
					Q.push(j.X);
				}
			}
		}
		/*
		if(myname=="9"){
			for(auto i:d1)debug("%.9lf %.9lf %d %.9lf\n",i.X.X,i.X.Y,i.Y.X,i.Y.Y);
			for(auto i:d2)debug("%.9lf %.9lf %d %.9lf\n",i.X.X,i.X.Y,i.Y.X,i.Y.Y);
			puts("");
		}
		*/
		pdd ans=mp(100,100);
		for(auto i:d1)for(auto j:d2)if(len(i.X-j.X)<4){
			if(i.Y.Y+j.Y.Y<ans.X+ans.Y)ans=mp(i.Y.Y,j.Y.Y);
		}
		return ans;
	}
	virtual interface::control::ControlCommand RunOneIteration(const interface::agent::AgentStatus& agent_status) {
		find_path(agent_status);
		auto posx=agent_status.vehicle_status().position().x();
		auto posy=agent_status.vehicle_status().position().y();
		auto velox=agent_status.vehicle_status().velocity().x();
		auto veloy=agent_status.vehicle_status().velocity().y();
		auto lenv=len(agent_status.vehicle_status().velocity());
		auto accx=agent_status.vehicle_status().acceleration_vcs().x();
		auto accy=agent_status.vehicle_status().acceleration_vcs().y();
		auto lenacc=len(agent_status.vehicle_status().acceleration_vcs());
		auto oriz=agent_status.vehicle_status().orientation().z();
		auto oriw=agent_status.vehicle_status().orientation().w();
		auto oriangle=atan2(oriz,oriw);
		auto orix=cos(oriangle*2);
		auto oriy=sin(oriangle*2);
		auto time0=agent_status.simulation_status().simulation_time();
		double light_dis=1e9;
		int light_type=0;
		auto omega=agent_status.vehicle_status().angular_velocity_vcs().z();
		//debug("%.9lf %.9lf %.9lf %.9lf %.9lf\n",oriz,oriw,oriangle,orix,oriy);
		vector<pair<int,pdd> >V_obs;
		int ok=0,danger=0;
		interface::control::ControlCommand command;
		for(auto i:agent_status.perception_status().obstacle()){
			double avgx=0,avgy=0,avgz=0;int num=0;
			for(auto j:i.polygon_point()){
				num++;avgx+=j.x();avgy+=j.y();avgz+=j.z();
			}
			avgx/=num;avgy/=num;avgz/=num;
			avgx-=posx;avgy-=posy;
			auto d1=(avgx*orix+avgy*oriy);
			auto d2=fabs(avgx*oriy-avgy*orix);
			int on_path=0;
			if(i.type()==1){//car
				auto p=get_colli_dist(mp(posx,posy),mp(avgx+posx,avgy+posy));
				if(myname=="9"){
					debug("%.9lf %.9lf\n",p.X,p.Y);
				}
				int dan=1;
				if(p.X+p.Y>100)dan=0;
				if(p.X-p.Y<-4&&lenv>10)dan=0;
				rep(j,0,SZ(R0.V)-1)if(len(R0.V[j]-mp(avgx+posx,avgy+posy))<1)on_path=1;
				if(!on_path&&dan){
					if(lenv>5)command.set_brake_ratio(min(1.0,(lenv-4)*0.1)),ok=1;
				}
				if(on_path||dan)V_obs.pb(mp(i.type(),mp(d1,d2)));
			}else V_obs.pb(mp(i.type(),mp(d1,d2)));

			auto psp=6/3.6;
			auto ztime=min(get_colli_time(d1),d1/lenv);
			if(i.id()=="P288"){
				debug("%.9lf %.9lf %.9lf %.9lf %.9lf\n",lenv,accx,calc_dis(lenv,accx),d1,(ztime*psp-(d2-1))*psp);
			}
		}
		if(fabs(orix)<0.1||fabs(oriy)<0.1)for(auto i:agent_status.perception_status().traffic_light()){
			int type;double dis=1e60;
			for(auto k:MAP.traffic_light()){
				auto ax=(k.stop_line().point(1).x()+k.stop_line().point(0).x())/2;
				auto ay=(k.stop_line().point(1).y()+k.stop_line().point(0).y())/2;
				ax-=posx;ay-=posy;
				auto d1=(ax*orix+ay*oriy);
				auto d2=fabs(ax*oriy-ay*orix);
				if(d2<3&&d1>0){
					type=fabs(k.stop_line().point(1).x()-k.stop_line().point(0).x())<1e-3?0:1;
					if(d1<dis)dis=d1;
				}
			}
			if(dis==1e60)continue;
			auto t1=time0;while(t1>46)t1-=46;
			int clr=0;
			if(t1<3||t1>23&&t1<26)clr=1;
			else if((clr<24)^type)clr=2;
			light_dis=dis;
			light_type=type;
			//debug("%d %.9lf\n",clr,time0);
		}
		//fprintf(stderr,"%d\n",V_obs.size());
		//fprintf(stderr,"%.3lf %.3lf %.3lf\n",agent_status.vehicle_status().position().x(),agent_status.vehicle_status().position().y(),agent_status.vehicle_status().position().z());
		/*
		fprintf(f,"%.15lf\n",len(agent_status.vehicle_status().velocity()));
		fprintf(stderr,"  V%.3lf\n",len(agent_status.vehicle_status().velocity()));
		fprintf(stderr,"ACC%.3lf\n",len(agent_status.vehicle_status().acceleration_vcs()));
		fprintf(stderr,"%d %u %.3lf\n",now_dest,route_points.size(),dis2(get(route_points[now_dest]),get(agent_status.vehicle_status().position())));
		fprintf(stderr,"%3lf\n",CalcDistance(agent_status.vehicle_status().position(),agent_status.route_status().destination()));
		*/
		//debug("%d %d\n",now_dest,route_points.size());
		R0.update_point(mp(posx,posy));
		//if(!R0.is_last_point()&&dis2(get(R0.get_point()),get(agent_status.vehicle_status().position()))<50)R0.
		if(counter==0)init_pos=mp(posx,posy),counter=1;
		
		//debug("%d %d %.9lf %9lf\n",now_dest,route_points.size()-10,dis2(get(route_points[route_points.size()-1]),get(agent_status.vehicle_status().position())),lenv);
		//decide throttle or brake
		for(auto i2:V_obs){
			auto psp=6/3.6;
			auto i=i2.Y;
			auto ztime=min(get_colli_time(i.X),i.X/lenv);
			if(i2.X==2){//pedestrian
				if(i.X<3.8)continue;
				if(ztime*psp>=i.Y-1||(fabs(omega)>0.1)&&ztime*max(lenv,5.0)>=i.Y-1){
					if(i.X<calc_dis(lenv,accx)+max(0.0,ztime*psp-(i.Y-1))+5)command.set_brake_ratio(1.0),ok=1;
					else if(i.X<calc_dis(lenv,accx)+(ztime*psp-(i.Y-1))*psp+15){
						//danger=1;
					}
				}
			}else if(i2.X==1){//car
				if(i.X<3.8)continue;
				//if(2>=i.Y||(fabs(omega)>0.1)&&ztime*max(lenv,5.0)>=i.Y-1){
				if(ztime*max(lenv,5.0)>=i.Y-1){
					if(i.X<calc_dis(lenv,accx)+3.8+2.4*2+3)command.set_brake_ratio(1.0),ok=1;
					else if(i.X<calc_dis(lenv,accx)+25){
						//danger=1;
					}
				}
			}
		}
		//debug("danger%d\n",danger);
		if(!ok){
			if(light_dis-23<calc_dis(lenv,accx)){
				int can_pass=0;
				if(can_pass_until>time0)can_pass=1;
				if(lenv>10){
					if(check_light(time0+light_dis/lenv,light_type))can_pass=1;
				}else{
					if(light_dis<20||lenv>1){
						if(check_light(time0,light_type)&&check_light(time0+4,light_type))can_pass=1;
					}else{
						if(lenv<0.1&&check_light(time0+(light_dis-20)/12.0+4,light_type))can_pass_until=time0+4;
						if(check_light(time0,light_type)&&check_light(time0+(light_dis-20)/14.0+4,light_type))can_pass=1;
					}
				}
				if(!can_pass){
					ok=1;
					command.set_brake_ratio(1.0);
				}
			}else{
				
			}
		}
		//if(ok)debug("(SHACHE)");
		/*
		if (CalcVelocity(agent_status.vehicle_status().velocity()) > 4) ok=1;
		if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5) ok=1, command.set_brake_ratio(1.0);
		*/
		if(!ok){
			//debug("%.9lf\n",sqrt(dis2(get(R0.dest_point()),get(agent_status.vehicle_status().position()))));
			if (sqrt(dis2(get(R0.dest_point()),get(agent_status.vehicle_status().position())))<(calc_dis(lenv,accx))+20)danger=1;
			if (R0.is_last_num(30)&&sqrt(dis2(get(R0.dest_point()),get(agent_status.vehicle_status().position())))<calc_dis(lenv,accx)+0.5) command.set_brake_ratio(1.0);
			else if (!danger&&CalcVelocity(agent_status.vehicle_status().velocity()) < 13) command.set_throttle_ratio(1.0);
			else if (CalcVelocity(agent_status.vehicle_status().velocity()) < 1) command.set_throttle_ratio(0.3);
			else if (CalcVelocity(agent_status.vehicle_status().velocity()) < 2) command.set_throttle_ratio(0.2);
			else if (CalcVelocity(agent_status.vehicle_status().velocity()) < 3) command.set_throttle_ratio(0.1);
		}
		
		pdd now_destination=R0.get_point();
		double angle=atan2(now_destination.Y-agent_status.vehicle_status().position().y(),now_destination.X-agent_status.vehicle_status().position().x())
			-atan2(agent_status.vehicle_status().velocity().y(),agent_status.vehicle_status().velocity().x());
		while(angle>pi)angle-=pi+pi;
		while(angle<-pi)angle+=pi+pi;
		//debug("%.9lf %10.9lf\n",omega/lenv,angle);
		if(fabs(angle)>0.3||fabs(angle)>0.1&&fabs(agent_status.vehicle_status().angular_velocity_vcs().z())<0.5){
			if(fabs(len(agent_status.vehicle_status().angular_velocity_vcs()))>0.5&&fabs(angle)<0.3)command.set_steering_angle(0);
			else{
				if(angle>0)command.set_steering_angle(4);
				else command.set_steering_angle(-4);
			}
		}else{
			auto a=angle;
			a+=agent_status.vehicle_status().angular_velocity_vcs().z()*0.2;
			command.set_steering_angle(a);
		}
		const double p=4.0;
		if(angle>omega/lenv*1.6)command.set_steering_angle(50*omega/lenv+(angle-omega/lenv*1.6)*10);
		else command.set_steering_angle(50*omega/lenv+(angle-omega/lenv*1.6)*10);
		if(fabs(angle)<0.03)command.set_steering_angle(angle);
		else command.set_steering_angle(50*omega/lenv+(angle-omega/lenv*p)*10);
		return command;


	//testing codes
		command.set_steering_angle(0);
		command.set_throttle_ratio(0);
		command.set_brake_ratio(0);
		debug("V%.9lf\n",lenv);
		debug("%d %d\n",count_ag,counter);
		if(lenv<count_ag/10.0&&counter<=1) command.set_throttle_ratio(1.0);
		else counter++;
		fprintf(f,"%d %d %.9lf %.9lf %.9lf\n",counter,count_ag,accx,velox,posx-init_pos.X);
		if(counter>=100&&counter<1000){
			if(counter==100)init_pos=mp(posx,posy),init_velo=lenv;
			command.set_brake_ratio(1);
			if(CalcVelocity(agent_status.vehicle_status().velocity())<0.01){
				counter=0;
				fprintf(f,"%d %.9lf %.9lf\n",count_ag,init_velo,agent_status.vehicle_status().position().y()-init_pos.Y);
				count_ag--;
				fflush(f);
			}
		}else if(counter==1100){
			fprintf(f,"%d %d %.9lf\n",count_ag,count_ag2,agent_status.vehicle_status().angular_velocity_vcs().z()
				/len(agent_status.vehicle_status().velocity()));
			fflush(f);
			count_ag2++;
			if(count_ag2==31){
				count_ag2=-30;
				count_ag++;
			}
			counter=0;
		}
		return command;
	//testing codes end


	}
	double CalcDistance(const interface::geometry::Vector3d& position,
						const interface::geometry::Point3D& destination) {
		double sqr_sum =
			math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
		;
		return std::sqrt(sqr_sum);
	}

	double CalcVelocity(const interface::geometry::Vector3d& velocity) {
		double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
		return std::sqrt(sqr_sum);
	}
  
	interface::map::Map MAP;
	map<pair<double,double>,vector<pair<pair<double,double>,double> > >V;
	map<pair<double,double>,vector<pair<pair<double,double>,double> > >V_full;
	map<pair<double,double>,int>dis;
	map<pair<double,double>,pair<double,double> >pre;
	map<pdd,vector<vector<pdd> > >point_to_lane;
	map<pair<int,int>,pdd>pii_to_pdd;
	double sqr(double x){return x*x;}
	double eps=1e-6;
	int check(const interface::geometry::Point3D &a,const interface::geometry::Point3D &b){//Check if two points are the same
		return abs(a.x()-b.x())+abs(a.y()-b.y())+abs(a.z()-b.z())<eps;
	}
	interface::geometry::Point2D get(interface::geometry::Point3D a){
		interface::geometry::Point2D res;
		res.set_x(a.x());
		res.set_y(a.y());
		return res;
	}
	interface::geometry::Point2D get(interface::geometry::Vector3d a){
		interface::geometry::Point2D res;
		res.set_x(a.x());
		res.set_y(a.y());
		return res;
	}
	interface::geometry::Point2D get(pair<double,double> a){
		interface::geometry::Point2D res;
		res.set_x(a.first);
		res.set_y(a.second);
		return res;
	}
	double dis2(const interface::geometry::Point2D &a,const interface::geometry::Point2D &b){//return distance^2
		return sqr(a.x()-b.x())+sqr(a.y()-b.y());
	}
	pair<double,double> tp(interface::geometry::Point2D a){
		return make_pair(a.x(),a.y());
	}
	void ae(interface::geometry::Point2D a,interface::geometry::Point2D b,lf c=-1e31){
		if(c<-1e30)c=len(to_pdd(a)-to_pdd(b))+0.1;
		//fprintf(stderr,"%.3lf %.3lf %.3lf %.3lf\n",a.x(),a.y(),b.x(),b.y());
		V[tp(a)].push_back(mp(tp(b),c));
	}
	void ae2(pdd a,pdd b){
		lf c=len(a-b)+0.1;
		//fprintf(stderr,"%.3lf %.3lf %.3lf %.3lf\n",a.x(),a.y(),b.x(),b.y());
		V_full[a].push_back(mp(b,c));
	}
	const double pi=acos(-1);
	FILE *f;


	// Whether vehicle's current position reaches the destination
	bool position_reached_destination_ = false;
	// Whether vehicle's current velocity reaches 5 m/s
	bool velocity_reached_threshold_ = false;

private:
};

}  // namespace sample
