// Copyright @2018 Pony AI Inc. All rights reserved.


#include "perception/perception.h"
#include<algorithm>
#include<iostream>
#include <cmath>
#include <vector>
#include <map>

struct pt {
	double x, y, z;
	double dis() {
		return std::sqrt(x*x+y*y+z*z);
	}
	double dis2() {
		return std::sqrt(x*x+y*y);
	}
	double alpha(){
		return atan2(y,x);
	}
	pt rotate (double al) {
		return pt(x*cos(al)-y*sin(al), x*sin(al)+y*cos(al),z);
	}
	pt (){}
	pt (double X, double Y):x(X),y(Y){}
	pt (double X, double Y, double Z):x(X),y(Y),z(Z){}
	pt to2(){ return pt(x,y);}
};

pt operator+(const pt&a,const pt&b){return pt(a.x+b.x,a.y+b.y,a.z+b.z);}
pt operator-(const pt&a,const pt&b){return pt(a.x-b.x,a.y-b.y,a.z-b.z);}
pt operator-(const pt&b){return (pt){-b.x,-b.y,-b.z};}
double dot(const pt&a,const pt&b){return a.x*b.x+a.y*b.y+a.z*b.z;}
pt cro(const pt&a,const pt&b){return pt(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);}
bool operator<(const pt&a, const pt&b){
	return a.x<b.x;
}

const double radius = 2.0;
const double height_diff = 0.5;
const double max_ground_height1 = 0.0;
const double max_ground_height2 = -1.0;
pt origin;
const double link_dist = 2;
int cmpx(pt a,pt b){
	return a.x<b.x;
}

int n;
std::vector<pt> p;

std::vector<int>fa;

int gf(int x){
	return x==fa[x]?x:fa[x]=gf(fa[x]);
}

std::vector< std::vector<int> > children;

void addedge(int x,int y){
	fa[gf(x)]=gf(y);
}
std::vector<double> ground_zs;
double zground;
void removeground() {

	std::sort(p.begin(),p.end(),cmpx);

	std::vector<bool> mark;
	mark.resize(n);

	for (int i=0;i<n;i++) {
		/*if (pointcloud.points[i](2) > max_ground_height1 || pointcloud.points[i](2)>max_ground_height2 && pow(pointcloud.points[i](0),2) + pow(pointcloud.points[i](1),2) < 1.5*1.5) {
		  mark[i]=true;
		  continue;
		  }*/

		int tot = 0;
		int found  = 0;

		int start=i;
		while(start>0 && p[i].x-p[start-1].x <=radius)
			start--;

		for (int j=start;j<n;j++){
			auto temp = p[j] - p[i];

			if (temp.x>radius) break;
			if (temp.dis()>radius) continue;

			++tot;
			if(temp.z < -height_diff) {
				++found;
			}
			if (found > 10){
				mark[i]=true;
				break;
			}
		}
	}

	std::vector<pt> ans;
	for (int i=0;i<mark.size();i++) {
		if (mark[i]) {
			ans.push_back(p[i]);
		} else {
			ground_zs.push_back(p[i].z);
		}
	}
	n=ans.size();
	p=ans;

	std::sort(ground_zs.begin(),ground_zs.end());
	if (!ground_zs.empty()) zground = ground_zs[ground_zs.size()/2];
	else zground = 0.0;
}

interface::perception::PerceptionObstacles Perception::RunPerception(
		const PointCloud& pointcloud, const utils::Optional<cv::Mat>& image) {
	interface::perception::PerceptionObstacles perception_result;

	p.clear();
	fa.clear();
	children.clear();
	ground_zs.clear();
	zground=0;

	n = pointcloud.points.size();
	for (int i = 0;i<n;i++) {
		Eigen::Vector3d pp = pointcloud.rotation * pointcloud.points[i] + pointcloud.translation;

		p.push_back(pt(pp(0),pp(1),pp(2)));
	}
	origin = pt(pointcloud.translation(0),pointcloud.translation(1),pointcloud.translation(2)) ;

	removeground();

	int u = 0;
	int far  = -1;
	for (int j=0;j<p.size();j++){
		if(far==-1 || ( ((p[u]-p[j]).dis()>(p[u]-p[far]).dis2()))) {
			far = j;
		}
	}
	u = far;
	far=-1;
	for (int j=0;j<p.size();j++){
		if(far==-1 || ( ((p[u]-p[j]).dis()>(p[u]-p[far]).dis2()))) {
			far = j;
		}
	}
	pt dir = p[far]-p[u];
	double diralpha = dir.alpha();
	//double diralpha = 0;

	//std::cout<<"tot"<<n<<std::endl;
	fa.resize(n);
	for (int i=0;i<n;i++)fa[i]=i;
	std::sort(p.begin(),p.end(),cmpx);
	for (int i=0;i<n;i++) {

		int start=i;
		while(start>0 && p[i].x-p[start-1].x <=link_dist)
			start--;

		int cnt=0;
		for (int j=start;j<n;j++){
			auto temp = p[j] - p[i];

			if (temp.x>link_dist) break;
			double d = temp.dis();
			if (d>link_dist) continue;

			if (d<1.0)cnt++;
			//addedge(i,j);
		}
		double mydist;
		if (cnt>30) mydist = 1.2;
		else mydist = 1.9;

		std::vector<std::pair<double, int> > pa;
		for (int j=start;j<n;j++){
			auto temp = p[j] - p[i];

			if (temp.x>mydist) break;
			double d = temp.dis();
			if (d>mydist) continue;

			pa.push_back(std::make_pair(d, j));
		}
		sort(pa.begin(),pa.end());
		//int nn = pa.size() * 0.4;
		int nn = pa.size() ;
		for (int j=0;j<nn;j++)addedge(i, pa[j].second);

	}

	children.resize(n);
	for (int i=0;i<n;i++) {
		children[gf(i)].push_back(i);
	}

	for (int i=0;i<n;i++) if(children[i].size()>0) {
		std::vector<double> zs;

		double oldmaxz=-1e9;
		for (auto j:children[i]) {
			oldmaxz = std::max(oldmaxz,p[j].z);
			zs.push_back(p[j].z);
		}
		int K = (zs.size()-1)*0.7;
		std::nth_element(zs.begin(),zs.begin()+K,zs.end());
		double ZZ = zs[K];
		double minx=1e9, maxx=-1e9, miny=1e9,maxy=-1e9,minz=1e9,maxz=-1e9;

		int u = children[i][0];
		int far  = -1;
		for (auto j:children[i]) {
			if(far==-1 || ( ((p[u]-p[j]).dis()>(p[u]-p[far]).dis2()))) {
				far = j;
			}
		}
		u = far;
		far=-1;
		for (auto j:children[i]) {
			if(far==-1 || ( ((p[u]-p[j]).dis()>(p[u]-p[far]).dis2()))) {
				far = j;
			}
		}

		for (auto j:children[i]) {
			if (p[j].z>ZZ)continue;
			minx = std::min (minx, p[j].rotate(-diralpha).x);
			maxx = std::max (maxx, p[j].rotate(-diralpha).x);
			miny = std::min (miny, p[j].rotate(-diralpha).y);
			maxy = std::max (maxy, p[j].rotate(-diralpha).y);
			minz = std::min (minz, p[j].rotate(-diralpha).z);
			maxz = std::max (maxz, p[j].rotate(-diralpha).z);
		}

		double xx = maxx-minx;
		double yy = maxy-miny;
		double zz = maxz-minz;

		if (zz >=2.5 && std::max(yy,xx)<3.3) continue;
		if (maxz-zground > 2.5 && std::max(xx,yy)<3.5) continue;
		if (minz-zground > 1.6) continue;
		if (maxz-zground > std::max(xx,yy)+0.9) continue;
		if (maxz-zground>5) continue;
		if (-zground<0.5) continue;

		if (children[i].size()<10) continue;
		if (oldmaxz-minz<0.4) continue;
		if (std::max(zz,yy)<0.5) continue;
		if (std::max(zz,xx)<0.5) continue;
		if (std::max(xx,yy)>15)continue;
		if (std::max(xx,std::max(yy,oldmaxz-zground)) < 0.8) continue;

		if (zz>3 && std::max(xx,yy) / zz <1.5)continue;
		double minx1=1e9,maxx1=-1e9,miny1=1e9,maxy1=-1e9;
		auto zhou = p[far]-p[u];
		pt yzhou(-zhou.y, zhou.x);
		for (auto j:children[i]) {

			double X = dot(p[j], zhou)/zhou.dis(), Y = dot(p[j],yzhou)/yzhou.dis();;
			minx1 = std::min (minx1, X);
			maxx1 = std::max (maxx1, X);
			miny1 = std::min (miny1, Y);
			maxy1 = std::max (maxy1, Y);
		}
		if (std::max(maxx1-minx1, maxy1-miny1)< 0.6) continue;




		/*if (yy<1.7 || yy > 10) continue;
		if (xx<1.7 || xx > 10) continue;*/

		{
			//std::cout<<"found"<<minx<<" "<<miny<<" "<<maxx<<" "<<maxy<<std::endl;
			auto* obstacle = perception_result.add_obstacle();
			obstacle->set_type(interface::perception::ObjectType::CAR);
			{
				auto* polygon_point = obstacle->add_polygon_point();
				pt P(minx,miny,minz);
				P = P.rotate(diralpha);
				polygon_point->set_x(P.x);
				polygon_point->set_y(P.y);
				polygon_point->set_z(P.z);
			}
			{
				auto* polygon_point = obstacle->add_polygon_point();
				pt P(maxx,miny,minz);
				P = P.rotate(diralpha);
				polygon_point->set_x(P.x);
				polygon_point->set_y(P.y);
				polygon_point->set_z(P.z);
			}
			{
				auto* polygon_point = obstacle->add_polygon_point();
				pt P(maxx,maxy,minz);
				P = P.rotate(diralpha);
				polygon_point->set_x(P.x);
				polygon_point->set_y(P.y);
				polygon_point->set_z(P.z);
			}
			{
				auto* polygon_point = obstacle->add_polygon_point();
				pt P(minx,maxy,minz);
				P = P.rotate(diralpha);
				polygon_point->set_x(P.x);
				polygon_point->set_y(P.y);
				polygon_point->set_z(P.z);
			}
			obstacle->set_height(zz);
		}
	}

	std::cout<<"done"<<std::endl;
	LOG(INFO) << "Perception done.";
	return perception_result;
}

