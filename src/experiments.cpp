#include "physics.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
using namespace lab;
int main(int argc,char** argv) {
 std::filesystem::path dir=argc>1?argv[1]:"results";std::filesystem::create_directories(dir);
 std::ofstream tunnel(dir/"tunnel.csv"),perf(dir/"performance.csv"),stable(dir/"stability.csv"),det(dir/"determinism.csv");
 tunnel<<"mode,base_hz,fast_hz,speed,hit,tunneled,x_at_0_2,vx_at_0_2\n";
 perf<<"scene,mode,base_hz,fast_hz,bodies,median_ms_frame,min_ms_frame,max_ms_frame,median_ms_substep,fast_steps_per_frame,event_limits\n";
 stable<<"mode,max_penetration_px,max_slow_speed_px_s,max_vertical_jitter_px,event_limits\n";
 det<<"mode,run,hash\n";
 for(double speed:{1000.,2000.,4000.,10000.}) for(auto mode:{Mode::Discrete,Mode::CCD,Mode::MultiRate}) {
  Config c;c.speed=speed;World w;w.mode=mode;w.reset(Scene::Tunnel,c);
  for(int i=0;i<12;++i)w.tick();
  tunnel<<modeName(mode)<<",60,240,"<<speed<<','<<w.stats.wallHit<<','<<w.stats.tunneled<<','<<std::setprecision(12)<<w.fast[0].p.x<<','<<w.fast[0].v.x<<'\n';
 }
 for(auto scene:{Scene::Tunnel,Scene::Laboratory}) for(int variant=0;variant<4;++variant) {
  Mode mode=variant==1?Mode::CCD:variant==2?Mode::MultiRate:Mode::Discrete;
  Config c;if(variant==3)c.baseHz=240;
  std::vector<double> samples;int caps=0;size_t bodies=0;
  for(int trial=0;trial<7;++trial) {
   World w;w.mode=mode;w.reset(scene,c);
   bodies=w.fixed.size()+w.normal.size()+w.fast.size();
   for(int i=0;i<60;++i)w.tick();
   double total=0;
   for(int frame=0;frame<600;++frame) {
    if(scene==Scene::Tunnel&&frame%30==0)w.reset(scene,c);
    auto start=std::chrono::steady_clock::now();
    for(int step=0;step<c.baseHz/60;++step)w.tick();
    total+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
   }
   caps+=w.stats.eventLimit;samples.push_back(total/600);
  }
  std::sort(samples.begin(),samples.end());int steps=variant==2||variant==3?4:1;
  perf<<(scene==Scene::Tunnel?"tunnel":"laboratory")<<','<<modeName(mode)<<','<<c.baseHz<<",240,"<<bodies<<','<<std::setprecision(8)<<samples[3]<<','<<samples.front()<<','<<samples.back()<<','<<samples[3]/steps<<','<<steps<<','<<caps<<'\n';
 }
 for(auto mode:{Mode::Discrete,Mode::CCD,Mode::MultiRate}) {
  for(int run=0;run<3;++run) {
   Config c;World w;w.mode=mode;w.reset(Scene::Laboratory,c);for(int i=0;i<600;++i)w.tick();
   det<<modeName(mode)<<','<<run+1<<','<<w.hash()<<'\n';
  }
  World w;Config c;w.mode=mode;w.reset(Scene::Stack,c);
  w.fast.clear();double pen=0,speed=0;
  std::vector<double> low(12,1e10),high(12,-1e10);
  for(int i=0;i<1800;++i) {
   w.tick();if(i<600)continue;
   pen=std::max(pen,w.maxPenetration());
   for(size_t k=0;k<w.normal.size();++k) {
    auto& b=w.normal[k];speed=std::max(speed,std::sqrt(dot(b.v,b.v)));
    low[k]=std::min(low[k],b.p.y);high[k]=std::max(high[k],b.p.y);
   }
  }
  double jitter=0;for(size_t k=0;k<low.size();++k)jitter=std::max(jitter,high[k]-low[k]);
  stable<<modeName(mode)<<','<<std::setprecision(10)<<pen<<','<<speed<<','<<jitter<<','<<w.stats.eventLimit<<'\n';
 }
 std::cout<<"Experiments saved to "<<dir.string()<<'\n';
}
