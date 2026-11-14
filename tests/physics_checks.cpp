#include "physics.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace lab;
int main() {
 int passed=0;
 auto check=[&](bool yes,const char* name){if(!yes) throw std::runtime_error(name);++passed;std::cout<<"PASS "<<name<<'\n';};
 try {
  Body a{0,{0,0},{},{100,0},{1,1},1},b{1,{10,0},{},{0,0},{1,1},0};
  auto h=sweep(a,b,1);
  check(h.valid&&std::abs(h.time-.08)<1e-12&&h.normal.x==1,"analytic swept AABB TOI");
  a.v={-100,0};check(!sweep(a,b,1).valid,"separating objects do not collide");
  a.v={0,100};check(!sweep(a,b,1).valid,"parallel separated slabs");
  a.p={8,0};a.v={100,0};check(sweep(a,b,1).time==0&&sweep(a,b,1).valid,"touching approaching contact");
  a.v={-100,0};check(!sweep(a,b,1).valid,"touching departing contact");
  a.p={0,0};a.v={100,0};b.invMass=1;
  impulse(a,b,{1,0},1);
  check(std::abs(a.v.x)<1e-12&&std::abs(b.v.x-100)<1e-12,"equal mass elastic velocity exchange");
  a.v={100,0};b.v={0,0};b.invMass=0;impulse(a,b,{1,0},.65);
  check(std::abs(a.v.x+65)<1e-12,"restitution against fixed wall");
  Config c;World w;
  w.mode=Mode::Discrete;w.reset(Scene::Tunnel,c);for(int i=0;i<12;++i)w.tick();
  check(w.stats.tunneled&&!w.stats.wallHit,"60 Hz demonstration tunnels");
  w.mode=Mode::CCD;w.reset(Scene::Tunnel,c);for(int i=0;i<12;++i)w.tick();
  check(w.stats.wallHit&&!w.stats.tunneled&&std::abs(w.fast[0].p.x-487)<1e-5,"CCD remainder agrees with analytic x=487");
  w.mode=Mode::MultiRate;w.reset(Scene::Tunnel,c);for(int i=0;i<12;++i)w.tick();
  check(w.stats.wallHit&&!w.stats.tunneled&&w.stats.fastSteps==4,"multi-rate 240 Hz hits wall with four substeps");
  c.speed=10000;w.reset(Scene::Tunnel,c);for(int i=0;i<10;++i)w.tick();
  check(w.stats.tunneled,"multi-rate is not an unconditional CCD replacement");
  c=Config{};c.restitution=1;w.mode=Mode::CCD;w.reset(Scene::Tunnel,c);
  w.fixed={{0,{0,0},{},{},{1,10},0},{1,{10,0},{},{},{1,10},0}};
  w.fast={{2,{5,0},{},{2000,0},{1,1},1,true}};w.tick();
  check(w.fast[0].p.x>=2-1e-5&&w.fast[0].p.x<=8+1e-5&&w.stats.ccdEvents>=5,"multiple impacts in one step remain inside corridor");
  for(auto m:{Mode::Discrete,Mode::CCD,Mode::MultiRate}) {
   World u,v;u.mode=v.mode=m;u.reset(Scene::Laboratory,c);v.reset(Scene::Laboratory,c);
   for(int i=0;i<120;++i){u.tick();v.tick();}
   check(u.hash()==v.hash(),"identical input reproduces state bit for bit");
  }
  c=Config{};w.mode=Mode::Discrete;w.reset(Scene::Stack,c);
  bool contained=true;
  for(int i=0;i<600;++i) {
   w.tick();
   for(auto group:{&w.normal,&w.fast})for(const auto& body:*group)
    contained=contained&&body.p.x>=body.half.x-1e-6&&body.p.x<=900-body.half.x+1e-6
      &&body.p.y>=body.half.y-1e-6&&body.p.y<=440-body.half.y+1e-6;
  }
  check(contained,"stack scene keeps every body inside the arena for ten seconds");
  for(auto mode:{Mode::Discrete,Mode::CCD,Mode::MultiRate}) {
   c=Config{};c.gravity=0;c.speed=12000;c.restitution=.65;
   for(Vec velocity:{Vec{12000,0},Vec{-12000,0},Vec{0,12000},Vec{0,-12000},Vec{12000,12000}}) {
    w.mode=mode;w.reset(Scene::Stack,c);w.normal.clear();w.fixed.clear();
    w.fast[0].p={velocity.x<0?4.:896.,velocity.y<0?4.:436.};
    w.fast[0].v=velocity;w.tick();auto& projectile=w.fast[0];
    check(projectile.p.x>=3&&projectile.p.x<=897&&projectile.p.y>=3&&projectile.p.y<=437,
          "high-speed overshoot is contained at sides and corners");
    check((velocity.x==0||projectile.v.x*velocity.x<0)&&(velocity.y==0||projectile.v.y*velocity.y<0),
          "arena contact reflects outward velocity inward");
   }
  }
  c=Config{};c.gravity=0;w.mode=Mode::Discrete;w.reset(Scene::Stack,c);w.normal.clear();w.fixed.clear();
  w.fast[0].p={897,100};w.fast[0].v={-100,0};w.tick();
  check(w.fast[0].v.x==-100,"inward velocity is not reflected again at arena boundary");
  std::cout<<passed<<" checks passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
