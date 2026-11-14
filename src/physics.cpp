#include "physics.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <limits>

namespace lab {
const char* modeName(Mode m) {return m==Mode::CCD?"CCD":m==Mode::MultiRate?"Multi-rate":"Discrete";}
bool overlap(const Body& a,const Body& b,Vec& n,double& d) {
    Vec delta=b.p-a.p;
    double ox=a.half.x+b.half.x-std::abs(delta.x), oy=a.half.y+b.half.y-std::abs(delta.y);
    if(ox<=0 || oy<=0) return false;
    if(ox<oy) {d=ox;n={delta.x>=0?1.0:-1.0,0};}
    else {d=oy;n={0,delta.y>=0?1.0:-1.0};}
    return true;
}
Hit sweep(const Body& a,const Body& b,double dt) {
    Vec q=a.p-b.p, v=a.v-b.v, h=a.half+b.half;
    double entry=-std::numeric_limits<double>::infinity();
    double leave=std::numeric_limits<double>::infinity();
    Vec n;
    for(int axis=0;axis<2;++axis) {
        double p=axis?q.y:q.x, speed=axis?v.y:v.x, extent=axis?h.y:h.x;
        if(std::abs(speed)<1e-12) {
            if(std::abs(p)>=extent-1e-9) return {};
            continue;
        }
        double t1=(-extent-p)/speed,t2=(extent-p)/speed;
        if(t1>t2) std::swap(t1,t2);
        if(t1>entry) {entry=t1; n=axis?Vec{0,speed>0?1.0:-1.0}:Vec{speed>0?1.0:-1.0,0};}
        leave=std::min(leave,t2);
    }
    if(entry>leave || leave<0 || entry < -1e-9 || entry>dt+1e-12) return {};
    if(dot(b.v-a.v,n)>=-1e-9) return {};
    return {std::max(0.0,entry),n,true};
}
bool impulse(Body& a,Body& b,Vec n,double e) {
    double mass=a.invMass+b.invMass, vn=dot(b.v-a.v,n);
    if(mass<=0 || vn>=-1e-9) return false;
    if(-vn<30) e=0;
    double j=-(1+e)*vn/mass;
    a.v+=n*(-j*a.invMass); b.v+=n*(j*b.invMass);
    return true;
}
std::vector<Body*> World::bodies() {
    std::vector<Body*> all; all.reserve(fixed.size()+normal.size()+fast.size());
    for(auto& b:fixed) all.push_back(&b);
    for(auto& b:normal) all.push_back(&b);
    for(auto& b:fast) all.push_back(&b);
    return all;
}
void World::reset(Scene s,const Config& c) {
    scene=s;config=c;fixed.clear();normal.clear();fast.clear();stats={};time=0;
    int id=0;
    auto add=[&](std::vector<Body>& out,Vec p,Vec half,Vec v,double mass,bool f=false){
        out.push_back({id++,p,p,v,half,mass,f});
    };
    if(s==Scene::Tunnel) {
        add(fixed,{508,210},{5,150},{},0);
        add(fast,{120,210},{3,3},{c.speed,0},1,true);
        return;
    }
    for(Vec p:std::vector<Vec>{{160,390},{520,390},{750,390},{80,200},{810,160}})
        add(fixed,p,{32,32},{},0);
    add(fixed,{450,450},{470,10},{},0);
    add(fixed,{-10,220},{10,230},{},0);
    add(fixed,{910,220},{10,230},{},0);
    add(fixed,{450,-10},{470,10},{},0);
    for(size_t i=5;i<fixed.size();++i) fixed[i].boundary=true;
    if(s==Scene::Stack) {
        for(int col=0;col<3;++col) for(int row=0;row<4;++row)
            add(normal,{300.+col*50,424.-row*32},{16,16},{},1);
        add(fast,{620,250},{3,3},{c.speed,0},0.5,true);
    } else {
        auto state=c.seed;
        auto random=[&](){state=1664525u*state+1013904223u;return double(state&65535u)/65535.;};
        for(int row=0;row<4;++row) for(int col=0;col<6;++col)
            add(normal,{220.+col*75,60.+row*65},{16,16},{(random()-.5)*100,0},1);
        add(fast,{110,310},{3,3},{c.speed,0},0.5,true);
        add(fast,{780,280},{3,3},{-c.speed,0},0.5,true);
    }
}
void World::solve(bool onlyFast) {
    auto all=bodies();
    for(int iter=0;iter<config.solverIterations;++iter) {
        for(size_t i=0;i<all.size();++i) for(size_t j=i+1;j<all.size();++j) {
            auto& a=*all[i];auto& b=*all[j];
            if(a.boundary||b.boundary) continue;
            if(a.invMass+b.invMass==0 || (onlyFast && !a.fast && !b.fast)) continue;
            Vec n;double d;
            if(!overlap(a,b,n,d)) continue;
            if(impulse(a,b,n,config.restitution)) ++stats.contacts;
            if(scene==Scene::Tunnel && (a.fast||b.fast)) stats.wallHit=true;
            double correction=std::max(d-0.001,0.0)*0.8/(a.invMass+b.invMass);
            a.p+=n*(-correction*a.invMass);b.p+=n*(correction*b.invMass);
        }
        constrainArena();
    }
}
void World::constrainArena() {
    if(scene==Scene::Tunnel) return;
    Body wall;wall.invMass=0;
    for(auto group:{&normal,&fast}) for(auto& b:*group) {
        auto contact=[&](double& coordinate,double limit,bool upper,Vec outward) {
            if(upper?coordinate>=limit:coordinate<=limit) {
                coordinate=limit;
                if(impulse(b,wall,outward,config.restitution)) ++stats.contacts;
            }
        };
        contact(b.p.x,b.half.x,false,{-1,0});
        contact(b.p.x,900-b.half.x,true,{1,0});
        contact(b.p.y,b.half.y,false,{0,-1});
        contact(b.p.y,440-b.half.y,true,{0,1});
    }
}
void World::discrete(double dt,bool onlyFast) {
    if(!onlyFast) for(auto& b:normal) b.p+=b.v*dt;
    for(auto& b:fast) b.p+=b.v*dt;
    solve(onlyFast);
}
void World::advance(double dt) {
    for(auto& b:normal) b.p+=b.v*dt;
    for(auto& b:fast) b.p+=b.v*dt;
}
void World::continuous(double dt) {
    solve(false);
    auto all=bodies(); double remaining=dt; int events=0;
    constexpr int maxEvents=128;
    while(remaining>1e-10 && events<maxEvents) {
        Hit best{remaining,{},false};Body *first=nullptr,*second=nullptr;
        for(size_t i=0;i<all.size();++i) for(size_t j=i+1;j<all.size();++j) {
            if(all[i]->invMass+all[j]->invMass==0) continue;
            auto h=sweep(*all[i],*all[j],remaining);
            if(h.valid && (!best.valid||h.time<best.time)) {best=h;first=all[i];second=all[j];}
        }
        if(!best.valid) {advance(remaining);remaining=0;break;}
        advance(best.time);remaining-=best.time;
        impulse(*first,*second,best.normal,config.restitution);
        ++stats.contacts;++stats.ccdEvents;++events;
        if(scene==Scene::Tunnel) stats.wallHit=true;
        double m=first->invMass+second->invMass;
        first->p+=best.normal*(-1e-7*first->invMass/m);
        second->p+=best.normal*(1e-7*second->invMass/m);
    }
    if(remaining>1e-10) {
        ++stats.eventLimit;
    }
    solve(false);
}
void World::tick() {
    auto start=std::chrono::steady_clock::now();
    stats.contacts=stats.fastSteps=stats.ccdEvents=0;
    double dt=1.0/config.baseHz;
    double g=scene==Scene::Tunnel?0:config.gravity;
    for(auto& b:normal) {b.previous=b.p;b.v.y+=g*dt;}
    for(auto& b:fast) b.previous=b.p;
    if(mode==Mode::MultiRate) {
        for(auto& b:normal) b.p+=b.v*dt;
        solve(false);
        int count=std::max(1,config.fastHz/config.baseHz);
        for(int i=0;i<count;++i) {
            for(auto& b:fast) b.v.y+=g*dt/count;
            discrete(dt/count,true);++stats.fastSteps;
        }
    } else {
        for(auto& b:fast) b.v.y+=g*dt;
        if(mode==Mode::CCD) continuous(dt); else discrete(dt,false);
        stats.fastSteps=1;
    }
    time+=dt;
    if(scene==Scene::Tunnel && !stats.wallHit && fast[0].p.x-fast[0].half.x>513) stats.tunneled=true;
    stats.physicsMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
}
std::uint64_t World::hash() const {
    std::uint64_t h=1469598103934665603ull;
    for(auto group:{&normal,&fast}) for(auto& b:*group) for(double x:{b.p.x,b.p.y,b.v.x,b.v.y}) {
        std::uint64_t bits;std::memcpy(&bits,&x,8);h^=bits;h*=1099511628211ull;
    }
    return h;
}
double World::maxPenetration() const {
    std::vector<const Body*> all;
    for(auto group:{&fixed,&normal,&fast}) for(auto& b:*group) all.push_back(&b);
    double max=0,d;Vec n;
    for(size_t i=0;i<all.size();++i) for(size_t j=i+1;j<all.size();++j)
        if(all[i]->invMass+all[j]->invMass>0 && overlap(*all[i],*all[j],n,d)) max=std::max(max,d);
    return max;
}
double World::maxSpeed() const {
    double max=0;for(auto group:{&normal,&fast}) for(auto& b:*group) max=std::max(max,std::sqrt(dot(b.v,b.v)));return max;
}
}
