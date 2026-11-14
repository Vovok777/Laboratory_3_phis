#pragma once
#include <vector>
#include <string>
#include <cstdint>

namespace lab {
struct Vec {
    double x=0, y=0;
    Vec operator+(Vec b) const { return {x+b.x,y+b.y}; }
    Vec operator-(Vec b) const { return {x-b.x,y-b.y}; }
    Vec operator*(double s) const { return {x*s,y*s}; }
    Vec& operator+=(Vec b) { x+=b.x;y+=b.y;return *this; }
};
inline double dot(Vec a,Vec b) {return a.x*b.x+a.y*b.y;}
enum class Mode { Discrete, CCD, MultiRate };
enum class Scene { Tunnel, Laboratory, Stack };
struct Config {
    double speed=2000, restitution=0.65, gravity=700;
    int baseHz=60, fastHz=240, solverIterations=20;
    unsigned seed=42;
};
struct Body {
    int id=0;
    Vec p, previous, v, half;
    double invMass=1;
    bool fast=false;
    bool boundary=false;
};
struct Hit { double time=0; Vec normal; bool valid=false; };
Hit sweep(const Body& a,const Body& b,double dt);
bool overlap(const Body& a,const Body& b,Vec& normal,double& depth);
bool impulse(Body& a,Body& b,Vec normal,double restitution);
struct Stats {
    int contacts=0, fastSteps=0, ccdEvents=0, eventLimit=0;
    bool wallHit=false, tunneled=false;
    double physicsMs=0;
};
class World {
public:
    Config config;
    Mode mode=Mode::CCD;
    Scene scene=Scene::Tunnel;
    std::vector<Body> fixed, normal, fast;
    Stats stats;
    double time=0;
    void reset(Scene s,const Config& c);
    void tick();
    std::uint64_t hash() const;
    double maxPenetration() const;
    double maxSpeed() const;
private:
    std::vector<Body*> bodies();
    void solve(bool onlyFast);
    void constrainArena();
    void discrete(double dt,bool onlyFast);
    void continuous(double dt);
    void advance(double dt);
};
const char* modeName(Mode m);
}
