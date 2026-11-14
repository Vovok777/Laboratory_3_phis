#include "physics.hpp"
#include <SFML/Graphics.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <array>
#include <functional>
#include <algorithm>
#include <cmath>
#ifdef _WIN32
#include <windows.h>
#endif
using namespace lab;
namespace fs=std::filesystem;
namespace {
const sf::Color bg(13,19,31),panel(22,31,47),border(42,55,75),ink(235,241,249),muted(151,169,193);
const std::array<sf::Color,3> accents={sf::Color(246,156,93),sf::Color(102,218,192),sf::Color(136,171,255)};
sf::String utf(const std::string& s){return sf::String::fromUtf8(s.begin(),s.end());}
std::string num(double v,int p=2){std::ostringstream s;s<<std::fixed<<std::setprecision(p)<<v;return s.str();}
struct Button {sf::FloatRect box;std::string label,hint;std::function<void()> action;bool active=false;};
class App {
 sf::Font font;
 std::array<World,3> worlds;
 std::array<std::vector<Vec>,3> trails;
 std::array<double,3> timings{};
 std::array<double,3> frameMs{};
 std::array<int,3> steps{};
 std::vector<Button> buttons;
 Config config;
 Scene scene=Scene::Tunnel;
 int selected=1;
 bool running=false,compare=true,help=false;
 double scale=.25,accumulator=0;
 sf::Vector2f mouse{-1,-1};
 std::string notice="Выберите опыт и нажмите «Старт». Все режимы получают одинаковые условия.";
 fs::path root;
 void rect(sf::RenderTarget& t,float x,float y,float w,float h,sf::Color color,sf::Color outline=sf::Color::Transparent) {
  sf::RectangleShape r({w,h});r.setPosition({x,y});r.setFillColor(color);r.setOutlineColor(outline);r.setOutlineThickness(outline.a?1.f:0.f);t.draw(r);
 }
 void text(sf::RenderTarget& t,std::string s,float x,float y,unsigned size=18,sf::Color color=ink) {
  sf::Text tx(font,utf(s),size);tx.setFillColor(color);tx.setPosition({x,y});t.draw(tx);
 }
 void line(sf::RenderTarget& t,sf::Vector2f a,sf::Vector2f b,sf::Color c) {
  sf::Vertex v[2];v[0].position=a;v[1].position=b;v[0].color=v[1].color=c;t.draw(v,2,sf::PrimitiveType::Lines);
 }
 void button(float x,float y,float w,float h,std::string s,std::function<void()> f,bool active=false,std::string hint="") {
  buttons.push_back({{{x,y},{w,h}},s,hint,std::move(f),active});
 }
 void reset() {
  for(int i=0;i<3;++i){worlds[i].mode=static_cast<Mode>(i);worlds[i].reset(scene,config);trails[i].clear();timings[i]=0;steps[i]=0;}
  accumulator=0;frameMs={};
 }
 void changeScene(Scene s){scene=s;compare=s==Scene::Tunnel;reset();running=false;notice="Опыт готов. Старт — движение, Шаг — один шаг физики.";}
 void tick(){
  for(int i=0;i<3;++i){worlds[i].tick();timings[i]=.92*timings[i]+.08*worlds[i].stats.physicsMs;frameMs[i]+=worlds[i].stats.physicsMs;steps[i]+=worlds[i].stats.fastSteps;
   if(!worlds[i].fast.empty()){trails[i].push_back(worlds[i].fast[0].p);if(trails[i].size()>45)trails[i].erase(trails[i].begin());}}
 }
 void toggle(){if(scene==Scene::Tunnel&&worlds[0].time>=.3)reset();running=!running;}
 void exportState(){
  fs::create_directories(root/"results");std::ofstream f(root/"results"/"snapshot.csv");
  f<<"mode,time,speed,restitution,base_hz,fast_hz,physics_ms,fast_steps_last_render,hit,tunneled,hash\n";
  for(int i=0;i<3;++i){auto&w=worlds[i];f<<modeName(w.mode)<<','<<w.time<<','<<config.speed<<','<<config.restitution<<','<<config.baseHz<<','<<config.fastHz<<','<<timings[i]<<','<<steps[i]<<','<<w.stats.wallHit<<','<<w.stats.tunneled<<','<<w.hash()<<'\n';}
  notice="Сохранено: results/snapshot.csv рядом с приложением.";
 }
 void controls(){
  buttons.clear();
  button(28,152,264,42,"01  Тонкая стенка",[&]{changeScene(Scene::Tunnel);},scene==Scene::Tunnel);
  button(28,202,264,42,"02  Система объектов",[&]{changeScene(Scene::Laboratory);},scene==Scene::Laboratory);
  button(28,252,264,42,"03  Устойчивость стопки",[&]{changeScene(Scene::Stack);},scene==Scene::Stack);
  button(28,348,42,38,"−",[&]{config.speed=std::max(500.,config.speed-500);reset();},false,"Скорость снаряда, px/с. Изменение перезапускает опыт.");
  button(250,348,42,38,"+",[&]{config.speed=std::min(12000.,config.speed+500);reset();});
  button(28,434,42,38,"−",[&]{config.restitution=std::max(0.,config.restitution-.05);reset();});
  button(250,434,42,38,"+",[&]{config.restitution=std::min(1.,config.restitution+.05);reset();},false,"e = 1 — упругий отскок; e = 0 — без отскока.");
  for(int j=0;j<2;++j)button(28.f+j*136,520,128,38,j?"120 Гц":"60 Гц",[&,j]{config.baseHz=j?120:60;reset();},config.baseHz==(j?120:60));
  for(int j=0;j<3;++j)button(28.f+j*90,607,84,38,j==0?"0.1×":j==1?"0.25×":"1×",[&,j]{scale=j==0?.1:j==1?.25:1.;},scale==(j==0?.1:j==1?.25:1.));
  button(28,678,264,48,running?"Пауза   [Пробел]":"Старт   [Пробел]",[&]{toggle();},true);
  button(28,736,128,42,"Шаг   [N]",[&]{running=false;tick();},false,"Один базовый шаг; в multi-rate внутри него 4 или 2 подшага.");
  button(164,736,128,42,"Сброс   [R]",[&]{reset();running=false;});
  button(28,797,128,36,"Справка",[&]{help=!help;});
  button(164,797,128,36,"CSV",[&]{exportState();});
  for(int i=0;i<3;++i)button(344.f+i*350,102,338,58,i==0?"1   Дискретный":i==1?"2   CCD":"3   Multi-rate",[&,i]{selected=i;compare=false;},selected==i&&!compare);
  button(1185,32,215,40,compare?"Один режим":"Сравнить все",[&]{compare=!compare;},compare);
 }
 void world(sf::RenderTarget& t,int index,float x,float y,float width,float height,bool lane){
  auto&w=worlds[index];sf::Color ac=accents[index];
  rect(t,x,y,width,height,panel,border);
  text(t,index==0?"ДИСКРЕТНЫЙ":index==1?"CCD":"MULTI-RATE",x+20,y+12,16,ac);
  std::string state=scene==Scene::Tunnel?(w.stats.tunneled?"Пролёт сквозь стенку":w.stats.wallHit?"Контакт и отскок":"До столкновения"):"t = "+num(w.time)+" с";
  text(t,state,x+220,y+12,16,scene==Scene::Tunnel&&w.stats.tunneled?accents[0]:muted);
  float factor=lane?(width-60)/900:std::min((width-50)/900,(height-115)/450);
  float ox=x+(width-900*factor)/2,oy=lane?y+height*.57f-210*factor:y+60;
  auto pos=[&](Vec p){return sf::Vector2f{ox+float(p.x)*factor,oy+float(p.y)*factor};};
  if(lane){
   auto py=pos({0,210}).y;line(t,{x+26,py},{x+width-26,py},border);
   for(int v=0;v<=900;v+=100){auto p=pos({double(v),210});line(t,{p.x,py+20},{p.x,py+25},muted);text(t,std::to_string(v),p.x-10,py+28,12,muted);}
  } else {
   for(int v=0;v<=900;v+=50)line(t,pos({double(v),0}),pos({double(v),440}),sf::Color(30,42,60));
   for(int v=0;v<=450;v+=50)line(t,pos({0,double(v)}),pos({900,double(v)}),sf::Color(30,42,60));
  }
  for(size_t k=1;k<trails[index].size();++k){
   auto a=trails[index][k-1],b=trails[index][k];
   if(a.x<0||b.x<0||a.x>900||b.x>900||a.y<0||b.y>440)continue;
   auto color=ac;color.a=static_cast<std::uint8_t>(25+100*k/trails[index].size());line(t,pos(a),pos(b),color);
  }
  auto body=[&](const Body& b,sf::Color c){
   if(b.p.x+b.half.x<0||b.p.x-b.half.x>900||b.p.y+b.half.y<0||b.p.y-b.half.y>450)return;
   auto p=pos(b.p-b.half);float wv=float(b.half.x*2)*factor,hv=float(b.half.y*2)*factor;
   if(lane&&!b.fast){p.y=y+51;hv=height-104;}
   rect(t,p.x,p.y,std::max(3.f,wv),std::max(3.f,hv),c);
   if(b.fast){auto center=pos(b.p);sf::CircleShape halo(8);halo.setOrigin({8,8});halo.setPosition(center);halo.setFillColor(sf::Color(c.r,c.g,c.b,35));t.draw(halo);}
  };
  for(auto&b:w.fixed)body(b,sf::Color(91,109,135));
  for(auto&b:w.normal)body(b,sf::Color(97,139,188));
  for(auto&b:w.fast)body(b,ac);
  if(lane){auto p=pos({508,210});text(t,"10 px",p.x+17,y+57,13,muted);text(t,"t = "+num(w.time)+" с",x+width-115,y+12,14,muted);}
  else {text(t,"x вправо, y вниз · Мир 900 × 450 px",x+20,y+height-27,13,muted);}
 }
 void draw(sf::RenderTarget& t){
  t.clear(bg);controls();
  text(t,"МЕХАНИКА",28,26,15,accents[1]);text(t,"Лаборатория столкновений",28,49,28);
  text(t,"CCD и разная частота физики",550,49,17,muted);
  rect(t,12,104,308,742,panel);
  text(t,"СЦЕНАРИЙ",28,119,13,muted);
  text(t,"Скорость снаряда",28,317,16,muted);text(t,num(config.speed,0)+" px/с",94,351,19);
  text(t,"Коэффициент восстановления",28,403,15,muted);text(t,"e = "+num(config.restitution),108,437,19);
  text(t,"Обычные объекты",28,489,16,muted);
  text(t,"Скорость воспроизведения",28,576,15,muted);
  for(auto&b:buttons){bool hover=b.box.contains(mouse);rect(t,b.box.position.x,b.box.position.y,b.box.size.x,b.box.size.y,b.active?sf::Color(37,83,86):hover?sf::Color(50,65,88):sf::Color(31,43,63),b.active?sf::Color(77,157,150):border);text(t,b.label,b.box.position.x+12,b.box.position.y+(b.box.size.y-23)/2,17,b.active?sf::Color(165,244,224):ink);}
  if(compare&&scene==Scene::Tunnel)for(int i=0;i<3;++i)world(t,i,344,180.f+i*172,1038,160,true);
  else if(compare){
   for(int i=0;i<3;++i)world(t,i,344.f+i*350,180,338,340,false);
   text(t,"Одинаковые начальные условия · все три мира считаются независимо",365,552,18,muted);
   text(t,"Нажмите на название режима, чтобы рассмотреть сцену крупнее.",365,588,18,muted);
  }else world(t,selected,344,180,1038,504,scene==Scene::Tunnel);
  for(int i=0;i<3;++i){float x=344.f+i*350;
   rect(t,x,705,338,90,panel);text(t,num(timings[i],4)+" мс / базовый шаг",x+16,712,18,accents[i]);
   text(t,"Шагов снаряда / кадр: "+std::to_string(steps[i]),x+16,742,14,muted);
   text(t,num(frameMs[i],4)+" мс физики / кадр",x+16,766,13,muted);
  }
  std::string explain=scene==Scene::Tunnel?"60 Гц: 33.3 px за шаг. 240 Гц: 8.3 px. CCD находит момент контакта внутри шага.":scene==Scene::Laboratory?"24 обычных тела · 2 снаряда · 5 препятствий и 4 границы. Снаряды выделены цветом.":"12 блоков в трёх стопках · 1 снаряд · 5 препятствий и 4 границы.";
  if(config.speed!=2000||config.baseHz!=60)explain="Путь за шаг: "+num(config.speed/config.baseHz,1)+" px у обычного режима; "+num(config.speed/config.fastHz,1)+" px у multi-rate.";
  text(t,explain,344,813,15,muted);
  std::string hint=notice;for(auto&b:buttons)if(b.box.contains(mouse)&&!b.hint.empty())hint=b.hint;
  text(t,hint,28,864,15,muted);
  if(help){
   rect(t,0,0,1440,900,sf::Color(0,0,0,190));rect(t,340,190,760,510,panel,border);
   text(t,"Как пользоваться лабораторией",374,220,26);
   std::vector<std::string> rows={"1. Выберите сцену слева. Начните с тонкой стенки.","2. Нажмите Старт. В конце короткого опыта будет пауза.","3. Сравните пролёт, момент контакта и отскок.","4. Меняйте скорость, упругость и частоту; опыт начнётся заново.","Пробел — старт / пауза     N — один шаг     R — сброс","1 / 2 / 3 — режим     C — сравнение     F12 — скриншот","CSV — сохранить параметры и текущее состояние трёх миров.","Замедление меняет воспроизведение, а не шаг физики.","F1 или Esc — закрыть справку"};
   for(size_t i=0;i<rows.size();++i)text(t,rows[i],374,280+float(i)*42,17,i==8?accents[1]:muted);
  }
 }
 void capture(sf::RenderTarget& target,const fs::path& path){
  sf::RenderTexture rt({1440,900});draw(rt);rt.display();(void)target;
  if(!rt.getTexture().copyToImage().saveToFile(path))throw std::runtime_error("Cannot save screenshot");
 }
public:
 explicit App(fs::path r):root(std::move(r)){
  if(!font.openFromFile(root/"assets"/"NotoSans.ttf"))throw std::runtime_error("assets/NotoSans.ttf is missing");
  reset();
 }
 int run(const std::string& command){
  if(command=="--capture"||command=="--record"){
   fs::create_directories(root/"results"/"frames");sf::RenderTexture rt({1440,900});
   if(command=="--capture"){
    for(int i=0;i<12;++i){steps={};frameMs={};tick();}capture(rt,root/"results"/"tunnel.png");
    changeScene(Scene::Laboratory);for(int i=0;i<120;++i){steps={};frameMs={};tick();}capture(rt,root/"results"/"laboratory.png");
    changeScene(Scene::Stack);for(int i=0;i<300;++i){steps={};frameMs={};tick();}capture(rt,root/"results"/"stack.png");
    selected=0;capture(rt,root/"results"/"stack_discrete.png");
   } else {
    for(int f=0;f<360;++f){
     if(f==72){config.speed=10000;reset();}
     if(f==120){config.speed=2000;changeScene(Scene::Laboratory);scale=1;}
     if(f==240){changeScene(Scene::Stack);scale=1;}
     running=true;steps={};frameMs={};
     if(scene==Scene::Tunnel){
      int local=f<72?f:f-72;
      int interval=f<72?3:8, count=f<72?18:3;
      running=local<interval*count;
      if(running&&local%interval==interval-1)tick();
     }
     else {
      int local=(f-120)%120;
      if(local==40||local==67||local==93)reset();
      compare=local<40;
      selected=local<67?0:local<93?1:2;
      for(int k=0;k<5;++k)tick();
     }
     notice="";
     std::ostringstream name;name<<"frame_"<<std::setw(4)<<std::setfill('0')<<f<<".png";capture(rt,root/"results"/"frames"/name.str());
    }
   }return 0;
  }
  sf::RenderWindow window(sf::VideoMode({1440,900}),utf("Механика | CCD и Multi-rate"));window.setFramerateLimit(60);window.setMinimumSize(sf::Vector2u{1000,650});
  auto resize=[&]{auto sz=window.getSize();float aspect=float(sz.x)/sz.y;sf::View view(sf::FloatRect({0,0},{1440,900}));
   if(aspect>1.6f){float w=1.6f/aspect;view.setViewport({{(1-w)/2,0},{w,1}});}else{float h=aspect/1.6f;view.setViewport({{0,(1-h)/2},{1,h}});}window.setView(view);};resize();
  sf::Clock clock;
  while(window.isOpen()){
   double elapsed=std::min(.1,double(clock.restart().asSeconds()));steps={};frameMs={};
   while(auto event=window.pollEvent()){
    if(event->is<sf::Event::Closed>())window.close();
    if(event->is<sf::Event::Resized>())resize();
    if(auto e=event->getIf<sf::Event::MouseMoved>())mouse=window.mapPixelToCoords(e->position);
    if(auto e=event->getIf<sf::Event::MouseButtonPressed>();e&&e->button==sf::Mouse::Button::Left){
     mouse=window.mapPixelToCoords(e->position);if(help)help=false;else{controls();for(auto&b:buttons)if(b.box.contains(mouse)){b.action();break;}}
    }
    if(auto e=event->getIf<sf::Event::KeyPressed>()){
     using K=sf::Keyboard::Key;auto k=e->code;
     if(k==K::F1)help=!help;else if(k==K::Escape)help=false;
     else if(!help){if(k==K::Space)toggle();if(k==K::R){reset();running=false;}if(k==K::N){running=false;tick();}
      if(k==K::C)compare=!compare;if(k==K::Num1){selected=0;compare=false;}if(k==K::Num2){selected=1;compare=false;}if(k==K::Num3){selected=2;compare=false;}
      if(k==K::F12){fs::create_directories(root/"results");capture(window,root/"results"/"screenshot.png");notice="Сохранено: results/screenshot.png рядом с приложением.";}
     }
    }
   }
   if(running&&!help){accumulator+=elapsed*scale;while(accumulator>=1./config.baseHz){tick();accumulator-=1./config.baseHz;
    if(scene==Scene::Tunnel&&worlds[0].time>=.3){running=false;accumulator=0;notice="Опыт завершён. Сравните результаты; Старт повторит опыт с теми же параметрами.";break;}}}
   draw(window);window.display();
  }return 0;
 }
};
}
int main(int argc,char**argv){
 try{
  fs::path root=fs::absolute(argv[0]).parent_path();
  App app(root);return app.run(argc>1?argv[1]:"");
 }catch(const std::exception&e){
  std::ofstream("mechanics_error.log")<<e.what();
#ifdef _WIN32
  MessageBoxA(nullptr,e.what(),"MechanicsLab",MB_OK|MB_ICONERROR);
#endif
  return 1;
 }
}
