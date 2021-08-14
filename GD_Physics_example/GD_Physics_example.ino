#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>
#include <GD_Physics.h>


void setup() {
  //Serial.begin(115200);
  //while (!Serial) { ;} // debug

  GD.begin();

  GD.cmd_loadimage(0, 0);
  GD.load("g_sphere.jpg");

  

  //setup material
  rubber.mass = 400; //??
  rubber.spring = 100000; //??
  rubber.damping = 500;
  rubber.yield_t = 100000;
  rubber.yield_c = -100000;



  //setup model
  model.SetModel(GD.w, GD.h, 5);

  DrawBlock(-30, 120, 12, 12)->Fix_Y();
  DrawBlock(0, 100, 12, 12)->Fix_XY();
  DrawBlock(30, 80, 12, 12)->Fix_X();

  DrawNodes(16);

}



void loop() {
    
  for(int i = 0; i < 100; i++)
  {
    model.Collisions(); 
    model.Step(1.0/2000.0);       
  }
  
  //model.MapNodes(&debug_point);

  GD.ClearColorRGB(0xe0e0e0);
  GD.Clear();
  GD.ColorRGB(30,30,30);
  GD.Begin(POINTS);
  GD.PointSize(int((5 * scale) + 0.5));
  //GD.Begin(BITMAPS);


  model.MapNodes(&dp);

  GD.Begin(LINES);
  GD.LineWidth(16 * 2);
  model.MapBars(&draw_bar);

  //model.MapBars(&debug_bar);
  

  GD.swap();

}
