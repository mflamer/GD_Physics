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

  GD.BlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);

  //setup material
  rubber.density = 1; //??
  rubber.elastic = 100000; //??
  rubber.damping = 500;



  //setup model
  model.SetModel(GD.w, GD.h);

  // DrawBlock(-30, 120, 12, 12);
  // DrawBlock(0, 100, 12, 12);
  // DrawBlock(30, 80, 12, 12);

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
  GD.PointSize(int((radius * scale) + 0.5));
  //GD.Begin(BITMAPS);


  model.MapNodes(&dp);

  GD.Begin(LINES);
  GD.LineWidth(16 * 2);
  model.MapBars(&draw_bar);

  //model.MapBars(&debug_bar);
  

  GD.swap();

}
