#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>
#include <GD_Physics.h>



void setup() {
  // Serial.begin(115200);
  // while (!Serial) { ;} // debug


  GD.begin();

   //setup material
  rubber.mass = 400; //??
  rubber.spring = 300000; //??
  rubber.damping = 400;
  rubber.yield_t = 600000;
  rubber.yield_c = -600000;



  //setup model
  model.SetModel(GD.w, GD.h, 5);

  //DrawBlock(-30, 120, 12, 12)->Fix_Y();
  //DrawBlock(0, 100, 12, 12)->Fix_XY();
  //DrawBlock(30, 80, 12, 12)->Fix_X();

  //DrawNodes(16);

  // if (!SD.begin(9)) {
  //   Serial.println("initialization failed!");
  //   while (1);
  // }
  // ImportMeshSD("parse_test.txt", &model, &rubber);


  

  // GD.cmd_loadimage(0, 0);
  // GD.load("g_sphere.jpg");

  //const unsigned char* test_data = "N 100.0 100.0 N -25.25 77.0 B 0 1";
  #include "mesh.h"
  ImportMeshBuffer(test_data, &model, &rubber);
  

  

}



void loop() {
  // GD.get_inputs();
  // if (GD.inputs.x != -32768) {
  //   float x = GD.inputs.x 
  // }

  int time = micros(); 
  for(int i = 0; i < 50; i++)
  {    
    model.Collisions(); 
    model.Step(1.0/1000.0);     
  }
  time = micros() - time;
  Serial.print("Step = "); Serial.print(1000000 / time); Serial.print("\n");
  
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
