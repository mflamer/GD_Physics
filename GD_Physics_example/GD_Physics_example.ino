#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>
#include <GD_Physics.h>

int t_start;

void setup() {
  // Serial.begin(115200);
  // while (!Serial) { ;} // debug


  GD.begin();

   //setup material
  // r = 5 m
  // A = 78.54 m2 
  // V = 523.6 m3  
  rubber.mass = 579958; // 1100 Kg/m3 
  rubber.spring = 3927000000; // EA in Pa/m (A * 0.05GPa) 
  rubber.damping = 1500000;       // Nm * m/s = Nm2/s ?  
  rubber.yield_t = 1099560000; // N (A * 14MPa) 
  rubber.yield_c = -1099560000;



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
  
  #include "mesh.h"
  ImportMeshBuffer(test_data, &model, &rubber);
  
  //Node* n0 = model.AddNode(0, 0, &rubber);
  t_start = millis();

}



void loop() {
  // GD.get_inputs();
  // if (GD.inputs.x != -32768) {
  //   float x = GD.inputs.x 
  // }

  int t_collide;
  int t_force;
  for(int i = 0; i < 50; i++)
  {
    t_collide = micros();    
    model.Collisions(); 
    t_collide = micros() - t_collide;
    t_force = micros();
    model.Step(1.0/3000.0);  
    t_force = micros() - t_force; 
  }
  
  Serial.print("t_collide = "); Serial.print(t_collide); Serial.print("\n");
  Serial.print("t_force = "); Serial.print(t_force); Serial.print("\n");
  //model.MapNodes(&debug_point);

  GD.ClearColorRGB(0xe0e0e0);
  GD.Clear();
  GD.ColorRGB(30,30,30);
  GD.cmd_number(40, 20, 23, OPT_CENTER, millis() - t_start);
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
