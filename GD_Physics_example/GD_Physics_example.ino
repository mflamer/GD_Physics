#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>
#include <GD_Physics.h>

int t_start;

void setup() {
  // Serial.begin(115200);
  // while (!Serial) { ;} // debug


  GD.begin();

   //setup model
  model.SetModel(GD.w, GD.h, 5);

  //setup materials
  // r = 5 m
  // A = 78.54 m2 
  // V = 523.6 m3  
                               // D       E           B       F       T            C 
  rubber    = model.InitMaterial(1100, 50000000,     8,     0,   14000000,   -14000000);
  concrete  = model.InitMaterial(2300, 25000000000,  500,    0,   14000000,   -28000000);
  steel     = model.InitMaterial(7840, 200000000000, 1000,    0,   400000000,  -400000000);
  wood      = model.InitMaterial(450,  10000000000,  450,    0,   20000000,   -20000000);

  //rubber.mass       = 579958;       // 1100 Kg/m3 
  //rubber.spring     = 3927000000;   // EA in Pa/m (A * 0.05GPa) 
  //rubber.damping    = 1500000;      // Nm * m/s = Nm2/s ?  
  //rubber.yield_t    = 1099560000;   // N (A * 14MPa) 
  //rubber.yield_c    = -1099560000; 

  // concrete.mass     = 1204280;       // 2300 Kg/m3 
  // concrete.spring   = 1963500000000;   // EA in Pa/m (A * 25GPa) 
  // concrete.damping  = 30000000;      // Nm * m/s = Nm2/s ?  
  // concrete.yield_t  = 1099560000;   // 0.5 * yield_c 
  // concrete.yield_c  = -2199120000;  // N (A * 28MPa) 

 

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
  
  //Node* n0 = model.AddNode(-45, 100, &rubber);
  //Node* n1 = model.AddNode(-15, 100, &concrete);
  //Node* n2 = model.AddNode(15,  100, &steel);
  //Node* n3 = model.AddNode(45,  100, &wood);
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
  
  //Serial.print("t_collide = "); Serial.print(t_collide); Serial.print("\n");
  //Serial.print("t_force = "); Serial.print(t_force); Serial.print("\n");
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
