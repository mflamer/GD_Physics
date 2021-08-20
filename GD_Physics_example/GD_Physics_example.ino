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
  model.SetModel(GD.w, GD.h, 1, 64);

  //setup materials
  // r = 5 m
  // A = 78.54 m2 
  // V = 523.6 m3  
                               // D            E           B        F    T         YC          YT           UC           UT 
  rubber    = model.AddMaterial("rubber",     950,      50000000,  .8,   0,    12000000,   -12000000,    16000000,   -16000000);
  concrete  = model.AddMaterial("concrete",  2300,   25000000000,   0,   0,    16000000,   -10000000,    16000000,   -14000000);
  steel     = model.AddMaterial("steel",     7840,  200000000000,   0,   0,   250000000,  -250000000,   400000000,  -400000000);
  wood      = model.AddMaterial("wood",       450,   10000000000,   0,   0,    10000000,   -10000000,    11000000,   -11000000);

  _rubber    = model.AddMaterial("_rubber",    950,      5000000,  .8,   0,     1200000,    -1200000,     1600000,    -1600000);
  _concrete  = model.AddMaterial("_concrete", 2300,   2500000000,   0,   0,     1600000,    -1000000,     1600000,    -1400000);
  _steel     = model.AddMaterial("_steel",    7840,  10000000000,   0,   0,    25000000,   -25000000,    40000000,   -40000000);
  _wood      = model.AddMaterial("_wood",      450,   1000000000,   0,   0,     1000000,    -1000000,     1100000,    -1100000);

  //game_struct = model.InitMaterial(600,    100000000, 30, 0,   2000000,   -2000000);

  //rubber.mass       = 579958;       // 1100 Kg/m3 
  //rubber.spring     = 3927000000;   // EA in Pa/m (A * 0.05GPa) 
  //rubber.damping    = 1 500 000;      // Nm * m/s = Nm2/s ?  
  //rubber.yield_t    = 1099560000;   // N (A * 14MPa) 
  //rubber.yield_c    = -1099560000; 

  // concrete.mass     = 1204280;       // 2300 Kg/m3 
  // concrete.spring   = 1963500000000;   // EA in Pa/m (A * 25GPa) 
  // concrete.damping  = 30 000 000;      // Nm * m/s = Nm2/s ?  
  // concrete.yield_t  = 1099560000;   // 0.5 * yield_c 
  // concrete.yield_c  = -2199120000;  // N (A * 28MPa) 

 

  //DrawBlock(-45, 30, 3, 3, _rubber);//->Fix_Y();
  //DrawBlock(-25, 30, 3, 3, _concrete);//->Fix_XY();
  //DrawBlock(5, 30, 3, 3, _steel);//->Fix_X();
  //DrawBlock(25, 30, 3, 3, _wood);

  //DrawNodes(16);

  // if (!SD.begin(9)) {
  //   Serial.println("initialization failed!");
  //   while (1);
  // }
  // ImportMeshSD("parse_test.txt", &model, &rubber);


  

  // GD.cmd_loadimage(0, 0);
  // GD.load("block.jpg");
  //

  //  File dataFile = SD.open(file_name, FILE_READ);
  //  dataFile.close(); 
  #include "mesh.h"
  ByteStream bs = ByteStream((byte*)mesh_data, strlen(mesh_data));
  ImportModel(&bs, &model);

  model.AddMeshToSim("mesh0");
  model.AddMeshToSim("mesh1");
  model.AddMeshToSim("mesh2");


// 
  // Node* bullet = model.AddNode(-200, 100, 90, 0, &steel);
  // bullet = model.AddNode(-200, 70, 90, 0, &steel);
  // bullet = model.AddNode(-200, 40, 90, 0, &steel);
  // bullet = model.AddNode(-200, 20, 90, 0, &steel);
  
  //Node* n0 = model.AddNode(-9, 25, &rubber);
  //Node* n1 = model.AddNode(-3, 25, &concrete);
  //Node* n2 = model.AddNode(3,  25, &steel);
  //Node* n3 = model.AddNode(9,  25, &wood);
  // t_start = millis();

}



void loop() {
  // GD.get_inputs();
  // if (GD.inputs.x != -32768) {
  //   float x = GD.inputs.x 
  // }

  int t_collide;
  int t_force;
  int t_frame = micros();
  for(int i = 0; i < 40; i++)
  {
    t_collide = micros();    
    model.Collisions();
    t_collide = micros() - t_collide;
    t_force = micros();
    model.Step(1.0/2400.0);  
    t_force = micros() - t_force; 
  }
  
  t_frame = micros() - t_frame;
 //Serial.print("t_collide = "); Serial.print(t_collide); Serial.print("\n");
 //Serial.print("t_force = "); Serial.print(t_force); Serial.print("\n");
 //Serial.print("frames/s = "); Serial.print(1000000/t_frame); Serial.print("\n\n");
  

  GD.ClearColorRGB(0xe0e0e0);
  GD.Clear();
  GD.ColorRGB(30,30,30);
  GD.cmd_number(40, 20, 23, OPT_CENTER, millis() - t_start);
  GD.Begin(POINTS);
  GD.PointSize(int((1 * 64) + 0.5));
  //GD.Begin(BITMAPS);


  model.MapNodes(&dp);

  GD.Begin(LINES);
  GD.LineWidth(32);
  model.MapBars(&draw_bar);

  //model.MapBars(&debug_bar);
  

  GD.swap();

}
