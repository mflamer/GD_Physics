#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>
#include <GD_Physics.h>

//#include "assets.h"


Model model(new ArduinoDebugger);

void setup() {


  GD.begin();

  //LOAD_ASSETS();

   //setup model
  
  model.InitModel(GD.w/4, GD.h/4, 1);

  GD.BitmapHandle(0);
  GD.cmd_loadimage(0, 0);
  GD.load("c_ball.png");
  GD.BitmapHandle(1);
  GD.cmd_loadimage(-1, 0); // cell 0
  GD.load("blk.png");
  GD.BitmapHandle(2);
  GD.cmd_loadimage(-1, 0); // cell 1
  GD.load("blk_d0.png");
  GD.BitmapHandle(3);
  GD.cmd_loadimage(-1, 0); // cell 2
  GD.load("blk_d1.png");

  GD.BitmapHandle(4);
  GD.cmd_loadimage(-1, 0); // cell 2
  GD.load("skyline.png");
  
  

  //model.AddFunctToTagMap(0, new DrawNode_Basic(64));
  model.AddFunctToTagMap(0, new DrawNode_Bitmap(0));  // cannon ball

  model.AddFunctToTagMap(1, new DrawNode_Bitmap(1));  // brick
  model.AddFunctToTagMap(2, new DrawNode_Bitmap(2));  // brick
  model.AddFunctToTagMap(3, new DrawRotatedBitmap(3));  // brick  

  model.AddFunctToTagMap(0, new DrawBar_Stress(16));
  
  model.SetBarDestructionEvent(new BarDestroyer());


  #include "mesh.h"
  ByteStream bs = ByteStream((byte*)mesh_data, strlen(mesh_data));
  ImportModel(&bs, &model);


  model.AddMeshToSim("building");

  

}


int pause;
void loop() {
  GD.get_inputs();  
  if (GD.inputs.x != -32768 && pause == 0) {
    float x = ScreenToModel_X(GD.inputs.x);
    float y = ScreenToModel_Y(GD.inputs.y); 
    Node* bullet = model.AddNode(x, y, 80, 0, model.GetMaterial("_steel"), 0);
    pause = 30;
  }
  if(pause > 0)pause--;
  else pause = 0; 


  int fps =  model.Frame(30);  

  GD.ClearColorRGB(0xe0e0e0);
  GD.Clear();

  GD.Begin(BITMAPS);
  GD.Vertex2ii( 0, 0, 4, 0);  

  GD.ColorRGB(30, 30, 30);
  GD.cmd_number(40, 20, 23, OPT_CENTER, fps);

  

  model.BatchBarsByTag();
  model.BatchNodesByTag();
  

  GD.swap();

}
