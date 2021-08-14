#include <GD2.h>
#include <plib.h>

float scale = 16;//  subpix / m 
float bar_k = 50000;

 
class DrawPoint : public NodeFunct {
public:
  void operator()(Node* n){
    int xf = int(((n->pos.x * scale) + 0.5) + (GD.w / 2)*16); 
    int yf = int(((-1 * n->pos.y * scale) + 0.5) + (GD.h / 2)*16); 

    GD.Vertex2f(xf, yf);
  }
};

class DrawBar : public BarFunct {
public:
  void operator()(Bar* b){
    int x0 = int(((b->n0->pos.x * scale) + 0.5) + (GD.w / 2)*16); 
    int y0 = int(((-1 * b->n0->pos.y * scale) + 0.5) + (GD.h / 2)*16); 
    int x1 = int(((b->n1->pos.x * scale) + 0.5) + (GD.w / 2)*16); 
    int y1 = int(((-1 * b->n1->pos.y * scale) + 0.5) + (GD.h / 2)*16);

    int red = b->f > 0 ? int(255 * (b->f / (2 * bar_k))) : 0;
    int blu = b->f < 0 ? int(255 * (-(b->f) / (2 * bar_k))) : 0;

    Serial.print(red); Serial.print("\n");

    GD.ColorRGB(red+20, 20, blu+20);
    GD.Vertex2f(x0, y0);
    GD.Vertex2f(x1, y1);
  }
};

class DebugPoint : public NodeFunct {
public:
  void operator()(Node* n){
    Serial.print("\n"); Serial.print("node = "); Serial.print((int)n); Serial.print("\n");    
    Serial.print("vel = "); Serial.print(n->vel.x); Serial.print(", "); Serial.print(n->vel.y); Serial.print("\n");
    Serial.print("pos = "); Serial.print(n->pos.x); Serial.print(", "); Serial.print(n->pos.y); Serial.print("\n");      
  }
};

class DebugBar : public BarFunct {
public:
  void operator()(Bar* b){
      Serial.print("f = "); Serial.print(b->f); Serial.print("\n");
    }
  };

class ArduinoDebuger : public Debuger {
  public:
    void operator()(char* s){Serial.print(s);}
    void operator()(float f){Serial.print(f);}
};






DrawPoint dp;
DrawBar draw_bar;
DebugPoint debug_point;
DebugBar debug_bar;

ArduinoDebuger printer;

Model model(&printer);
Material rubber;  

Node* DrawBlock(float x, float y, float w, float h){

  Node* n0 = model.AddNode(x,         y, &rubber);
  Node* n1 = model.AddNode(x+(1*w),   y, &rubber);
  Node* n2 = model.AddNode(x+(2*w),   y, &rubber);
  Node* n3 = model.AddNode(x+(3*w),   y, &rubber); 
  Node* n4 = model.AddNode(x+(4*w),   y, &rubber);

  Node* n5 = model.AddNode(x,         y+h, &rubber);
  Node* n6 = model.AddNode(x+(1*w),   y+h, &rubber);
  Node* n7 = model.AddNode(x+(2*w),   y+h, &rubber);
  Node* n8 = model.AddNode(x+(3*w),   y+h, &rubber); 
  Node* n9 = model.AddNode(x+(4*w),   y+h, &rubber);

  model.AddBar(n0, n1);
  model.AddBar(n1, n2);
  model.AddBar(n2, n3);
  model.AddBar(n3, n4);

  model.AddBar(n5, n6);
  model.AddBar(n6, n7);
  model.AddBar(n7, n8);
  model.AddBar(n8, n9);

  model.AddBar(n0, n5);
  model.AddBar(n1, n6);
  model.AddBar(n2, n7);
  model.AddBar(n3, n8);
  model.AddBar(n4, n9);

  model.AddBar(n0, n6);
  model.AddBar(n1, n7);
  model.AddBar(n2, n8);
  model.AddBar(n3, n9);

  return n0;
}

void DrawNodes(int n){
  for(int i = 0; i < n; i++){
    model.AddNode(-200 + i * 15, i * 5, &rubber);
  }
}