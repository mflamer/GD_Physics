#include <GD2.h>
//#include <SD.h>
#include <plib.h>
#include <ByteStream.h>

static const float scale = 64; //subpix/m 


int     ModelToScreen_X(float x){return (int)(((x * scale) + 0.5) + (GD.w * 8));}
int     ModelToScreen_Y(float y){return (int)(((-y * scale) + 0.5) + (GD.h * 8));}
float   ScreenToModel_X(int x){return (x - (GD.w/2)) / (scale/16);}
float   ScreenToModel_Y(int y){return (-y + (GD.h/2)) / (scale/16);}


class DrawNode_Basic : public NodeFunct {
public:
    DrawNode_Basic(int r){radius = r;}// r in subpix
    void Init(){
        GD.Begin(POINTS);
        GD.PointSize(radius);
        GD.ColorRGB(120,120,120);
    }
	void operator()(Node* n){
		GD.Vertex2f(ModelToScreen_X(n->pos.x), ModelToScreen_Y(n->pos.y));
    }

    int radius;
};

class DrawNode_Bitmap : public NodeFunct {
public:
    DrawNode_Bitmap(int h){handle = h;}// r in subpix
    void Init(){
        GD.ColorRGB(0xFFFFFF);
        GD.Begin(BITMAPS);        
    }
    void operator()(Node* n){
        int x = (ModelToScreen_X(n->pos.x) / 16) - 6;//!!
        int y = (ModelToScreen_Y(n->pos.y) / 16) - 6;//!!
        GD.Vertex2ii( x, y, handle, 0);
        //if(handle == 1){Serial.print("hnd = "); Serial.print(handle); Serial.print("\n");}
    }

    int handle;
};

class DrawBar_Stress : public BarFunct {
public:
    DrawBar_Stress(int w){width = w;}// w in subpix
    void Init(){
        GD.Begin(LINES);
        GD.LineWidth(width);
        GD.BlendFunc(SRC_ALPHA, ONE);
    }
	void operator()(Bar* b){
        int red = b->f > 0 ? int(255 * (b->f / (b->Ult_T()))) : 0;
		int blu = b->f < 0 ? int(255 * (b->f / (b->Ult_C()))) : 0;		

		GD.ColorRGB(red, 0, blu);
		GD.Vertex2f(ModelToScreen_X(b->n0->pos.x), ModelToScreen_Y(b->n0->pos.y));
		GD.Vertex2f(ModelToScreen_X(b->n1->pos.x), ModelToScreen_Y(b->n1->pos.y));
	}

    int width;
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

class ArduinoDebugger : public Debugger {
	public:
		void print(const char* s){Serial.print(s);}
		void print(float f){Serial.print(f);}
        void print(int i){Serial.print(i);}
        int ticks(){return micros();}
};



//DebugPoint debug_point;
//DebugBar debug_bar;
 


// Node* DrawBlock(float x, float y, float w, float h, Material* m){

// 	Node* n0 = model.AddNode(x,         y, m);
// 	Node* n1 = model.AddNode(x+(1*w),   y, m);
// 	Node* n2 = model.AddNode(x+(2*w),   y, m);
// 	Node* n3 = model.AddNode(x+(3*w),   y, m); 
// 	Node* n4 = model.AddNode(x+(4*w),   y, m);

// 	Node* n5 = model.AddNode(x,         y+h, m);
// 	Node* n6 = model.AddNode(x+(1*w),   y+h, m);
// 	Node* n7 = model.AddNode(x+(2*w),   y+h, m);
// 	Node* n8 = model.AddNode(x+(3*w),   y+h, m); 
// 	Node* n9 = model.AddNode(x+(4*w),   y+h, m);

// 	model.AddBar(n0, n1);
// 	model.AddBar(n1, n2);
// 	model.AddBar(n2, n3);
// 	model.AddBar(n3, n4);

// 	model.AddBar(n5, n6);
// 	model.AddBar(n6, n7);
// 	model.AddBar(n7, n8);
// 	model.AddBar(n8, n9);

// 	model.AddBar(n0, n5);
// 	model.AddBar(n1, n6);
// 	model.AddBar(n2, n7);
// 	model.AddBar(n3, n8);
// 	model.AddBar(n4, n9);

// 	model.AddBar(n0, n6);
// 	model.AddBar(n1, n7);
// 	model.AddBar(n2, n8);
// 	model.AddBar(n3, n9);

// 	return n0;
// }


// void DrawNodes(int n){
// 	for(int i = 0; i < n; i++){
// 		model.AddNode(-200 + i * 15, i * 5, concrete);
// 	}
// }


//  File dataFile = SD.open(file_name, FILE_READ);
//  dataFile.close(); 
void ImportModel(Stream* stream, Model* model){          
    if(stream)
    {        
        Mesh* mesh = NULL;
        Material* material = NULL; 
        int tag = 0;
        char c = stream->read();
        while(stream->available()){           
            if(c == 'X'){ // mesh
                String name;
                c = stream->read();
                for(int i = 0; i < 32 && stream->available(); i++){
                    c = stream->read(); 
                    if(c == ';'){break;}
                    name += c;
                }           
                mesh = new Mesh();
                model->AddMeshToModel(mesh, name.c_str()); 
            }
            else if(c == 'N'){ // nodes
                float x = stream->parseFloat();
                float y = stream->parseFloat();
                mesh->AddNode(x, y, material, tag);
            }
            else if(c == 'B'){ // bars
                int idx_n0 = stream->parseInt();
                int idx_n1 = stream->parseInt();             
                Node* n0 = mesh->GetNodeIdx(idx_n0);
                Node* n1 = mesh->GetNodeIdx(idx_n1);
                mesh->AddBar(n0, n1, tag);
            }
            else if(c == 'M'){ // material
                String name;
                c = stream->read();
                for(int i = 0; i < 32 && stream->available(); i++){
                    c = stream->read(); 
                    if(c == ';'){break;}
                    name += c;
                }             
                material = model->GetMaterial(name.c_str());               
            }
            else if(c == 'T'){ // tag
                tag = stream->parseInt();
            }
            c = stream->read();
        };
    }
}

