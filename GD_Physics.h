#include <GD2.h>
//#include <SD.h>
#include <plib.h>
#include <ByteStream.h>

 
class DrawPoint : public NodeFunct {
public:
	void operator()(Node* n){
		int xf = int(((n->pos.x * 64) + 0.5) + (GD.w / 2)*16); //!!!!fix scale
		int yf = int(((-1 * n->pos.y * 64) + 0.5) + (GD.h / 2)*16); //!!!!fix scale

		GD.Vertex2f(xf, yf);
	}
};

class DrawBar : public BarFunct {
public:
	void operator()(Bar* b){
		int x0 = int(((b->n0->pos.x * 64) + 0.5) + (GD.w / 2)*16); //!!!!fix scale
		int y0 = int(((-1 * b->n0->pos.y * 64) + 0.5) + (GD.h / 2)*16); 
		int x1 = int(((b->n1->pos.x * 64) + 0.5) + (GD.w / 2)*16); 
		int y1 = int(((-1 * b->n1->pos.y * 64) + 0.5) + (GD.h / 2)*16);//!!!!fix scale

		float lim_t = (b->n0->mat->yield_t + b->n1->mat->yield_t) / 2;
        float lim_c = (b->n0->mat->yield_c + b->n1->mat->yield_c) / 2;
        int red = b->f > 0 ? int(255 * (b->f / (.3 * lim_t))) : 0;
		int blu = b->f < 0 ? int(255 * (b->f / (.3 * lim_c))) : 0;

		//Serial.print(red); Serial.print("\n");

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

class ArduinoPrinter : public Printer {
	public:
		void operator()(const char* s){Serial.print(s);}
		void operator()(float f){Serial.print(f);}
        void operator()(int i){Serial.print(i);}
};






DrawPoint dp;
DrawBar draw_bar;
DebugPoint debug_point;
DebugBar debug_bar;


Printer* p = new ArduinoPrinter;
Model model(p);

 

Material*   rubber;
Material*   concrete;
Material*   steel;
Material*   wood;
Material*   _rubber;
Material*   _concrete;
Material*   _steel;
Material*   _wood;
Material*   game_struct;



Node* DrawBlock(float x, float y, float w, float h, Material* m){

	Node* n0 = model.AddNode(x,         y, m);
	Node* n1 = model.AddNode(x+(1*w),   y, m);
	Node* n2 = model.AddNode(x+(2*w),   y, m);
	Node* n3 = model.AddNode(x+(3*w),   y, m); 
	Node* n4 = model.AddNode(x+(4*w),   y, m);

	Node* n5 = model.AddNode(x,         y+h, m);
	Node* n6 = model.AddNode(x+(1*w),   y+h, m);
	Node* n7 = model.AddNode(x+(2*w),   y+h, m);
	Node* n8 = model.AddNode(x+(3*w),   y+h, m); 
	Node* n9 = model.AddNode(x+(4*w),   y+h, m);

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

// Node* DrawBlock(float x, float y, int nx, int ny, float spac, Material* mat){

//   for(int i = 0; i < ny; i++){
//     for(int j = 0; j < xy; j++){
//       Node* n = model.AddNode(x + (j * spac), y + (i * spac), mat);
//   }

//   return n0;
// }

void DrawNodes(int n){
	for(int i = 0; i < n; i++){
		model.AddNode(-200 + i * 15, i * 5, concrete);
	}
}







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
                mesh->AddBar(n0, n1);
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

