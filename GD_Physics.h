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
		void operator()(char* s){Serial.print(s);}
		void operator()(float f){Serial.print(f);}
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





// void ImportMeshSD(const char* file_name, Model* model, Material* material){
// 	File dataFile = SD.open(file_name, FILE_READ);
//     Serial.print("running the test \n");
//     Serial.print((int)dataFile);		
// 	if(dataFile)
// 	{
// 		int base_idx = model->SizeNodes();
//         char c;
//     	do{
//             c = dataFile.read();
//             Serial.print(c);
//             if(c == 'N'){
//                 float x = dataFile.parseFloat();
//                 float y = dataFile.parseFloat();
//                 Serial.print((int)model->AddNode(x, y, material));
//             }
//             else if(c == 'B'){
//                 int idx_n0 = dataFile.parseInt();
//                 int idx_n1 = dataFile.parseInt();             
//                 Node* n0 = model->GetNodeIdx(idx_n0 + base_idx);
//                 Node* n1 = model->GetNodeIdx(idx_n1 + base_idx);
//                  Serial.print((int)model->AddBar(n0, n1));       
//             }
//         }while(c != -1);
// 		dataFile.close();
// 	}
// }

void ImportMeshBuffer(const unsigned char* buff, Model* model){          
    if(buff)
    {
        ByteStream bs = ByteStream(buff, strlen(buff));
        int base_idx = model->SizeNodes();
        Material* material = model->GetMaterial(); 
        int tag = 0;
        char c = bs.read();
        while(bs.available()){           
            if(c == 'N'){ // nodes
                float x = bs.parseFloat();
                float y = bs.parseFloat();
                model->AddNode(x, y, material, tag);
            }
            else if(c == 'B'){ // bars
                int idx_n0 = bs.parseInt();
                int idx_n1 = bs.parseInt();             
                Node* n0 = model->GetNodeIdx(idx_n0 + base_idx);
                Node* n1 = model->GetNodeIdx(idx_n1 + base_idx);
                model->AddBar(n0, n1);
            }
            else if(c == 'M'){ // material
                char name[32]; int i = 0;
                do{
                    name[i] = bs.read();
                }while(c != ';' && bs.available());
                material = model->GetMaterial(name);                
            }
            else if(c == 'T'){ // tag
                tag = bs.parseInt();
            }
            c = bs.read();
        };
    }
}