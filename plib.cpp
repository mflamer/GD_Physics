#include "plib.h"
#include <algorithm>
#include <stdio.h>


float V2::Distance(V2& v)
{ 
	return sqrt(pow(x - v.x, 2) + pow(y - v.y, 2));
}

float V2::Mag(){
	return sqrt(x * x + y * y);
}

float V2::Dot(V2& v){
	return (x * v.x) + (y * v.y);
}

V2 V2::Unit(){
	V2 u;
	float m = Mag(); 
	u.x = x / m;
	u.y = y / m;
	return u;
}

V2	V2::operator-(){
	V2 result;
	result.x = -x; 
	result.y = -y;
	return result;
	}

void Node::ApplyForce(float fx, float fy){
	if(force.x != 0xFFFFFFFF) force.x += fx;
	if(force.y != 0xFFFFFFFF) force.y += fy;
}	

void Node::ApplyDampedForce(const V2& f){
	V2 funit = f.Unit();
	float f_damp = mat->damping * vel.Dot(funit);
	float f_damp_x = f_damp * f.x / f.Mag();
	float f_damp_y = f_damp * f.y / f.Mag();
	ApplyForce(f.x - f_damp_x, f.y - f_damp_y);	
}

V2	Bar::Dir(){
	V2 dir;
	dir.x = n1->pos.x - n0->pos.x;
	dir.y = n1->pos.y - n0->pos.y;
	return dir; 
}


// +F = tension, -F = compression
float Bar::Force(){
	// bar force
	float k = (n0->mat->spring + n1->mat->spring) * 0.5;
	float l_ = n0->pos.Distance(n1->pos); // current length
	float def = l_ - l; // current deflection
	
	// bar velocity damping
	float damp = (n0->mat->damping + n1->mat->damping) * 0.5;
	V2 rel_v;	
	rel_v.x = n1->vel.x - n0->vel.x;
	rel_v.y = n1->vel.y - n0->vel.y;

	// apply to nodes
	V2 dir = Dir();
	f = (def * k) + (damp * rel_v.Dot(dir));// save f for graphics
	V2 F;
	F.x = f * (n1->pos.x - n0->pos.x) / l_;
	F.y = f * (n1->pos.y - n0->pos.y) / l_;
	n0->ApplyForce(F.x, F.y);
	n1->ApplyForce(-F.x, -F.y);
	return f;
}


Node* Model::AddNode(float x, float y, Material* m){
	Node* n = new Node();
	n->pos.x = x; n->pos.y = y; n->mat = m; 	 
	//n->m = (4/3) * 3.14159265359 * pow(radius, 3) * mat->density;
	nodes.push_back(n);
	return n;
}

Bar* Model::AddBar(Node* n0, Node* n1){
	Bar* b = new Bar();
	b->n0 = n0;
	b->n1 = n1;
	b->l = n0->pos.Distance(n1->pos);
	bars.push_back(b);
	return b;
}

Model::Model(Debuger* d){	
	printer = d;
}

void Model::SetModel(float w, float h, float r){
	width = w;
	height = h;
	radius = r;
}

Model::~Model(){
	std::vector<Node*>::iterator itr;
	for(itr = nodes.begin(); itr != nodes.end(); itr++){
		delete *itr;
	}
}	

void Model::Step(float t){
	// apply forces from bars
	std::vector<Bar*>::iterator b_itr;
	for(b_itr = bars.begin(); b_itr != bars.end(); b_itr++){
		float bf = (*b_itr)->Force();
		//test for bar yield and remove if so
		if((bf > (*b_itr)->Yield_T()) | (bf < (*b_itr)->Yield_C())) {
			delete *b_itr;
			b_itr = bars.erase(b_itr);
			if(b_itr == bars.end()) break;
		}
	}

	// node dynamics for  t step
	std::vector<Node*>::iterator itr;
	for(itr = nodes.begin(); itr != nodes.end(); itr++){	

		// step if component not locked
		if((*itr)->force.x != 0xFFFFFFFF){
			// apply nodal velocity damping
			(*itr)->force.x -= (*itr)->vel.x * fluid_damping;
			// update acceleration
			float acc_x = ((*itr)->force.x / (*itr)->mat->mass);
			// update velocity for next step
			(*itr)->vel.x += acc_x * t;
			// update position
			(*itr)->pos.x += ((*itr)->vel.x * t);
		}	
		// do the same for y
		if((*itr)->force.y != 0xFFFFFFFF){
			(*itr)->force.y -= (*itr)->vel.y * fluid_damping;		
			float acc_y = ((*itr)->force.y / (*itr)->mat->mass) + gravity;		
			(*itr)->vel.y += acc_y * t;		
			(*itr)->pos.y += ((*itr)->vel.y * t);
		}		
	
	}
}

void Model::Collisions(){
	sort(nodes.begin(), nodes.end(), NodeLess());
	std::vector<Node*>::iterator n;
	std::vector<Node*>::iterator m;
	//clear forces
	for(n = nodes.begin(); n != nodes.end(); n++){ 
		if((*n)->force.x != 0xFFFFFFFF)(*n)->force.x = 0;
		if((*n)->force.y != 0xFFFFFFFF)(*n)->force.y = 0;
	}
	for(n = nodes.begin(); n != nodes.end(); n++){
		for(m = n + 1; m != nodes.end(); m++){
			if((*m)->pos.x - (*n)->pos.x > (radius * 2)) break;
			float d = (*m)->pos.Distance((*n)->pos);
			if(d < radius * 2){
				float avg_elasticity = ((*m)->mat->spring + (*n)->mat->spring) * 0.5;
				float f = ((2 * radius) - d) * avg_elasticity;
				V2 collision_f;  
				collision_f.x = f * (((*m)->pos.x - (*n)->pos.x) / d);
				collision_f.y = f * (((*m)->pos.y - (*n)->pos.y) / d); 

				(*m)->ApplyDampedForce(collision_f);
				(*n)->ApplyDampedForce(-collision_f);
			}
		}

		// test left and right model edges
		float dx_l = (-width / 2) - ((*n)->pos.x - radius);
		float dx_r = (width / 2) - ((*n)->pos.x + radius);		
		if(dx_l > 0){(*n)->ApplyDampedForce(V2(dx_l * (*n)->mat->spring, 0));}
		if(dx_r < 0){(*n)->ApplyDampedForce(V2(dx_r * (*n)->mat->spring, 0));}

		// test top and bottom model edges
		float dy_b = (-height / 2) - ((*n)->pos.y - radius);
		float dy_t = (height / 2) - ((*n)->pos.y + radius);		
		if(dy_b > 0){(*n)->ApplyDampedForce(V2(0, dy_b * (*n)->mat->spring));}
		if(dy_t < 0){(*n)->ApplyDampedForce(V2(0, dy_t * (*n)->mat->spring));}
		

	}

}





void Model::MapNodes(NodeFunct* f){
	std::vector<Node*>::iterator n;
	for(n = nodes.begin(); n != nodes.end(); n++){
		(*f)(*n);
	}
}

void Model::MapBars(BarFunct* f){
	std::vector<Bar*>::iterator b;
	for(b = bars.begin(); b != bars.end(); b++){
		(*f)(*b);
	}
}

// void Model::Import(const char* file_name, Material* m){
// 	FILE* pFile;
//   	pFile = fopen (file_name, "r");
//   	if (pFile!=NULL)
//   	{
//   		int base_idx = nodes.size();
// 		char line[64];
// 		while(fgets(line , 64, pFile) != NULL){
// 			if(line[0] == 'N'){ // parse node
// 				float x;
// 				float y;
// 				sscanf(line, "N %f %f ", &x, &y);
// 				AddNode(x, y, m);
// 			}
// 			else if(line[0] == 'B'){ // parse bar
// 				int idx_n0;
// 				int idx_n1;
// 				sscanf(line, "B %i %i ", &idx_n0, &idx_n1);
// 				Node* n0 = nodes.at(idx_n0 + base_idx);
// 				Node* n1 = nodes.at(idx_n1 + base_idx);
// 				AddBar(n0, n1);				
// 			}
// 		}

//   		fclose (pFile);
//   	}
// }
