#include "plib.h"
#include <algorithm>


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


Node::Node(float x, float y, Material* mat){
	pos.x = x; pos.y = y; k = mat->E; 
	force.x = 0; force.y = 0;
	acc.x = 0; acc.y = 0;
	vel.x = 0; vel.y = 0; 
	m = (4/3) * 3.14159265359 * pow(radius, 3) * mat->D;
}


float Bar::DampingForce(){
	V2 rel_v;
	rel_v.x = n1->vel.x - n0->vel.x;
	rel_v.y = n1->vel.y - n0->vel.y;
	return elast_damping * rel_v.Mag();
}

// +F = tension, -F = compression
float Bar::Force(V2& v){
	float l_ = n0->pos.Distance(n1->pos); // current length
	float def = l_ - l; // current deflection
	float f_dmp = DampingForce();
	f = (def * k) + f_dmp;  // save f for graphics
	v.x = f * (n1->pos.x - n0->pos.x) / l_;
	v.y = f * (n1->pos.y - n0->pos.y) / l_;
	return f;
}

Node* Model::AddNode(float x, float y, Material* m){
	Node* n = new Node(x, y, m);
	nodes.push_back(n);
	return n;
}

Bar* Model::AddBar(Node* n0, Node* n1, float k){
	Bar* b = new Bar();
	b->n0 = n0;
	b->n1 = n1;
	b->k = k;
	b->l = n0->pos.Distance(n1->pos);
	bars.push_back(b);
	return b;
}

Model::Model(Debuger* d){	
	printer = d;
}

void Model::SetModel(float w, float h){
	width = w;
	height = h;
}

Model::~Model(){
	std::vector<Node*>::iterator itr;
	for(itr = nodes.begin(); itr != nodes.end(); itr++){
		delete *itr;
	}
}	

void Model::Step(float t){
	// forces from bars
	std::vector<Bar*>::iterator b_itr;
	for(b_itr = bars.begin(); b_itr != bars.end(); b_itr++){
		V2 f;
		(*b_itr)->Force(f);
		(*b_itr)->n0->force.x += f.x;
		(*b_itr)->n0->force.y += f.y;
		(*b_itr)->n1->force.x -= f.x;
		(*b_itr)->n1->force.y -= f.y;
	}

	// node dynamics for  t step
	std::vector<Node*>::iterator itr;
	for(itr = nodes.begin(); itr != nodes.end(); itr++){
		// update velocity for next step
		(*itr)->vel.x += (*itr)->acc.x * t;
		(*itr)->vel.y += (*itr)->acc.y * t;

		// apply velocity damping
		(*itr)->force.x -= (*itr)->vel.x * air_damping;
		(*itr)->force.y -= (*itr)->vel.y * air_damping;

		// update acceleration
		(*itr)->acc.x = ((*itr)->force.x / (*itr)->m);
		(*itr)->acc.y = ((*itr)->force.y / (*itr)->m) - 9.81;

		// update position
		(*itr)->pos.x += ((*itr)->vel.x * t) + (((*itr)->acc.x * t * t) / 2);
		(*itr)->pos.y += ((*itr)->vel.y * t) + (((*itr)->acc.y * t * t) / 2);	
	
	}
}

void Model::Collisions(){
	sort(nodes.begin(), nodes.end(), NodeLess());
	std::vector<Node*>::iterator n;
	std::vector<Node*>::iterator m;
	//clear forces
	for(n = nodes.begin(); n != nodes.end(); n++){ 
		(*n)->force.x = 0;
		(*n)->force.y = 0;
	}
	for(n = nodes.begin(); n != nodes.end(); n++){
		for(m = n + 1; m != nodes.end(); m++){
			if((*m)->pos.x - (*n)->pos.x > (radius * 2)) break;
			float d = (*m)->pos.Distance((*n)->pos);
			if(d < radius * 2){
				float f = ((2 * radius) - d) * ((*m)->k + (*n)->k) * 0.5;  
				float fx = f * (((*m)->pos.x - (*n)->pos.x) / d);
				float fy = f * (((*m)->pos.y - (*n)->pos.y) / d); 
				(*m)->force.x += fx;
				(*m)->force.y += fy;
				(*n)->force.x -= fx;
				(*n)->force.y -= fy;
			}
		}

		// test left and right model edges
		float dx_l = (-width / 2) - ((*n)->pos.x - radius);
		float dx_r = (width / 2) - ((*n)->pos.x + radius);		
		if(dx_l > 0){(*n)->force.x += dx_l * (*n)->k;}
		if(dx_r < 0){(*n)->force.x += dx_r * (*n)->k;}

		// test top and bottom model edges
		float dy_b = (-height / 2) - ((*n)->pos.y - radius);
		float dy_t = (height / 2) - ((*n)->pos.y + radius);		
		if(dy_b > 0){(*n)->force.y += dy_b * (*n)->k;}
		if(dy_t < 0){(*n)->force.y += dy_t * (*n)->k;}
		

	}

}





void Model::MapNodes(NodeFunctor* f){
	std::vector<Node*>::iterator n;
	for(n = nodes.begin(); n != nodes.end(); n++){
		(*f)(*n);
	}
}

void Model::MapBars(BarFunctor* f){
	std::vector<Bar*>::iterator b;
	for(b = bars.begin(); b != bars.end(); b++){
		(*f)(*b);
	}
}
