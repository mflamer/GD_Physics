#include "plib.h"
#include <algorithm>


float V2::Distance(const V2& v)
{ 
	return sqrt(pow(x - v.x, 2) + pow(y - v.y, 2));
}

float V2::Mag(){
	return sqrt(x * x + y * y);
}

float V2::Dot(const V2& v){
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

float Node::ApplyDampedForce(const V2& f){
	float f_damp = elast_damping * vel.Dot(f.Unit());
	float f_damp_x = f_damp * f.x / f.Mag();
	float f_damp_y = f_damp * f.y / f.Mag();
	force.x += (f.x - f_damp_x);
	force.y += (f.y - f_damp_y);
	return f_damp_y;
}


float Bar::DampingForce(){
	V2 rel_v;
	V2 rel_p;
	rel_p.x = n1->pos.x - n0->pos.x;
	rel_p.y = n1->pos.y - n0->pos.y;
	rel_v.x = n1->vel.x - n0->vel.x;
	rel_v.y = n1->vel.y - n0->vel.y;

	return elast_damping * rel_v.Dot(rel_p);
}

// +F = tension, -F = compression
float Bar::Force(V2& v){
	float l_ = n0->pos.Distance(n1->pos); // current length
	float def = l_ - l; // current deflection
	f = (def * k) + DampingForce();// save f for graphics
	v.x = f * (n1->pos.x - n0->pos.x) / l_;
	v.y = f * (n1->pos.y - n0->pos.y) / l_;
	return f;
}

Node* Model::AddNode(float x, float y, Material* mat){
	Node* n = new Node();
	n->pos.x = x; n->pos.y = y; n->k = mat->elastic; 	 
	n->m = (4/3) * 3.14159265359 * pow(radius, 3) * mat->density;
	nodes.push_back(n);
	return n;
}

Bar* Model::AddBar(Node* n0, Node* n1, Material* m){
	Bar* b = new Bar();
	b->n0 = n0;
	b->n1 = n1;
	b->k = m->elastic;
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
	// apply forces from bars
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

		// apply nodal velocity damping
		(*itr)->force.x -= (*itr)->vel.x * air_damping;
		(*itr)->force.y -= (*itr)->vel.y * air_damping;

		// update acceleration
		float acc_x = ((*itr)->force.x / (*itr)->m);
		float acc_y = ((*itr)->force.y / (*itr)->m) + gravity;

		// update velocity for next step
		(*itr)->vel.x += acc_x * t;
		(*itr)->vel.y += acc_y * t;

		// update position
		(*itr)->pos.x += ((*itr)->vel.x * t);
		(*itr)->pos.y += ((*itr)->vel.y * t);
	
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
				V2 col_f;  
				col_f.x = f * (((*m)->pos.x - (*n)->pos.x) / d);
				col_f.y = f * (((*m)->pos.y - (*n)->pos.y) / d); 

				(*m)->ApplyDampedForce(col_f);
				(*n)->ApplyDampedForce(-col_f);
			}
		}

		// test left and right model edges
		float dx_l = (-width / 2) - ((*n)->pos.x - radius);
		float dx_r = (width / 2) - ((*n)->pos.x + radius);		
		if(dx_l > 0){(*n)->ApplyDampedForce(V2(dx_l * (*n)->k, 0));}
		if(dx_r < 0){(*n)->ApplyDampedForce(V2(dx_r * (*n)->k, 0));}

		// test top and bottom model edges
		float dy_b = (-height / 2) - ((*n)->pos.y - radius);
		float dy_t = (height / 2) - ((*n)->pos.y + radius);		
		if(dy_b > 0){(*n)->ApplyDampedForce(V2(0, dy_b * (*n)->k));}
		if(dy_t < 0){(*n)->ApplyDampedForce(V2(0, dy_t * (*n)->k));}
		

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
