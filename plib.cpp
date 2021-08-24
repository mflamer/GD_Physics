#include "plib.h"
#include <algorithm>
#include <stdio.h>


Debugger* Debuggable::dbg = 0;
int Debuggable::counter = 0;

float V2::Distance(V2& v)
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

void Node::ApplyForce(float fx, float fy){
	if(force.x != 0xFFFFFFFF) force.x += fx;
	if(force.y != 0xFFFFFFFF) force.y += fy;
}	

void Node::ApplyDampedForce(const V2& f){
	float dir = vel.Dot(f);
	float cor = dir > 0 ? mat->damping : 1; 
	ApplyForce(cor * f.x, cor * f.y);
}

V2	Bar::Dir(){
	V2 dir;
	dir.x = n1->pos.x - n0->pos.x;
	dir.y = n1->pos.y - n0->pos.y;
	return dir.Unit(); 
}


// +F = tension, -F = compression
float Bar::Force(){
	// bar force
	float k = ((n0->mat->spring + n1->mat->spring) * 0.5) / l; 
	float l_ = n0->pos.Distance(n1->pos); // current length
	float def = l_ - l; // current deflection
	f = def * k;
	if(f > Yield_T()){
		float plastic_def = def * ((Yield_T() / f));
		l += plastic_def;
		f = Yield_T() + ((f - Yield_T()) * 0.05); // global 5% strain hardening
	}
	else if(f < Yield_C()){
		float plastic_def = def * ((Yield_C() / f));
		l += plastic_def;
		f = Yield_C() + ((f - Yield_C()) * 0.05); // global 5% strain hardening
	}

	// bar relative velocity for damping
	V2 rel_v;	
	rel_v.x = n1->vel.x - n0->vel.x;
	rel_v.y = n1->vel.y - n0->vel.y;

	// does the relative velocity match the direction of force?	 
	float dir = rel_v.Dot(Dir());	
	
	// coefficent of restitution
	float cor = ((dir < 0) && (def > 0)) || ((dir > 0) & (def < 0)) ? (n0->mat->damping + n1->mat->damping) * 0.5 : 1; 
	f *= cor;
	float fx = f * Dir().x;
	float fy = f * Dir().y;
	n0->ApplyForce(fx, fy);
	n1->ApplyForce(-fx, -fy);
	return f;
}


Node* Mesh::AddNode(float x, float y, Material* m, int t){
	Node* n = new Node();
	n->pos.x = x; n->pos.y = y; n->mat = m; n->tag = t;	 
	nodes.push_back(n);
	return n;
}

Node* Mesh::AddNode(float x, float y, float vx, float vy, Material* m, int tag){
	Node* n = AddNode(x, y, m, tag);
	n->vel.x = vx;
	n->vel.y = vy;
	return n;
}

// this function does not check if the node is referenced by any bars
void Mesh::RemoveNode(Node* n){
    std::vector<Node*>::iterator found = std::find(nodes.begin(), nodes.end(), n);
    if(found != nodes.end()){
        nodes.erase(found);
    } 
}


Bar* Mesh::AddBar(Node* n0, Node* n1, int tag){
	Bar* b = new Bar();
	b->n0 = n0;
	b->n1 = n1;
	b->tag = tag;
	b->l = n0->pos.Distance(n1->pos);
	bars.push_back(b);
	return b;
}

Model::Model(Debugger* d){	
	dbg = d;

}

void Model::InitModel(float w, float h, float r){
	width = w;
	height = h;
	radius = r;

    // D  = density in Kg/m3 
    // E  = modulus of elasticity in Pa/m2 
    // B  = coefficent of restitution  
    // F  = friction coefficent
    // YT = yield strength in tension
    // YC = yield strength in compression
    // UT = ultimate strength in tension
    // UC = ultimate strength in compression
	
    // Setup default materials    D           E        B    F         YT          YC           UT           UC 
    AddMaterial("rubber",        950,      50000000,  .8,   0,    12000000,   -12000000,    16000000,   -16000000);
    AddMaterial("concrete",     2300,   25000000000,   0,   0,    16000000,   -10000000,    16000000,   -14000000);
    AddMaterial("steel",        7840,  200000000000,   0,   0,   250000000,  -250000000,   400000000,  -400000000);
    AddMaterial("wood",          450,   10000000000,   0,   0,    10000000,   -10000000,    11000000,   -11000000);
    AddMaterial("_rubber",       950,       5000000,  .8,   0,     1200000,    -1200000,     1600000,    -1600000);
    AddMaterial("_concrete",    2300,    2500000000,   0,   0,     3000000,    -5000000,     3800000,    -5000000);
    AddMaterial("_steel",       7840,   10000000000,   0,   0,    25000000,   -25000000,    40000000,   -40000000);
    AddMaterial("_wood",         450,    1000000000,   0,   0,     1000000,    -1000000,     1100000,    -1100000);




}

Model::~Model(){
	std::vector<Node*>::iterator itr;
	for(itr = nodes.begin(); itr != nodes.end(); itr++){
		delete *itr;
	}
	std::vector<Bar*>::iterator bitr;
	for(bitr = bars.begin(); bitr != bars.end(); bitr++){
		delete *bitr;
	}
	std::map<std::string, Material*>::iterator mitr;
	for(mitr = materials.begin(); mitr != materials.end(); mitr++){
		delete std::get<1>(*mitr);
	}
}	

void Model::Step(float t){
	// apply forces from bars
	std::vector<Bar*>::iterator b_itr;
	for(b_itr = bars.begin(); b_itr != bars.end(); b_itr++){
		float bf = (*b_itr)->Force();
		//test for bar yield and remove if so
		if((bf > (*b_itr)->Ult_T()) | (bf < (*b_itr)->Ult_C())) {
            if(barDestructEvent) (*barDestructEvent)(*b_itr);
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
	sort(nodes.begin(), nodes.end(), SortNodes_X());
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
            if((*m)->pos.y - (*n)->pos.y < (radius * 2)){
    			float d = (*m)->pos.Distance((*n)->pos);
    			if(d < radius * 2){
    				float k = (((*m)->mat->spring + (*n)->mat->spring) * 0.5) / (2 * radius);
    				float f = ((2 * radius) - d) * k;
    				V2 collision_f;  
    				collision_f.x = f * (((*m)->pos.x - (*n)->pos.x) / d);
    				collision_f.y = f * (((*m)->pos.y - (*n)->pos.y) / d); 

    				(*m)->ApplyDampedForce(collision_f);
    				(*n)->ApplyDampedForce(-collision_f);
                    if(NodeCollisionEvent){
                        (*NodeCollisionEvent)(*n);
                        (*NodeCollisionEvent)(*m);
                    } 
    			}
            }
		}		

		// test left and right model edges
		float k = (*n)->mat->spring / radius;
		float dx_l = (-width / 2) - ((*n)->pos.x - radius);
		float dx_r = (width / 2) - ((*n)->pos.x + radius);		
		if(dx_l > 0){(*n)->ApplyDampedForce(V2(dx_l * k, 0));}
		if(dx_r < 0){(*n)->ApplyDampedForce(V2(dx_r * k, 0));}

		// test top and bottom model edges
		float dy_b = (-height / 2) - ((*n)->pos.y - radius);
		float dy_t = (height / 2) - ((*n)->pos.y + radius);		
		if(dy_b > 0){(*n)->ApplyDampedForce(V2(0, dy_b * k));}
		if(dy_t < 0){(*n)->ApplyDampedForce(V2(0, dy_t * k));}
		

	}

}

int Model::Frame(int iters){
	int t_collide;
    int t_force;
    int fps = dbg->ticks();
    for(int i = 0; i < iters; i++)
    {
      t_collide = dbg->ticks();    
      Collisions();
      t_collide = dbg->ticks() - t_collide;
      t_force = dbg->ticks();
      Step(1.0/(60.0 * iters));  
      t_force = dbg->ticks() - t_force; 
    }
    
    fps = 1000000 / (dbg->ticks() - fps);
    dbg->print("t_collide = "); dbg->print(t_collide); dbg->print("\n");
    dbg->print("t_force = "); dbg->print(t_force); dbg->print("\n");
    //dbg->print("frames/s = "); dbg->print(fps); dbg->print("\n\n");
    return fps;
}


Material* Model::AddMaterial(const char* N, float D, float E, float B, float F, float YT, float YC, float UT, float UC){
	Material* m = new Material();
	float A = 3.14159265359 * pow(radius, 2);
	float V = (4/3) * 3.14159265359 * pow(radius, 3);
	m->mass = D * V;
	m->spring = E * A;
	m->damping = B;
	m->friction = F;
	m->yield_c = A * YC;
	m->yield_t = A * YT;
	m->ult_c = A * UC;
	m->ult_t = A * UT;
	materials.insert({N, m});
	dbg->print(N); dbg->print(" = "); dbg->print((int)materials[N]); dbg->print("\n");
	return m;
}

Material* Model::GetMaterial(const char* name){
	Material* m = materials[name];
	//dbg->print("Got material "); dbg->print(name); dbg->print(" = "); dbg->print((int)m); dbg->print("\n");
	return m;
}

void Model::AddMeshToModel(Mesh* mesh, const char* name){
	meshes[name] = mesh;
	dbg->print(name); dbg->print(" = "); dbg->print((int)meshes[name]); dbg->print("\n");
}

Mesh* Model::AddMeshToSim(const char* name){
	Mesh* mesh = meshes[name];    
	if(mesh){
		nodes.insert(nodes.end(), mesh->nodes.begin(), mesh->nodes.end());
		bars.insert(bars.end(), mesh->bars.begin(), mesh->bars.end());
		dbg->print(name); dbg->print(" added to sim, model nodes cnt = "); dbg->print((int)nodes.size()); dbg->print("\n");
		return mesh;
	}
	return NULL;
}

Mesh* Model::RemoveMeshFromSim(const char* name){
    // need to implement
    return NULL;
}


void Mesh::Apply(NodeFunct& f){
	std::vector<Node*>::iterator n;
	for(n = nodes.begin(); n != nodes.end(); n++){
		f(*n);
	}
}

void Mesh::Apply(BarFunct& f){
	std::vector<Bar*>::iterator b;
	for(b = bars.begin(); b != bars.end(); b++){
		f(*b);
	}
}

void Model::AddFunctToTagMap(int t, NodeFunct* f){
	nodeTagMap[t] = f; 
}

void Model::AddFunctToTagMap(int t, BarFunct* f){
	barTagMap[t] = f; 
}

void Model::BatchNodesByTag(){
	sort(nodes.begin(), nodes.end(), SortNodes_Tag());
	std::vector<Node*>::iterator n;
	int current_tag = 0xFFFFFFFF;
	NodeFunct* f = NULL;
	for(n = nodes.begin(); n != nodes.end(); n++){
		if((*n)->tag != current_tag){
			current_tag = (*n)->tag;
			f = nodeTagMap[current_tag];
			if(f==NULL)dbg->print("Node funct = 0 in tag map. crash! \n");
			f->Init();
		}
		(*f)(*n);
	}
}

void Model::BatchBarsByTag(){
	sort(bars.begin(), bars.end(), SortBars_Tag());
	std::vector<Bar*>::iterator b;
	int current_tag = 0xFFFFFFFF;
	BarFunct* f = NULL;	
	for(b = bars.begin(); b != bars.end(); b++){
		if((*b)->tag != current_tag){
			current_tag = (*b)->tag;
			f = barTagMap[current_tag];
			if(f==NULL)dbg->print("Bar funct = 0 in tag map. crash! \n");
			f->Init();
		}			
		(*f)(*b);
	}
}

