#include "math.h"
#include <vector>

//static const float radius = 5;	
static const float gravity = -9.81;
//static const float elast_damping = 500;

struct V2{
			V2(){;}
			V2(float _x, float _y){x = _x; y = _y;}
	float 	Distance(const V2& v);
	float 	Mag();
	float 	Dot(const V2& v);
	V2	  	Unit();
	V2		operator-();

	float x;
	float y;
};


struct Material{
	
	float 		mass;			
	float 		spring;			
	float 		damping;	
	float 		yield_t;	// (+) tensile yield strength
	float 		yield_c;	// (-) compressive yield strength		
};


class Node{
public:
			
	float		ApplyDampedForce(const V2& f);
	V2 			pos;	
	V2 			vel;
	V2			force;
	Material* 	mat;
	
};

class Bar{
public:
	float		Force(V2& v);
	float		DampingForce();		
	
	Node* 		n0;
	Node* 		n1;
	Material*	mat;
	float 		l;			// length at 0 stress
	float 		f;			// current axial force in bar
};


class NodeFunct{
public:
	virtual void operator()(Node* n) = 0;
};

class BarFunct{
public:
	virtual void operator()(Bar* n) = 0;
};

class Debuger{
public:
	virtual void operator()(char* s) = 0;
	virtual void operator()(float f) = 0;
};


class Model{
public:
	Model(Debuger* d); 
	~Model();

	void	SetModel(float w, float h, float r);

	Node* 	AddNode(float x, float y, Material* m);
	Bar*	AddBar(Node* n0, Node* n1, Material* m);
	void	Step(float t);
	void	Collisions();


	void 	MapNodes(NodeFunct* f);
	void 	MapBars(BarFunct* f);

private:

	struct NodeLess{
    	bool operator()(Node* n0, Node* n1) 
    		const { return n0->pos.x < n1->pos.x; }
	};

	float 				width;
	float 				height;
	float 				radius;	
	float 				fluid_damping = 10;
	Debuger* printer;
	std::vector<Node*> 	nodes;
	std::vector<Bar*> 	bars;

};





