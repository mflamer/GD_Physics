#include "math.h"
#include <vector>

static const float radius = 5;	
static const float gravity = -9.81;
static const float air_damping = 10;
static const float elast_damping = 500;

struct V2{
	float Distance(V2& v);
	float Mag();
	float Dot(V2& v);
	V2	  Unit();

	float x;
	float y;
};


struct Material{
	
	float density;			// density
	float elastic;			// modulus of elasticity
	float damping;	
};


class Node{
public:
	Node(float x, float y, Material* m);

	V2 		pos;	
	V2 		vel;
	V2		force;
	float 	k;
	float	m;
	
};

class Bar{
public:
	float	Force(V2& v);
	float	DampingForce();		
	
	Node* n0;
	Node* n1;
	float l;			// length at 0 stress
	float k;			// spring constant
	float f;			// current axial force in bar
};


class NodeFunctor{
public:
	virtual void operator()(Node* n) = 0;
};

class BarFunctor{
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

	void	SetModel(float w, float h);

	Node* 	AddNode(float x, float y, Material* m);
	Bar*	AddBar(Node* n0, Node* n1, float k);
	void	Step(float t);
	void	Collisions();


	void 	MapNodes(NodeFunctor* f);
	void 	MapBars(BarFunctor* f);

private:

	struct NodeLess{
    	bool operator()(Node* n0, Node* n1) 
    		const { return n0->pos.x < n1->pos.x; }
	};

	float width;
	float height;
	Debuger* printer;
	std::vector<Node*> 	nodes;
	std::vector<Bar*> 	bars;

};





