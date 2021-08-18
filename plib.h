#include "math.h"
#include <vector>
#include <string>
#include <map>
	
const float gravity = -9.81;
//float scale = 64;//  pix / m 



class Printer{
public:
	virtual void operator()(char* s){}
	virtual void operator()(float f){}
};

class Debug{
public:	
	static Printer* print;
};


struct V2 : public Debug{
			V2(){;}
			V2(float _x, float _y){x = _x; y = _y;}
	float 	Distance(V2& v);
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
	float		friction;	
	float 		yield_t;	// (+) tensile yield strength
	float 		yield_c;	// (-) compressive yield strength	
	float		ult_t;
	float		ult_c;	
};


class Node : public Debug{
public:	
	void		ApplyForce(float fx, float fy);
	void		ApplyDampedForce(const V2& f);
	void		Fix_X(){force.x = 0xFFFFFFFF;}
	void		Fix_Y(){force.y = 0xFFFFFFFF;}
	void		Fix_XY(){force.x = 0xFFFFFFFF; force.y = 0xFFFFFFFF;}

	V2 			pos;	
	V2 			vel;
	V2			force;
	Material* 	mat;
	int			tag;
};

class NodeFunct{
public:
	virtual void operator()(Node* n) = 0;
};


class Bar : public Debug{
public:
	float		Force();	// calculate and apply bar force
	float		Yield_T(){return (n0->mat->yield_t + n1->mat->yield_t) / 2;}
	float		Yield_C(){return (n0->mat->yield_c + n1->mat->yield_c) / 2;}	
	V2			Dir();	
	
	Node* 		n0;
	Node* 		n1;	
	float 		l;			// length at 0 stress
	float 		f;			// current axial force in bar
	int			tag;
};

class BarFunct{
public:
	virtual void operator()(Bar* n) = 0;
};


class Model : public Debug{
public:
	Model(Printer* p); 
	~Model();

	void		SetModel(float w, float h, float r, float s);


	Material*	AddMaterial(const char* N, float D, float E, float B, float F, float YT, float YC, float UT, float UC);
	Node* 		AddNode(float x, float y, Material* m, int tag = 0);
	Node* 		AddNode(float x, float y, float vx, float vy, Material* m, int tag = 0);
	Bar*		AddBar(Node* n0, Node* n1);
	void		Step(float t);
	void		Collisions();
	
	int			SizeNodes(){return nodes.size();}
	Node*		GetNodeIdx(int i){return nodes.at(i);}
	Material*	GetMaterial(const char* name = NULL);
	
	void 		MapNodes(NodeFunct* f);
	void 		MapBars(BarFunct* f);

private:

	struct NodeLess{
    	bool operator()(Node* n0, Node* n1) 
    		const { return n0->pos.x < n1->pos.x; }
	};

	float 								width;
	float 								height;
	float 								radius;
	float								scale;
	float 								fluid_damping = 15;
	
	std::vector<Node*> 					nodes;
	std::vector<Bar*> 					bars;
	std::map<std::string, Material*>	materials;

};





