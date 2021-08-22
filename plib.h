#include "math.h"
#include <vector>
#include <string>
#include <map>
	
const float gravity = -9.81;

class Debugger{
public:
	virtual void print(const char* s) = 0;
	virtual void print(float f) = 0;
	virtual void print(int i) = 0;
	virtual int ticks() = 0;
};

class Debuggable{
public:	
	static Debugger* dbg;
	static int		counter;
};


struct V2 : public Debuggable{
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


class Node : public Debuggable{
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

class Bar : public Debuggable{
public:
	float		Force();	// calculate and apply bar force
	float		Yield_T(){return (n0->mat->yield_t + n1->mat->yield_t) / 2;}
	float		Yield_C(){return (n0->mat->yield_c + n1->mat->yield_c) / 2;}
	float		Ult_T(){return (n0->mat->ult_t + n1->mat->ult_t) / 2;}
	float		Ult_C(){return (n0->mat->ult_c + n1->mat->ult_c) / 2;}	
	V2			Dir();	
	
	Node* 		n0;
	Node* 		n1;	
	float 		l;			// length at 0 stress
	float 		f;			// current axial force in bar
	int			tag;
};

class NodeFunct{
public:
	virtual void operator()(Node* n);
	virtual void Init(){;}
};

class BarFunct{
public:
	virtual void operator()(Bar* n);
	virtual void Init(){;}
};

class Mesh : public Debuggable{
public:
	Node* 		AddNode(float x, float y, Material* m, int tag = 0);
	Node* 		AddNode(float x, float y, float vx, float vy, Material* m, int tag = 0);
	Bar*		AddBar(Node* n0, Node* n1, int tag = 0);
	void 		Apply(NodeFunct& f);
	void 		Apply(BarFunct& f);
	Node*		GetNodeIdx(int i){return nodes.at(i);} //??
//protected:	
	std::vector<Node*> 					nodes;
	std::vector<Bar*> 					bars;
}; 


class Model : public Mesh{
public:
	Model(Debugger* d); 
	~Model();

	void		InitModel(float w, float h, float r);

	void		AddMeshToModel(Mesh* mesh, const char* name);
	Mesh*		AddMeshToSim(const char* name);
	Mesh*		RemoveMeshFromSim(const char* name);

	Material*	AddMaterial(const char* N, float D, float E, float B, float F, float YT, float YC, float UT, float UC);
	Material*	GetMaterial(const char* name = NULL);	

	void 		AddFunctToTagMap(int t, NodeFunct* f);
	void		AddFunctToTagMap(int t, BarFunct* f);
	void		BatchNodesByTag();
	void		BatchBarsByTag();

	void		Step(float t);
	void		Collisions();
	int			Frame(int iters);
	
	int			SizeNodes(){return nodes.size();}//???
	
	
	

private:

	struct SortNodes_X{
    	bool operator()(Node* n0, Node* n1) 
    		const { return n0->pos.x < n1->pos.x; }
	};

	struct SortNodes_Tag{
    	bool operator()(Node* n0, Node* n1) 
    		const { return n0->tag < n1->tag; }
	};

	struct SortBars_Tag{
    	bool operator()(Bar* b0, Bar* b1) 
    		const { return b0->tag < b1->tag; }
	};

	float 								width;
	float 								height;
	float 								radius;
	float 								fluid_damping = 15;
	
	std::map<std::string, Mesh*>		meshes;
	std::map<std::string, Material*>	materials;
	std::map<int, NodeFunct*>			nodeTagMap;
	std::map<int, BarFunct*>			barTagMap;

};





