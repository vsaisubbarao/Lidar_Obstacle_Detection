/* \author V Sai Subba Rao, Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point; 
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, std::vector<float> point, int id, int depth)
	{
		// Create a new node and place correctly with in the root
		
		if (*node == NULL)
			*node = new Node(point, id);

		else  
		{
			uint i = depth % point.size() ;
			// Split based on depth 
			if(point[i] < (*node)->point[i])
				insertHelper(&((*node)->left), point, id, depth+1);
			else 
				insertHelper(&((*node)->right), point, id, depth+1);
		}   
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, point, id, 0);
	}

	void searchHelper(Node* node, std::vector<int> &ids, int depth, std::vector<float> target, float distanceTol)
	{
		if (node != NULL)
		{
			bool basicCheck = false;
			for (int i=0; i<target.size(); i++){
				basicCheck = (fabs(node->point[i] - target[i]) < distanceTol) ? true : false; 
			}

			if (basicCheck)
			{
				float dist = 0; 
				for (int i=0; i < target.size(); i++){
					dist += sqrt(pow(node->point[i] - target[i], 2));
				}
				if (dist < distanceTol)
					ids.push_back(node->id);
			}

			uint i = depth % target.size();		
			
			if (target[i] - distanceTol < node->point[i])
				searchHelper(node->left, ids, depth+1, target, distanceTol);
			
			if (target[i] + distanceTol > node->point[i])
				searchHelper(node->right, ids, depth+1, target, distanceTol);
			
		}

		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids; 

		searchHelper(root, ids, 0, target, distanceTol);
		
		return ids;
	}
	

};
