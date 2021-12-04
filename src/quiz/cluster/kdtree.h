/* \author Aaron Brown */
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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertNode(Node *&node, std::vector<float> point, int id, int level)
	{
		if(node == NULL)
		{
			node = new Node(point, id);
		}
		else if(point[level % 2] < node->point[level % 2])
		{
			insertNode(node->left, point, id, level+1);
		}
		else
		{
			insertNode(node->right, point, id, level+1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNode(root, point, id, 0);
	}

	void searchNode(Node *node, std::vector<float> target, float distanceTol, std::vector<int> &ids, int level)
	{
		if(node != NULL)
		{
			std::vector<float> distance;
			distance.push_back(target[0] - node->point[0]);
			distance.push_back(target[1] - node->point[1]);

			if((fabs(distance[0]) <= distanceTol) && (fabs(distance[1]) <= distanceTol))
			{
				float eucDistance = sqrt(pow(distance[0], 2) + pow(distance[1], 2));
				if(eucDistance < distanceTol)
					ids.push_back(node->id);
			}

			if(distance[level % 2] <= distanceTol)
			{
				searchNode(node->left, target, distanceTol, ids, level+1);
			}
			if(distance[level % 2] > -distanceTol)
			{
				searchNode(node->right, target, distanceTol, ids, level+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchNode(root, target, distanceTol, ids, 0);

		return ids;
	}
	

};




