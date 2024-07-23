/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"

#ifndef KDTREE_H
#define KDTREE_H

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

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root,0,point,id);
	}

	void insertHelper(Node** node, unsigned int depth, std::vector<float> point, int id){

		if (*node==NULL){
			*node = new Node(point,id);
		}
		else{
			unsigned int cd = depth % 3;

			if (point[cd] < ((*node)->point[cd])){
				insertHelper(&((*node)->left),depth+1,point,id);
			}
			else{
				insertHelper(&((*node)->right),depth+1,point,id);
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root,ids,0,target,distanceTol);

		return ids;
	}

	void searchHelper(Node* node, std::vector<int>& ids, unsigned int depth, std::vector<float> target, float distanceTol){
		
		if(node!=NULL){
			if ((target[0] + distanceTol >= node->point[0]) && (target[0] - distanceTol <= node->point[0]) 
				&& (target[1] + distanceTol >= node->point[1]) && (target[1] - distanceTol <= node->point[1]) 
				&& (target[2] + distanceTol >= node->point[2]) && (target[2] - distanceTol <= node->point[2])){
				float distance = sqrt(pow(node->point[0]-target[0],2)+pow(node->point[1]-target[1],2)+pow(node->point[2]-target[2],2));
				if (distance <= distanceTol){
					ids.push_back(node->id);
				}
			}


			unsigned int currentDepth = depth % 3;
			if (target[currentDepth] - distanceTol < node->point[currentDepth]){
				searchHelper(node->left,ids,depth+1,target,distanceTol);
			}
			if (target[currentDepth] + distanceTol > node->point[currentDepth]){
				searchHelper(node->right,ids,depth+1,target,distanceTol);
				
			}
		}
	}
	

};

#endif	// KDTREE_H




