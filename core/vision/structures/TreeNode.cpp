#include <vision/structures/TreeNode.h>

TreeNode::TreeNode(int row, int start, int end, int colour){
  
  line = new LineRun(start, end, row);
  topleft = new Coordinates(start,row);
  bottomright = new Coordinates(end,row);
  color = colour;
  parent = 0;
  
}

bool TreeNode::hasSelfAsParent(){
  
  return (parent == this); 

}

struct TreeNode* TreeNode::findGlobalParent(){
  
  struct TreeNode *temp = this;
  while(temp->parent != temp){
    temp = temp->parent;        
  }
  return temp;
}

void DisjointSet::mergeNodes(struct TreeNode * node1, struct TreeNode * node2){
  struct TreeNode *parent1 = node1->findGlobalParent();
  struct TreeNode *parent2 = node2->findGlobalParent();
  parent2->actAsParent(parent1);
  rootSet.erase(parent1);
}

struct TreeNode * DisjointSet::makeset(int row, int start, int end, int color){
  
  struct TreeNode *temp = new TreeNode(row,start,end,color);
  temp->parent = temp; 
  rootSet.insert(temp); 
  return temp;
}

void DisjointSet::unionNodes(struct TreeNode *node1, struct TreeNode *node2){
  
  node1->actAsParent(node2);
  rootSet.erase(node2);
}

void DisjointSet::find(struct TreeNode * node){

  struct TreeNode *temp = node;
  while(temp->parent != temp){
    temp = temp->parent;        
  }
  
  struct TreeNode *next = node;
  while(next->parent != next){
    struct TreeNode *temp2 = next->parent;
    temp->actAsParent(next);
    rootSet.erase(next);
    next = temp2;
  }
}

void TreeNode::actAsParent(struct TreeNode *node){

  node->parent = this;
  if(node->line->posStart < topleft->x)
      topleft->x = node->line->posStart;
  if(node->line->posEnd > bottomright->x)
      bottomright->x = node->line->posEnd;
  bottomright->y = node->line->rowNum;  
}


