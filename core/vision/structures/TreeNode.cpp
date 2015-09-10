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

bool TreeNode::hasOverlap(struct TreeNode* node) {
  return (!(node->line->posStart > line->posEnd || node->line->posEnd < line->posStart));
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
  if (parent1 != parent2) {
    parent2->actAsParent(parent1);
    rootSet.erase(parent1);
  }
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
    next = temp2;
  }
}

void TreeNode::actAsParent(struct TreeNode *node){
  
  node->parent = this;
  if(node->topleft->x < topleft->x)
      topleft->x = node->topleft->x;
  if(node->bottomright->x > bottomright->x)
      bottomright->x = node->bottomright->x;
  if(node->topleft->y < topleft->y)
      topleft->y = node->topleft->y;
  if(node->bottomright->y > bottomright->y)
      bottomright->y = node->bottomright->y;
}


