#include <vision/structures/TreeNode.h>

TreeNode::TreeNode(int row, int start, int end, int colour){
  
  line = new LineRun(start, end, row);
  topleft = new Coordinates(start,row);
  bottomright = new Coordinates(end,row);
  color = colour;
  numberOfPixels = end - start + 1;
  parent = 0;
  
}

bool TreeNode::hasSimilarWidth(struct TreeNode *node, float error){
  return (abs((bottomright->x - topleft->x) - (node->bottomright->x - node->topleft->x)) <= error);
}
bool TreeNode::hasSimilarHeight(struct TreeNode *node, float error){
  return (abs((bottomright->y - topleft->y) - (node->bottomright->y - node->topleft->y)) <= error);
}

bool TreeNode::hasExpectedHeight(int expectedHeight, float error){
  //std::cout<<"Expecting "<<bottomright->y - topleft->y<< " "<< expectedHeight<<"\n";
  return (abs((bottomright->y - topleft->y) - expectedHeight) <= error );
}

bool TreeNode::isStackedAbove(struct TreeNode *node, float errorX, float errorY){
  
  if ((bottomright->x <= node->topleft->x) || (node->bottomright->x <= topleft->x)) {
    return false;
  }
  return (!((abs(topleft->x - node->topleft->x) > errorX) || (abs(bottomright->x - node->bottomright->x) > errorX) ||
    (abs(bottomright->y - node->topleft->y) > errorY)));
}

bool TreeNode::isContainedWithin(struct TreeNode* node, float errorX) {
  return (topleft->x + errorX > node->topleft->x) && (topleft->y > node->topleft->y) && (bottomright->x < node->bottomright->x + errorX) && (bottomright->y < node->bottomright->y);
}

bool TreeNode::hasSimilarDensity(struct TreeNode *node){
  float ratio = (float) node->numberOfPixels / (float) numberOfPixels;
  if (ratio > 1){
    ratio = 1 / ratio;
  }
  return (ratio>=0.75);
}

bool TreeNode::hasMinimumWidth(int minValue){
  return (bottomright->x - topleft->x >= minValue);
}

bool TreeNode::hasMinimumHeight(int minValue){
  return (bottomright->y - topleft->y >= minValue);
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

void DisjointSet::clear() {
  for(std::set<struct TreeNode *>::iterator node = rootSet.begin(); node != rootSet.end(); ++node){
      delete (*node);
  }  
  rootSet.clear();
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
  bool changed = false;
  if(node->topleft->x < topleft->x) {
      topleft->x = node->topleft->x;
      changed = true;
  }
  if(node->bottomright->x > bottomright->x) {
      bottomright->x = node->bottomright->x;
      changed = true;
  }
  if(node->topleft->y < topleft->y) {
      topleft->y = node->topleft->y;
      changed = true;
  }  
  if(node->bottomright->y > bottomright->y) {
      bottomright->y = node->bottomright->y;
      changed = true;
  }
  if (changed) {
    numberOfPixels += node->numberOfPixels; 
  }
}

void DisjointSet::deleteTinyBlobs(){
  int minValue = 3;
  for(std::set<struct TreeNode *>::iterator node = rootSet.begin(); node != rootSet.end(); ++node){
    int height = (*node)->bottomright->y - (*node)->topleft->y;
    int width = (*node)->bottomright->x - (*node)->topleft->x;
    if (height <= minValue || width <= minValue){
       rootSet.erase(*node);
    }
  }
}

