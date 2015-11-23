#ifndef TREE_NODE_H
#define TREE_NODE_H

#include<iostream>
#include<set>
#include<vector>
#include <inttypes.h>
#include <vision/structures/LineRun.h>
#include <vision/structures/Coordinates.h>

struct TreeNode {
  struct LineRun *line;
  int color;
  int numberOfPixels;
  struct Coordinates *topleft;
  struct Coordinates *bottomright;
  struct TreeNode *parent;
  TreeNode(int row, int start, int end, int color);
  ~TreeNode(){
    delete line;
    delete topleft;
    delete bottomright;
  }
  bool hasSelfAsParent();
  bool hasOverlap(struct TreeNode *node);
  struct TreeNode* findGlobalParent();
  void actAsParent(struct TreeNode *node);
  bool hasSimilarWidth(struct TreeNode *node, float error);
  bool hasSimilarHeight(struct TreeNode *node, float error);
  bool hasExpectedHeight(int expectedHeight, float error);
  bool isContainedWithin(struct TreeNode* node, float errorX);
  bool isStackedAbove(struct TreeNode *node, float errorX, float errorY);
  bool hasMinimumWidth(int minValue);
  bool hasMinimumHeight(int minValue);
  bool hasSimilarDensity(struct TreeNode *node);
};

struct DisjointSet{
  void clear();
  std::set<struct TreeNode *> rootSet;
  struct TreeNode * makeset(int row, int start, int end, int color);
  void unionNodes( struct TreeNode * node1, struct TreeNode * node2);
  void find(struct TreeNode * node);
  void mergeNodes(struct TreeNode * node1, struct TreeNode * node2);
  void deleteTinyBlobs();
};
 
#endif
