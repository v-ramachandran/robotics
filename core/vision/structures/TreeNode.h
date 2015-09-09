#ifndef TREE_NODE_H
#define TREE_NODE_H

#include<set>
#include<vector>
#include <inttypes.h>
#include <vision/structures/LineRun.h>
#include <vision/structures/Coordinates.h>

struct TreeNode {
  struct LineRun *line;
  int color;
  struct Coordinates *topleft;
  struct Coordinates *bottomright;
  struct TreeNode *parent;
  TreeNode(int row, int start, int end, int color);
  bool hasSelfAsParent();
  struct TreeNode* findGlobalParent();
  void actAsParent(struct TreeNode *node);
};

struct DisjointSet{
  std::set<struct TreeNode *> rootSet;
  struct TreeNode * makeset(int row, int start, int end, int color);
  void unionNodes( struct TreeNode * node1, struct TreeNode * node2);
  void find(struct TreeNode * node);
  void mergeNodes(struct TreeNode * node1, struct TreeNode * node2);
};
 
#endif


