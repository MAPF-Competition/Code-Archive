#include "../include/lnode.hpp"

int LACAM_PREFIX_LNode::COUNT = 0;

LACAM_PREFIX_LNode::LACAM_PREFIX_LNode() : who(), where(), depth(0) { ++COUNT; }

LACAM_PREFIX_LNode::LACAM_PREFIX_LNode(LACAM_PREFIX_LNode *parent, int i, LACAM_PREFIX_Vertex *v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  ++COUNT;
  who.push_back(i);
  where.push_back(v);
}

LACAM_PREFIX_LNode::~LACAM_PREFIX_LNode(){};
