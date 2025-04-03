//
// Created by take_ on 24-12-27.
//
#include "ECBSNode.h"


ECBSNode::ECBSNode(ECBSNode *parent): parent(parent)
{
    g_val = parent->g_val;
    h_val = 0;
    min_f_val = parent->min_f_val;
    depth = parent->depth + 1;
}

void ECBSNode::clear()
{
    conflicts.clear();
}