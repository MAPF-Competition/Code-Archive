#ifndef DIJKSTRANODE_H
#define DIJKSTRANODE_H

class DijkstraNode {
public:
    int location;
    int parentLocation;
    int g;
    DijkstraNode* parent;

    DijkstraNode(int _location, int _parentLoc, int _g, DijkstraNode* _parent = nullptr): 
        location(_location), parentLocation(_parentLoc), g(_g), parent(_parent) { }

    struct cmp {
        bool operator() (const DijkstraNode* const a, const DijkstraNode* const b) const {
            return a->g > b->g;
        }
    };
};

#endif // DIJKSTRANODE_H