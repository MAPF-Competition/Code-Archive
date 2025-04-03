#ifndef RRASTARNODE_H
#define RRASTARNODE_H

class RRAstarNode {
public:
    int location;
    int direction;
    int g,h;

    RRAstarNode(int _location,int _direction, int _g, int _h): location(_location), direction(_direction), g(_g), h(_h) { }

    int f() const { return g + h; }

    struct cmp {
        bool operator() (const RRAstarNode* const a, const RRAstarNode* const b) const {
            if(a->f() == b->f()) return a->g <= b->g;
            else return a->f() > b->f();
        }
    };
};

#endif // RRASTARNODE_H