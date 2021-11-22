
#include "LNode.h"

template<class T>
class LVertex {
private:
    T name;
public:
    LNode<T>* node;
    LVertex(const T& name) {
        this->name = name;
        node = NULL;
    }
    T getVertexName() const;
    void setVertexName(const T&);
};


template<class T>
void LVertex<T>::setVertexName(const T& name) {
    this->name = name;
}

template<class T>
T LVertex<T>::getVertexName() const {
    return name;
}