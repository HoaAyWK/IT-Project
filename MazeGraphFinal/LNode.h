
template<class T>
class LNode {
private:
    T value;
    int cost;
public:
    LNode<T>* next;
    LNode<T>* prev;
    LNode(const T& value, int cost, LNode<T>* next) {
        this->value = value;
        this->cost = cost;
        this->next = next;
        this->prev = NULL;
    }
    LNode(const T& value, int cost, LNode<T>* next, LNode<T>* prev) {
        this->value = value;
        this->cost = cost;
        this->next = next;
        this->prev = prev;
    }
    template<class U> friend class Graph;
    T getValue() const;
    void setValue(const T&);
    int getCost() const;
    void setCost(int);
    LNode<T>* getNext();
    LNode<T>* getPrev();
};

template<class T>
T LNode<T>::getValue() const {
    return value;
};

template<class T>
void LNode<T>::setValue(const T& value) {
    this->value = value;
}

template<class T>
int LNode<T>::getCost() const {
    return cost;
};

template<class T>
void LNode<T>::setCost(int cost) {
    this->cost = cost;
}

template<class T>
LNode<T>* LNode<T>::getNext() {
    return next;
};

template<class T>
LNode<T>* LNode<T>::getPrev() {
    return prev;
}