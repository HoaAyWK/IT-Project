
#include <string>

using namespace std;

template<class T>
class LEdge {
private:
    string name;
    T source, destination;
    int weight;
public:
    LEdge(const string& name, const T& source, const T& destination, int weight) {
        this->name = name;
        this->source = source;
        this->destination = destination;
        this->weight = weight;
    }
    T getSource() const;
    void setSource(const T&);
    T getDestination() const;
    void setDestination(const T&);
    int getWeight() const;
    void setWeight(int);
    string getName() const;
    void setName(const string&);
};

template<class T>
T LEdge<T>::getSource() const {
    return source;
}

template<class T>
void LEdge<T>::setSource(const T& source) {
    this->source = source;
}

template<class T>
T LEdge<T>::getDestination() const {
    return destination;
}

template<class T>
void LEdge<T>::setDestination(const T& destination) {
    this->destination = destination;
}

template<class T>
int LEdge<T>::getWeight() const {
    return weight;
}

template<class T>
void LEdge<T>::setWeight(int weight) {
    this->weight = weight;
}

template<class T>
void LEdge<T>::setName(const string& name) {
    this->name = name;
}

template<class T>
string LEdge<T>::getName() const {
    return name;
}
