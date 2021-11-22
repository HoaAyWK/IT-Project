
#include <string>

using namespace std;

template<class T>
class Node {
public:
    string name;
    T value;
    Node(const T& value) {
        this->value = value;
    }
    Node(const string& name, const T& value) {
        this->name = name;
        this->value = value;
    }
    string getName() const { return name; }
    void setName(const string& name) { this->name = name; }
    T getValue() const { return value; }
    void setValue(const T& value) { this->value = value; }
};