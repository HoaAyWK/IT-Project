#include <string>
using namespace std;

const int NONE = numeric_limits<int>::max() / 2;

template<class T>
class Edge {
private:
    string name;
    string src;
    string dest;
    int weight;
public:
    Edge();
    Edge(const string& name, const string& src, const string& dest) {
        this->name = name;
        this->src = src;
        this->dest = dest;
        this->weight = NONE;
    }
    Edge(const string& name, const string& src, const string& dest, int weight) {
        this->name = name;
        this->src = src;
        this->dest = dest;
        this->weight = weight;
    }
    void setName(const string& name) { this->name = name; }
    string getName() const { return name; }
    string getSrcVertex() const { return src; }
    void setSrcVertex(const string& src) { this->src = src; }
    string getDestVertex() const { return dest; }
    void setDestVertex(const string& dest) { this->dest = dest; }
    int getWeight() { return weight; }
    void setWeight(int weight) { this->weight = weight; }
};

