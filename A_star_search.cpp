#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <math.h>
#include <list>
#include <set>


using namespace std;

class Priority_queue{
    private:
    vector<pair<int,double>> P_queue;

    static bool cmp(const pair<int,double>& a,const pair<int,double>& b){
        return a.second < b.second;
    }

    public:
    void insert(pair<int,double> data){
        auto pos = lower_bound(P_queue.begin(),P_queue.end(),data,cmp);
        P_queue.insert(pos,data);
    }
    pair<int,double> front(){
        return P_queue.front();
    }
    pair<int,double> back(){
        return P_queue.back();
    }
    void remove(){
        P_queue.erase(P_queue.begin());
    }
    void Print(){
        for(auto i : P_queue){
            cout << "<-" << "("<< i.first <<","<<i.second << ")";
        }
    }
    bool Empty(){
        return P_queue.empty();
    }
};


class Graph{
private:
map<int,list<pair<int,double>>> graph;
map<int,pair<double,double>> directional_map;

bool Isdirected = true;
int number_of_vertex = 0;

double Compute_heuristic_distance(double x1,double x2, double y1, double y2){
    return sqrt(pow(x2-x1,2)+pow(y2 - y1,2));
}

public :
    void AddVertex(int value,double x,double y){
        if(graph.find(value) == graph.end()){
            graph[value] = {};
            directional_map[value] = {x,y};
            number_of_vertex+=1;
        }
        else{
            cout <<"Edge Already Exists";
        }
    }

    
    void AddEdge(int v1,int v2,double weight){
        if(graph.find(v1) == graph.end() || graph.find(v2) == graph.end()){
            cout<< "NO Edge Found\n";
            return;
        }
        graph[v1].push_back({v2,weight});
    }

    void print(){
        for(auto vertex : graph){
            cout << vertex.first << "->\t" ;
            for(auto edge :vertex.second){
                cout << "{" <<edge.first <<","<<edge.second <<"}->";
            }
            cout << "NULL\n";
        }
    }

    void Dijkstras_algorithm(int start,int end){
        if(graph.find(end) == graph.end()||graph.find(start) == graph.end()){
            cout<<"END Does not exsist";
            return;
        }
        int Steps = 0;

        Priority_queue queue;
        map<int,double> dist;
        set<int> visited;
        
        for(auto i:graph){
            dist[i.first] = INT_MAX;
        }
        
        pair<int,int> s = {start,0};
        queue.insert(s);
        

        while(!queue.Empty()){
            int edge = queue.front().first;
            int w = queue.front().second;

            if(visited.count(edge)){ continue;}
            visited.insert(edge);
            Steps+=1;

            queue.remove();

            if(edge == end){
                cout<< "\nDistance -> "<<dist[edge]<<"\tSteps-> "<<Steps<<"\n";
                return;
            }
            for(auto i : graph[edge]){
                if(w + i.second < dist[i.first]){
                    dist[i.first] =  w + i.second;
                    queue.insert({i.first,dist[i.first]});
                }
            }

        }
    }



    void A_star(int start,int end){
        Priority_queue queue;
        map<int,double> dist;
        set<int> visited;


        for(auto i:graph){
            dist[i.first] = INT_MAX;
        }
        double x_end = directional_map[end].first;
        double y_end = directional_map[end].second;

        double h_start = Compute_heuristic_distance(
            directional_map[start].first,x_end,
            directional_map[start].second,y_end
        );


        dist[start] = 0;
        queue.insert({start,h_start});

        int steps = 0;


        while(!queue.Empty()){
        int vertex = queue.front().first;
        int distance = queue.front().second;
        if(visited.count(vertex)){ continue;}
        visited.insert(vertex);
        steps+=1;

        queue.remove();
        if(vertex == end){
            cout <<"Distance-->" << (double)dist[vertex] << "\t Steps-->" << steps;
            return;
        }

        for(auto i : graph[vertex]){            
            double x = directional_map[i.first].first;
            double y = directional_map[i.first].second;
            double heuristic = Compute_heuristic_distance(x,x_end,y,y_end);
            double total_dist = i.second + dist[vertex];
            double f_n = heuristic +total_dist;

            if(total_dist < dist[i.first]){
                dist[i.first] = total_dist;
                queue.insert({i.first,f_n});
                }
            }
        }
    }
};


/*
here we would use a map which would contain {vertex:{weight,connection}} and one more map for containing positions;
{vertex:{x,y}}


*/


/*Main Function*/
int main(){
Graph g;
    // Add vertices with coordinates (x, y)
    g.AddVertex(1, 0, 0);
    g.AddVertex(2, 1, 0);
    g.AddVertex(3, 2, 0);
    g.AddVertex(4, 0, 1);
    g.AddVertex(5, 1, 1);
    g.AddVertex(6, 2, 1);
    
    g.AddEdge(1, 2, 1.0);
    g.AddEdge(1, 4, 1.0);
    g.AddEdge(2, 3, 1.0);
    g.AddEdge(2, 5, 1.0);
    g.AddEdge(3, 6, 1.0);
    g.AddEdge(4, 5, 1.0);
    g.AddEdge(5, 6, 1.0);

    g.Dijkstras_algorithm(1, 6);
    g.A_star(1, 6);

}
