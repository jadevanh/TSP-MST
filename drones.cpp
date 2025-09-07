// Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0

#include <getopt.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include "xcode_redirect.hpp"
#include <algorithm>
#include <unordered_map>
#include <map>
#include <iomanip>
#include <limits>
#include <cmath>
using namespace std;

struct Coordinates {
    int x_val = -1;
    int y_val = -1;
    bool mainCampus = false;
    bool medicalCampus = false;
    bool border = false;
};


class OptimalClass {
    private:

    public:
    double bestWeight = -1; 
    double currWeight = 0;
    vector<size_t> bestPath;
    vector<size_t> path;
    vector<Coordinates> verticies;
    vector<vector<double>> distanceMatrix;
    void genPerms(size_t permLength);
    bool isPromsing(size_t permLength);
    void countDistances();
    double optMst(vector<size_t> &point, size_t numPoints);

    OptimalClass() {}
    OptimalClass(vector<Coordinates> &verticies) :verticies(verticies) {}
};


class Data {
    private:
    bool mstMode = false;
    bool fast = false;
    bool optimize = false;
    OptimalClass optimizePath;
    size_t numPoints = 0;
    vector<Coordinates> points;

    public:
    void getOptions(int argc, char * argv[]);
    void readInput();
    void mst();
    void tspAlg(bool inFunction);
    void findOptPath();
};

void printHelp() {
  cout  << "Usage: \'./letter\n\t[--quiet | -q]\n"                 
        <<                    "\t[--help | -h]\n"
        <<                    "\t< <input file>\'" << endl;
}  // printHelp()

void Data::getOptions(int argc, char * argv[]) {
  // These are used with getopt_long()
  opterr = false; // Let us handle all error output for command line options
  int choice;
  int index = 0;
  option long_options[] = {
    { "mode", required_argument,nullptr,'m' },
    { "help", no_argument,nullptr,'h' },
    { nullptr, 0, nullptr, '\0' },
  };  // long_options[]

  while ((choice = getopt_long(argc, argv, "m:h", long_options, &index)) != -1) {
    switch (choice) {
      case 'h': {
        printHelp();
        exit(0);
        break;
      }
      case 'm': {
        if (*optarg == 'M' ) {
            mstMode = true;
        } 
        else if (*optarg == 'F' ) {
            fast = true;
        } 
        else if (*optarg == 'O' ) {
            optimize = true;
        } 
        else {
            cerr << "Error: Invalid command line option" << endl;
            exit(1);
        }
        break;
      }
      default:
        cerr << "Error: Unknown command line option" << endl;
        exit(1);
        break;
    }  // switch ..choice
  }  // while
}  // getMode()




void Data::readInput() {
    //read in the input for the 
    int x_val = 0;
    int y_val = 0;
    cin >> numPoints;
    points.reserve(numPoints);

    while (cin >> x_val >> y_val) {
        Coordinates coord;
        coord.x_val = x_val;
        coord.y_val = y_val;
        if(x_val < 0 && y_val < 0) {
            coord.medicalCampus = true;
        }
        else if((x_val < 0 && y_val == 0) || 
                (x_val == 0 && y_val < 0) || 
                (x_val == 0 && y_val == 0)) {
            coord.border = true;
        }
        else {
            coord.mainCampus = true;
        }
        points.push_back(coord);
    }
    if (mstMode) {
        mst();
    }
    else if (fast) {
        tspAlg(false);
    }
    else if (optimize) {
        findOptPath();
    }
} 


/*
 **************************************************************************************
                      Part A: 
                        Minimum Spanning tree using Prim's alg
                      TODO:
                        * how can i make this faster?
                        * What do i need to do make sure this works?
                      
**************************************************************************************
*/

double calcDist(const Coordinates &one, const Coordinates &two) {
    if ((one.border || two.border) || (one.mainCampus && two.mainCampus) || (one.medicalCampus && two.medicalCampus) ) {
        return (static_cast<double>((two.x_val - one.x_val)) * (two.x_val - one.x_val)) + (static_cast<double>((two.y_val - one.y_val)) * (two.y_val - one.y_val));
    }
    return numeric_limits<double>::infinity();
}

void Data::mst() {
    
    vector<bool> visited(numPoints, false);
    vector<int> parent(numPoints, -1);
    vector<double> dist(numPoints, numeric_limits<double>::infinity());
    double weight = 0;

    //start of PRIMS alg 
    dist[0] = 0;

    for (size_t i = 0; i < numPoints; i++) {
        size_t min_idx = 0;
        double minDist = numeric_limits<double>::max();
        for (size_t j = 0; j < numPoints; j++) {
            if (!visited[j] && dist[j] < minDist) {
                minDist = dist[j];
                min_idx = j;
            }
        }
        visited[min_idx] = true;
        double distt = 0; 
        for (size_t k = 0; k < numPoints; k++) {
             distt = calcDist(points[min_idx], points[k]);
            if (distt != numeric_limits<double>::infinity() && !visited[k]
                && distt < dist[k]) {
                    parent[k] = static_cast<int> (min_idx);
                    dist[k] = distt;
            }
        }
        weight+= sqrt(dist[min_idx]);
    } // end of Prims

    cout << weight << '\n';
    for (size_t i = 1; i < numPoints; i++) {
        if (parent[i] > static_cast<int>(i)) {
            cout << i << " " << parent[i] << '\n';
        }
        else {
            cout << parent[i] << " " << i << '\n';
        }  
    }      
}

/*
 **************************************************************************************
                      Part B: 
                      TODO:
**************************************************************************************
*/


double calcDistonly (const Coordinates &one, const Coordinates &two) {
    return (static_cast<double>((two.x_val - one.x_val)) * (two.x_val - one.x_val)) + (static_cast<double>((two.y_val - one.y_val)) * (two.y_val - one.y_val));
}

void Data::tspAlg(bool inFunction) {
    vector<size_t> map;
    vector<bool> visited (numPoints, false);
    double weightTSP = 0; 
    double minDist = numeric_limits<double>::max();
    int min_idx = -1;
    map.reserve(numPoints);

    //step 1
    map.push_back(0);
    visited[0] = true;
    //does this go into the for loop
    for (size_t i = 1; i < numPoints; i++) {
        double distFirst = calcDist(points[0], points[i]); //adding the first points
        if (distFirst < minDist) {
            minDist = distFirst;
            min_idx = static_cast<int>(i);
        }
    }
    map.push_back(static_cast<size_t>(min_idx));
    visited[static_cast<size_t>(min_idx)] = true;
    weightTSP += sqrt(minDist) * 2;     

    for (size_t i = 1; i < numPoints; i++) {
        double minDist = numeric_limits<double>::max();
        min_idx = -1; 
        if (!visited[i]) {
            for (size_t j = 0; j < map.size() - 1; j++) {
                double distances = sqrt(calcDistonly(points[map[j]], points[i])) + sqrt(calcDistonly(points[i], points[map[j+1]])) - sqrt(calcDistonly(points[map[j]], points[map[j + 1]]));
                if (distances < minDist) {
                    minDist = distances;
                    min_idx = static_cast<int>(j);
                }
            }
        }
        if (min_idx != -1) {
            weightTSP +=  minDist;
            visited[static_cast<size_t>(min_idx)] = true;
            map.insert(map.begin() + static_cast<int>(min_idx) + 1, i);
        }
    }
    if (!inFunction) {
            cout << weightTSP << '\n';
        for (size_t k = 0; k < map.size(); k++) {
            cout << map[k] << " ";
        }
    }
    else {
        optimizePath.bestWeight = weightTSP;
        optimizePath.bestPath = map; 
    }
}


/*
 *************************************************************************************
                      Part C: 
                      TODO:
**************************************************************************************
*/

void OptimalClass::countDistances() {
    size_t numP = verticies.size();
    distanceMatrix.resize(numP, vector<double>(numP, 0));
    for (size_t i = 0; i < numP; i++) {
        for (size_t j = 0; j < numP; j++) {
            distanceMatrix[i][j] = calcDistonly(verticies[i], verticies[j]);
        }
    }
}


double OptimalClass::optMst(vector<size_t> &points, size_t numPoints) {
        
    vector<bool> visited(numPoints, false);
    vector<int> parent(numPoints, -1);
    vector<double> dist(numPoints, numeric_limits<double>::infinity());
    double weight = 0;

    //start of PRIMS alg 
    dist[0] = 0;

    for (size_t i = 0; i < numPoints; i++) {
        size_t min_idx = 0;
        double minDist = numeric_limits<double>::max();
        for (size_t j = 0; j < numPoints; j++) {
            if (!visited[j] && dist[j] < minDist) {
                minDist = dist[j];
                min_idx = j;
            }
        }
        visited[min_idx] = true;
        double distt = 0; 
        for (size_t k = 0; k < numPoints; k++) {
            distt = distanceMatrix[points[min_idx]][points[k]];
            if (distt != numeric_limits<double>::infinity() && !visited[k]
                && distt < dist[k]) {
                    parent[k] = static_cast<int> (min_idx);
                    dist[k] = distt;
            }
        }
        double amt = sqrt(dist[min_idx]);
        weight += amt;
    } // end of Prims
    return weight;
}

void Data::findOptPath() {
    optimizePath = OptimalClass(points);
    tspAlg(true);
    optimizePath.path = optimizePath.bestPath;
    optimizePath.countDistances();
    optimizePath.genPerms(1);
    cout << optimizePath.bestWeight << '\n';
    for (size_t i = 0; i < numPoints; i++) {
        cout << optimizePath.bestPath[i] << " ";
    }
}

void OptimalClass::genPerms(size_t permLength) {
    if (permLength == path.size()) {
        currWeight += sqrt(distanceMatrix[path[permLength-1]][path[0]]);
        if (currWeight < bestWeight) {
            bestWeight = currWeight;
            bestPath = path;
        }
        currWeight -= sqrt(distanceMatrix[path[permLength-1]][path[0]]);
        return;
    }
    if (!isPromsing(permLength)) {
        return;
    }
    for(size_t i = permLength; i < path.size(); i++) {
        swap(path[permLength], path[i]);
        currWeight += sqrt(distanceMatrix[path[permLength-1]][path[permLength]]); /*edge of new weight*/
        genPerms(permLength + 1);
        currWeight -= sqrt(distanceMatrix[path[permLength-1]][path[permLength]]); /*edge of new weight*/
        swap(path[permLength], path[i]);
    }
}

bool OptimalClass::isPromsing(size_t permLength) {
    double minDistanceS = numeric_limits<double>::infinity();
    double minDistanceF = numeric_limits<double>::infinity();
    vector<size_t> nonPerm;
    size_t sizepath = path.size() - permLength;
    nonPerm.reserve(sizepath);
    for (size_t i = permLength; i < path.size(); i++) {
        nonPerm.push_back(path[i]);
    }
    for (size_t i = 0; i < nonPerm.size(); i++) {
        if (distanceMatrix[path[0]][nonPerm[i]] < minDistanceS) {
            minDistanceS = distanceMatrix[path[0]][nonPerm[i]];
        }
        if (distanceMatrix[path[permLength - 1]][nonPerm[i]] < minDistanceF) {
            minDistanceF = distanceMatrix[path[permLength - 1]][nonPerm[i]];
        }
    }
    double weightOfMST = currWeight + (sqrt(minDistanceF) + sqrt(minDistanceS));
    if (weightOfMST > bestWeight) {
        return false;
    }
    weightOfMST += optMst(nonPerm, nonPerm.size());
    if (bestWeight > weightOfMST) {
        return true;
    }
    return false;
}


int main(int argc, char *argv[]) {
    // This should be in all of your projects, speeds up I/O
    ios_base::sync_with_stdio(false);
    cout << std::setprecision(2); //Always show 2 decimal places
    cout << std::fixed; //Disable scientific notation for large numbers
    //so it saves in xcode
    xcode_redirect(argc,argv);

    Data drone;
    drone.getOptions(argc, argv);
    drone.readInput();
    

    return 0;
} //...main





