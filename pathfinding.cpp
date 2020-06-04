#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <math.h>
#include <limits>
#include <string.h>
#include <chrono>
using namespace std;
using namespace std::chrono;

struct Position {
    int x;
    int y;
};


struct PathFindStruct
{
    Position pos;
    Position camefrom;
    double cost = 1;
    double gscore = numeric_limits<double>::infinity();
    double fscore = numeric_limits<double>::infinity();
    bool in_open_set = false;
};


void print_vector(vector<PathFindStruct> v) {
    cout << "[";
    for (PathFindStruct item : v) {
        cout << "[" << item.pos.x << ", " << item.pos.y << "],";
    }
    cout << "]";
}


void print_map_grid(vector<vector<PathFindStruct>> v){
    cout << "Printing map..." << endl;
    for (vector<PathFindStruct> row : v) {
        for (PathFindStruct item : row) {
            if (item.cost >= 10) {
                cout << " m " << " ";
            } else {
                cout << item.fscore << " ";
            }
        }
        cout << endl;
    }
}

void print_map_grid_pos(vector<vector<PathFindStruct>> v){
    cout << "Printing map..." << endl;
    Position pos;
    for (vector<PathFindStruct> row : v) {
        for (PathFindStruct item : row) {
            pos = item.pos;
            cout << pos.x << "," << pos.y << " ";
        }
        cout << endl;
    }
}


void transpose(vector<vector<PathFindStruct>> &b)
{
    if (b.size() == 0)
        return;

    vector<vector<PathFindStruct>> trans_vec(b[0].size(), vector<PathFindStruct>());

    PathFindStruct item;

    for (int i = 0; i < b.size(); i++)
    {
        for (int j = 0; j < b[i].size(); j++)
        {
            item = b[i][j];
            item.pos = {i, j};
            trans_vec[j].push_back(item);
        }
    }

    b = trans_vec;
}


class Finder {
private:
    vector<vector<PathFindStruct>> map_grid_clear;
    vector<PathFindStruct> reconstruct_path(PathFindStruct target,
                                            vector<vector<PathFindStruct>> map_grid);
    double heuristic(Position pos1, Position pos2);
    bool write_to_file;

public:
    Finder(vector<vector<PathFindStruct>> map_grid, bool write_to_file);
    vector<PathFindStruct> find_path(PathFindStruct start,
                                     PathFindStruct target);
    vector<vector<PathFindStruct>> get_map_grid();
    PathFindStruct get_node(int x, int y);
};


Finder::Finder(vector<vector<PathFindStruct>> map_grid, bool write_to_file_) {
    map_grid_clear = map_grid;
    write_to_file = write_to_file_;
}


vector<PathFindStruct> Finder::reconstruct_path(PathFindStruct target,
                                                vector<vector<PathFindStruct>> map_grid) {
    vector<PathFindStruct> path = {target};
    PathFindStruct current = target;
    Position current_cf = current.camefrom;
    while (current_cf.x != -9 and current_cf.y != -9) {
        current = map_grid[current_cf.y][current_cf.x];
        current_cf = current.camefrom;
        path.push_back(current);
    }

    reverse(path.begin(),path.end());
    return path;
}


double Finder::heuristic(Position pos1, Position pos2) {
    double x_diff = pos1.x - pos2.x;
    double y_diff = pos1.y - pos2.y;

    double dist = sqrt(x_diff*x_diff + y_diff*y_diff);

    return dist;
}


vector<PathFindStruct> Finder::find_path(PathFindStruct start,
                                         PathFindStruct target) {

    vector<vector<PathFindStruct>> map_grid = map_grid_clear;
    ofstream outfile;
    Position target_pos = target.pos;
    Position start_pos = start.pos;
    start.camefrom.x = -9;
    start.camefrom.y = -9;
    start.gscore = 0;
    start.fscore = heuristic(start_pos, target_pos);

    vector<PathFindStruct> open_set = {start};
    vector<double> open_set_fscores = {start.fscore};
    vector<PathFindStruct> closed_set = {};
    start.in_open_set = true;

    map_grid[start_pos.y][start_pos.x] = start;

    int map_x_size = map_grid[0].size();
    int map_y_size = map_grid.size();
    int argmin = 0;
    int other_x;
    int other_y;

    PathFindStruct current;
    PathFindStruct neighbor;
    Position current_pos;
    Position neighbor_pos;
    double tentative_gscore;
    int index;

    if (write_to_file) {
        outfile.open("output.txt");
    }

    while (open_set.size() > 0) {
        argmin = distance(open_set_fscores.begin(),
                          min_element(open_set_fscores.begin(),
                          open_set_fscores.end()));
        current = open_set[argmin];


        if (outfile.is_open()) {
            for (vector<PathFindStruct> row : map_grid) {
                for (PathFindStruct item : row) {
                    outfile << "[";
                    outfile << item.pos.x << ",";
                    outfile << item.pos.y << ",";
                    outfile << item.camefrom.x << ",";
                    outfile << item.camefrom.y << ",";
                    outfile << item.gscore << ",";
                    outfile << item.fscore << ",";
                    outfile << item.cost << ",";
                    outfile << item.in_open_set << "];";
                }
            }
            outfile << "[" << current.pos.x << "," << current.pos.y << "];";
            outfile << "\n";
        }
        

        if (current.pos.x == target.pos.x && current.pos.y == target.pos.y) {
            //cout << "Target reached" << endl;
            if (outfile.is_open()) {
                outfile.close();
            }
            return reconstruct_path(current, map_grid);
        }

        open_set.erase(open_set.begin() + argmin);
        open_set_fscores.erase(open_set_fscores.begin() + argmin);
        current.in_open_set = false;
        closed_set.push_back(current);

        map_grid[current.pos.y][current.pos.x] = current;

        current_pos = current.pos;
        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                if (i == 0 && j == 0) {
                    continue;
                }
                other_x = current_pos.x + j;
                other_y = current_pos.y + i;
                if (other_x < 0 || other_y < 0) {
                    continue;
                }
                if (other_x >= map_x_size || other_y >= map_y_size) {
                    continue;
                }

                neighbor = map_grid[other_y][other_x];
                neighbor_pos = neighbor.pos;

                tentative_gscore = (double)current.gscore + sqrt((double)(i*i) + (double)(j*j))*(double)neighbor.cost;
                
                if (tentative_gscore < neighbor.gscore) {
                    neighbor.camefrom.x = current_pos.x;
                    neighbor.camefrom.y = current_pos.y;
                    neighbor.gscore = tentative_gscore;
                    neighbor.fscore = tentative_gscore + heuristic(neighbor_pos, target_pos);

                    index = 0;
                    for (PathFindStruct item : open_set) {
                        if (item.pos.x == neighbor.pos.x && item.pos.y == neighbor.pos.y) {
                            break;
                        }
                        index++;
                    }
                    
                    if (index >= open_set.size()) {
                        open_set.push_back(neighbor);
                        open_set_fscores.push_back(neighbor.fscore);
                        neighbor.in_open_set = true;
                    } else {
                        open_set[index] = neighbor;
                        open_set_fscores[index] = neighbor.fscore;
                    }

                    map_grid[other_y][other_x] = neighbor;
                }
            }
        }
    }
    if (outfile.is_open()) {
        outfile.close();
    }
    return {};
}


vector<vector<PathFindStruct>> Finder::get_map_grid() {
    return map_grid_clear;
}


PathFindStruct Finder::get_node(int x, int y) {
    return map_grid_clear[y][x];
}



int main() {
    string line;
    string letter;
    vector<vector<PathFindStruct>> map_grid;
    ifstream infile("maptxt_1.txt");

    int y = 0;
    int x = 0;
    if (infile.is_open()) {
        while (getline(infile,line)) {
            vector<PathFindStruct> row;
            for (int i = 0; i < line.length(); i++) {
                letter = line[i];
                Position pos{x, y};
                Position cf{-9, -9};
                PathFindStruct new_item{pos, cf, 1};
                if (letter == "1") {
                    new_item.cost = numeric_limits<double>::infinity();
                }
                row.push_back(new_item);
                x++;
            }
            map_grid.push_back(row);
            x = 0;
            y++;
        }
        infile.close();
    }
    transpose(map_grid);

    Finder finder{map_grid, false};

    PathFindStruct start;
    vector<PathFindStruct> final_path;
    PathFindStruct target = finder.get_node(0, 0);

    auto begin = high_resolution_clock::now();
    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 32; j++) {
            start = finder.get_node(j, i);
            final_path = finder.find_path(start, target);
        }
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - begin);

    double time_elapsed = duration.count();

    int count = 32*32;

    cout << "C++: " << count << " paths computed in:" << endl;
    cout << time_elapsed/1000.0 << " seconds" << endl;
    cout << time_elapsed/1000.0/(double)count << " seconds/path" << endl;
    
    return 0;
}