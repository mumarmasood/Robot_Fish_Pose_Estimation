// Plot 2D pose data from the csv file which has the following format: Time, X, Y, Theta

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    // Read the csv file
    string filename = "pose_data.csv";
    ifstream file(filename
    );
    if (!file.is_open()) {
        cerr << "Error: file not found" << endl;
        return -1;
    }

    // Read the data
    vector<double> time, x, y, theta;
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string token;
        getline(ss, token, ',');
        time.push_back(stod(token));
        getline(ss, token, ',');
        x.push_back(stod(token));
        getline(ss, token, ',');
        y.push_back(stod(token));
        getline(ss, token, ',');
        theta.push_back(stod(token));
    }

    // Plot the data
    Mat plot(600, 600, CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < time.size(); i++) {
        int x_ = 300 + x[i] * 100;
        int y_ = 300 - y[i] * 100;
        circle(plot, Point(x_, y_), 2, Scalar(0, 0, 0), -1);
        line(plot, Point(x_, y_), Point(x_ + 10 * cos(theta[i]), y_ - 10 * sin(theta[i])), Scalar(0, 0, 255), 2);
    }

    // Show the plot
    imshow("2D Plot", plot);
    waitKey(0);
    return 0;

}

// The csv file should have the following format:
// Time, X, Y, Theta
// 0, 0, 0, 0
// 1, 1, 1, 1
// 2, 2, 2, 2
// 3, 3, 3, 3
// 4, 4, 4, 4
// 5, 5, 5, 5
// 6, 6, 6, 6
