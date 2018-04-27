#ifndef _DATA_TYPE_
#define _DATA_TYPE_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;

typedef typename std::vector< array<double, 3> > Path3D; 
typedef typename std::vector< array<double, 2> > Path2D;

struct Cube;
struct Cube
{     
      //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;   // the 8 vertex of a cube 
      Eigen::MatrixXd vertex;
      Eigen::Vector3d center; // the center of the cube
      bool valid;    // indicates whether this cube should be deleted

      double t; // time allocated to this cube
      vector< pair<double, double> > box;
/*
           P4------------P3 
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /  
        P5------------P6              / x
*/                                                                                 

      // create a cube using 8 vertex and the center point
      Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(3);
      }

      // create a inscribe cube of a ball using the center point and the radius of the ball
      void setVertex( Eigen::MatrixXd vertex_)
      {     
            vertex = vertex_;
            setBox();
      }
      
      void setBox()
      {
            box.clear();
            box.resize(3);
            box[0] = make_pair( vertex(3, 0), vertex(0, 0) );
            box[1] = make_pair( vertex(0, 1), vertex(1, 1) );
            box[2] = make_pair( vertex(4, 2), vertex(1, 2) );
      }

      void printBox()
      {
            cout<<"center of the cube: \n"<<center<<endl;
            cout<<"vertex of the cube: \n"<<vertex<<endl;
      }

      Cube()
      {  
         center = Eigen::VectorXd::Zero(3);
         vertex = Eigen::MatrixXd::Zero(8, 3);

         valid = true;
         t = 0.0;
         box.resize(3);
      }

      ~Cube(){}
};

struct Cube2D;
struct Cube2D
{     
      //Eigen::Vector3d p1, p2, p3, p4
      Eigen::MatrixXd vertex;
      Eigen::Vector2d center; // the center of the cube
      bool valid;    // indicates whether this cube should be deleted

      double t; // time allocated to this cube
      vector< pair<double, double> > box;
/*
        P4------------P3 
         |             |              
         |             |             
        P1------------P2              
            
         |--------> y
         | 
         |
         x
*/                                                                                 

      // create a 2D cube using 4 vertex and the center point
      Cube2D( Eigen::MatrixXd vertex_, Eigen::Vector2d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(2);
      }

      // create a inscribe cube of a ball using the center point and the radius of the ball
      void setVertex( Eigen::MatrixXd vertex_)
      {     
            vertex = vertex_;
            setBox();
      }
      
      void setBox()
      {
            box.clear();
            box.resize(2);
            box[0] = make_pair( vertex(3, 0), vertex(0, 0) );
            box[1] = make_pair( vertex(0, 1), vertex(1, 1) );
      }

      void printBox()
      {
            cout<<"center of the cube: \n"<<center<<endl;
            cout<<"vertex of the cube: \n"<<vertex<<endl;
      }

      Cube2D()
      {  
         center = Eigen::VectorXd::Zero(2);
         vertex = Eigen::MatrixXd::Zero(4, 2);

         valid = true;
         t = 0.0;
         box.resize(2);
      }

      ~Cube2D(){}
};

#endif