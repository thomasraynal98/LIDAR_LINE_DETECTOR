/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

struct Point_2D
{
  int i;
  int j;
  int cluster{0};
} point_2D;

struct Cluster
{
  int id;
  int nombre;
  int total_i;
  int total_j;
  Point_2D min;
  Point_2D max;
} cluster;

struct Data_lidar
{
  double angle;
  double value;
} data_lidar;

struct Vector
{
  double i_value;
  double j_value;
  double norme;
  double i_norm;
  double j_norm;
} vector;

struct Color
{
  int R{0};
  int G{0};
  int B{0};
} color;

void normalisation_vector(Vector* vector)
{
  /*  DESCRIPTION:
      this function will normalize vector.
  */
  vector->norme = pow(pow(vector->i_value,2)+pow(vector->j_value,2),0.5);
  vector->i_norm = vector->i_value / vector->norme;
  vector->j_norm = vector->j_value / vector->norme;
}

double dot_product(Vector v1, Vector v2)
{
  return v1.i_norm*v2.i_norm+v1.j_norm*v2.j_norm;
}

void inverse_min_max(Cluster * line)
{
  Point_2D memory;
  memory.i = line->max.i;
  memory.j = line->max.j;
  line->max.i = line->min.i;
  line->max.j = line->min.j;
  line->min.i = line->max.i;
  line->min.j = line->max.j;
}

void wall_detector(std::vector<Point_2D> * list_points, std::vector<Cluster> * list_line_cluster)
{
  /* DESCRIPTION:
     this function will try to detect if the lidar see a wall.
  */

  int threshold_number_point_wall = 15;
  double threshold_dot_product = 0.9995;
  int cluster_id = 1;
  int ii_memory = -1;
  Point_2D* A;
  Point_2D* B;
  Point_2D* C;

  for(int i = 0; i < list_points->size(); i++)
  {
    // found first point.
    if(list_points->at(i).cluster == 0)
    {
      A = &list_points->at(i);

      // found second point.
      for(int ii = i+1; ii < list_points->size(); ii++)
      {
        if(list_points->at(ii).cluster == 0 && A->i != list_points->at(ii).i && A->j != list_points->at(ii).j)
        {
          B = &list_points->at(ii);
          ii_memory = ii;
          ii = 999999;
        }
      }
      

      // we have or first vector.
      Vector v1, v2;
      v1.i_value = B->i - A->i;
      v1.j_value = B->j - A->j;

      // compare with all other vector.
      // std::cout << "i:" << i << " ii:" << ii_memory << " ";
      for(int ii = ii_memory+1; ii < list_points->size(); ii++)
      {
        // take valid vector.
        if(list_points->at(ii).cluster == 0 && A->i != list_points->at(ii).i && A->j != list_points->at(ii).j \
        && B->i != list_points->at(ii).i && B->j != list_points->at(ii).j)
        {
          C = &list_points->at(ii);
          v2.i_value = C->i - A->i;
          v2.j_value = C->j - A->j;
        }

        // normalize vector.
        normalisation_vector(&v1);
        normalisation_vector(&v2);

        // std::cout << ii << " - " << dot_product(v1,v2) << "\n";
        // dot product between vector.
        if(dot_product(v1,v2) >= threshold_dot_product && dot_product(v1,v2) <= 1)
        {
          A->cluster = cluster_id;
          B->cluster = cluster_id;
          C->cluster = cluster_id;
        }
      }

      // now check if we are enought clusterised point.
      int compteur = 0;
      Point_2D first;
      Point_2D last;
      double total_i = 0;
      double total_j = 0;
      Cluster new_line;
      for(int ii = i; ii < list_points->size(); ii++)
      {
        if(list_points->at(ii).cluster == cluster_id) {
          compteur += 1;
          total_i += list_points->at(ii).i - A->i;
          total_j += list_points->at(ii).j - A->j;
          if(compteur == 1) 
          {
            first.i = list_points->at(ii).i;
            first.j = list_points->at(ii).j;
            new_line.min.i = list_points->at(ii).i;
            new_line.min.j = list_points->at(ii).j;
          }
          last.i;
          last.j;
          new_line.max.i = list_points->at(ii).i;
          new_line.max.j = list_points->at(ii).j;
        }
      }
      if(compteur >= threshold_number_point_wall && pow((pow(last.i-first.i,2)+pow(last.j-first.j,2)),0.5)*5/400 > 2)
      {
        new_line.total_i = total_i;
        new_line.total_j = total_j;
        new_line.id = cluster_id;
        new_line.nombre = compteur;

        // TODO LA 
        
        if(new_line.min.i > new_line.max.i)
        {
          inverse_min_max(&new_line);
        }
        if(new_line.min.j < new_line.max.j)
        {
          // diagonal HG-BD.
          std::cout << "left\n";
        }
        else
        {
          // diagonal DG-HD.
          std::cout << "right\n";
        }

        list_line_cluster->push_back(new_line);
      }
      else
      {
        A->cluster = -1;
        for(auto point : *list_points)
        {
          if(point.cluster == cluster_id) {point.cluster = 0;}
        }
      }

      cluster_id += 1;
    }
  }
}

void show_interface(cv::Mat interface_visuel, std::vector<Data_lidar> data_lidar_sample, std::vector<Color> color_liste, \
std::vector<Point_2D> * list_points, std::vector<Cluster> * list_line_cluster)
{ 
  /*  DESCRIPTION:
      this function will take all lidar data and show them in beautiful 
      opencv interface.
  */

  // convert brut data to 2D points.
  Point_2D last_save_for_WD;      
  last_save_for_WD.i = 0;
  last_save_for_WD.j = 0;
  for(auto point : data_lidar_sample)
  {
    int pixel_x = sin(point.angle)*point.value*400/5+400;
    int pixel_y = cos(point.angle)*point.value*400/5+400;
    if(pixel_x != 400 && pixel_y != 400 && point.value > 0.3)
    {
      Point_2D new_p;
      new_p.i = pixel_x;
      new_p.j = pixel_y;
      // std::cout << "point(" << pixel_x << "," << pixel_y << ") ";
      if(pow((pow(new_p.i-last_save_for_WD.i,2)+pow(new_p.j-last_save_for_WD.j,2)),0.5)*5/400 > 0.1)
      {list_points->push_back(new_p);
        last_save_for_WD.i = pixel_x;
        last_save_for_WD.j = pixel_y;
      }
    }
    // std::cout << point.angle << "," << point.value << " en pixel : " << pixel_x << "," << pixel_y << "\n";
    cv::circle(interface_visuel, cv::Point(pixel_x,pixel_y),1, cv::Scalar(0,0,0), cv::FILLED, 1, 0);
  }
  wall_detector(list_points, list_line_cluster);
  int color_compteur = 0;
  std::cout << "[LINE DETECTION]" << "\n";
  for(auto line : *list_line_cluster)
  {
    //compute angle.
    double t = pow(pow(line.total_i,2)+pow(line.total_j,2),0.5);
    double angle = acos(line.total_i/t);
    

    std::cout << "cluster: " << line.id << " ,size:" << line.nombre << " ,angle: " << angle << " totali:" << line.total_i << "," << line.total_j << "\n";
    for(auto point : *list_points)
    {
      if(point.cluster == line.id)
      {
        cv::circle(interface_visuel, cv::Point(point.i,point.j),2, cv::Scalar(color_liste[color_compteur].B,color_liste[color_compteur].G,color_liste[color_compteur].R), cv::FILLED, 1, 0);
      }
    }
    color_compteur += 1;
    if(color_compteur >= color_liste.size()) {color_compteur = 0;}
  }

  cv::namedWindow("interface_visuel",cv::WINDOW_AUTOSIZE);
  cv::imshow("interface_visuel", interface_visuel);

}

int main(int argc, char *argv[]) {
  
  // THOMAS VARIABLE
  cv::Mat interface_visuel(450, 800, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::circle(interface_visuel, cv::Point(400,400),4, cv::Scalar(0,0,0), cv::FILLED, 1, 0);
  cv::Mat copy_interface_visuel(450, 800, CV_8UC3, cv::Scalar(255, 255, 255));
  std::vector<Data_lidar> data_lidar_sample;
  std::vector<Color> color_liste;
  Color rouge;
  rouge.R = 255;
  Color bleu;
  bleu.B = 255;
  Color vert;
  vert.G = 255;
  color_liste.push_back(rouge);
  color_liste.push_back(bleu);
  color_liste.push_back(vert);
  std::vector<Point_2D> list_points;
  std::vector<Cluster> list_line_cluster;
  //

  // while(true)
  // {
  //   copy_interface_visuel = interface_visuel.clone();
  //   show_interface(copy_interface_visuel, data_lidar_sample);
  //   char d=(char)cv::waitKey(25);
  //   if(d==27)
  //     break;
  // }

  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  ydlidar::init(argc, argv);

  std::map<std::string, std::string> ports =
    ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 230400;
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 128000;
  baudrateList[2] = 153600;
  baudrateList[3] = 230400;
  baudrateList[4] = 512000;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("%d. %d\n", it->first, it->second);
  }

  while (ydlidar::ok()) {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;
    // number = 1;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  bool isSingleChannel = false;
  bool isTOFLidar = false;
  std::string input_channel;
  std::string input_tof;
  printf("Whether the Lidar is one-way communication[yes/no]:");
  // std::cin >> input_channel;
  input_channel = "no";
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_channel.find("yes") != std::string::npos) {
    isSingleChannel = true;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  printf("Whether the Lidar is a TOF Lidar [yes/no]:");
  // std::cin >> input_tof;
  input_tof = "no";
  std::transform(input_tof.begin(), input_tof.end(),
                 input_tof.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_tof.find("yes") != std::string::npos) {
    isTOFLidar = true;
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  std::string input_frequency;

  float frequency = 8.0;

  while (ydlidar::ok() && !isSingleChannel) {
    printf("Please enter the lidar scan frequency[3-15.7]:");
    // std::cin >> input_frequency;
    input_frequency = "5";
    frequency = atof(input_frequency.c_str());

    if (frequency <= 15.7 && frequency >= 3.0) {
      break;
    }

    fprintf(stderr,
            "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if (!ydlidar::ok()) {
    return 0;
  }




  CYdLidar laser;
  //<! lidar port
  laser.setSerialPort(port);
  //<! lidar baudrate
  laser.setSerialBaudrate(baudrate);

  //<! fixed angle resolution
  laser.setFixedResolution(false);
  //<! rotate 180
  laser.setReversion(false); //rotate 180
  //<! Counterclockwise
  laser.setInverted(false);//ccw
  laser.setAutoReconnect(true);//hot plug
  //<! one-way communication
  laser.setSingleChannel(isSingleChannel);

  //<! tof lidar
  laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  //unit: °
  laser.setMaxAngle(2*90);
  laser.setMinAngle(2*-90);

  //unit: m
  laser.setMinRange(0.01);
  laser.setMaxRange(10.0);

  //unit: Hz
  laser.setScanFrequency(frequency);
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError))
 {
      /*fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(),
	      1.0 / scan.config.scan_time);*/

      // THOMAS == //
      list_line_cluster.clear();
      data_lidar_sample.clear();
      list_points.clear();
      for(auto point : scan.points)
      {
        if(abs(point.angle)>M_PI_2)
        {
          Data_lidar observation;
          observation.angle = -point.angle;
          observation.value = point.range;
          data_lidar_sample.push_back(observation);
        // std::cout << "[" << point.angle*180/M_PI << "," << point.range << "\n"; 
        }
      }
      
      copy_interface_visuel = interface_visuel.clone();
      show_interface(copy_interface_visuel, data_lidar_sample, color_liste, &list_points, &list_line_cluster);
      char d=(char)cv::waitKey(25);
      if(d==27)
        break;
      // ========= //

      fflush(stdout);
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}