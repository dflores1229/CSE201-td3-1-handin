#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {

    double* newArr = new double[new_size];

    //copy over elements into new array
    for (int i=0; i<length; i++){
        newArr[i] = array[i];
    }

    //initialize remaining elements to 0
    for(;length< new_size; length++){
        newArr[length] = 0;
    }

    //deallocate memory used by old array
    delete array;

    return newArr;
}

double* shrink_array(double* array, int length, int new_size) {

    //allocate new array (smaller size)
    double* newArr = new double[new_size];

    //copy over elements into new array
    for (int i=0; i<new_size; i++){
        newArr[i] = array[i];
    }

    //deallocate memory used by old array
    delete array;

  return newArr;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {

    //ff current_size == max_size, need to extend array (since no more space)
    if (current_size == max_size){
        //increase array size by 5 elements
        array = extend_array(array, current_size, current_size+5);
        //update max size of array
        max_size +=5;
    }

    //append element to next available space in array
    array[current_size] = element;
    //update number of elements in array
    current_size++;

  return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {

  //shrink array if the difference between the total number of used elements
  //total_elements and the array maximum size array_size is at least 5
  if ( (max_size - current_size) >= 4){
      //shrink array size by 5 elements
      array = shrink_array(array, current_size, max_size-5);
      //update max array size
      max_size -=5;
  } else{ //if don't need to shrink array, change last element to 0
      array[current_size-1] = 0;
  }

  //update number of elements in array
  current_size--;

  return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {

    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;

      //update t, x, and y coords to array
      telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;

      //update t, x, and y coords to array
      telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

    } else {

        //update t, x, and y coords to array
        telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

        t = t + simulation_interval;
        y = v0_y * t  - 0.5 * g * t * t;
        x = v0_x * t;
    }
  }

  return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {

    //Step 1: Append all telemetries to global array
    //iterate through array of telemetries
    for (int i=0; i< tot_telemetries; i++){

        //iterate through current telemetry (representing a single projectile)
        for ( int j=0; j< telemetries_sizes[i]; j+=3){

            //append next set of points (t,x,y)
            global_telemetry = append_to_array(telemetries[i][j], global_telemetry,
                                               global_telemetry_current_size, global_telemetry_max_size);
            global_telemetry = append_to_array(telemetries[i][j+1], global_telemetry,
                                               global_telemetry_current_size, global_telemetry_max_size);
            global_telemetry = append_to_array(telemetries[i][j+2], global_telemetry,
                                               global_telemetry_current_size, global_telemetry_max_size);
        }//end iterating through current telemetry
    }//end iterating through all telemetries

    //Step 2: order sets of points (t,x,y) by increasing order of t
    for(int i=0; i<= global_telemetry_current_size-3; i+=3){

        for(int j=i+3; j<= global_telemetry_current_size-3; j+=3 ){

            if( global_telemetry[i] > global_telemetry[j]){

                //switch t coord
                double temp = global_telemetry[i];
                global_telemetry[i] = global_telemetry[j];
                global_telemetry[j] = temp;

                //switch x coord
                temp = global_telemetry[i+1];
                global_telemetry[i+1] = global_telemetry[j+1];
                global_telemetry[j+1] = temp;

                //switch y coord
                temp = global_telemetry[i+2];
                global_telemetry[i+2] = global_telemetry[j+2];
                global_telemetry[j+2] = temp;
            }
        }
    } //end ordering of global_telemetry

}
