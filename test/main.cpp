//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <vigir_perception_msgs/filtered_scan_conversion.h>


int main(int argc, char** argv)
{
  vigir_perception_msgs::FilteredLocalizedLaserScan filtered_scan;
  vigir_perception_msgs::CompressedFilteredScanData compressed_scan;

  compressed_scan.scan.push_back(3000);
  compressed_scan.scan.push_back(33000);
  compressed_scan.scan.push_back(65535);
  compressed_scan.scan.push_back(256);

  compressed_scan.intensities.push_back(0);
  compressed_scan.intensities.push_back(123);
  compressed_scan.intensities.push_back(10);
  compressed_scan.intensities.push_back(255);

  
  sensor_msgs::LaserScan scan_properties;

  //scan_properties.header.stamp = ros::Time();
  scan_properties.header.frame_id = "scan_header_test";

  scan_properties.angle_min = -1.0;
  scan_properties.angle_max = 1.0;
  scan_properties.angle_increment = 0.5;
  scan_properties.angle_increment = 0.5;
  scan_properties.scan_time = 2.0;
  scan_properties.range_min = 0.1;
  scan_properties.range_max = 20.0;


  std::cout << scan_properties;

  convertCompressedToFilteredScan(compressed_scan,
                                  scan_properties,
                                  filtered_scan);

  std::cout << "\nFiltered scan after conversion\n" << filtered_scan;

  //std::cout << "\nCompressed before to/from conversion\n" << compressed_scan;

  vigir_perception_msgs::CompressedFilteredScanData compressed_scan_out;
  
  convertFilteredToCompressedScan(filtered_scan,                                  
                                  compressed_scan_out);


  std::cout << "\nCompressed after to/from conversion\n" << compressed_scan_out;
  
  return 0;
}
