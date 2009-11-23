/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
 * \file       DataLog.cpp
 * \author     Ellen Klingbeil
 */

#include "DataLog.hpp"

using std::string;
using std::endl;
using std::cerr;

namespace wbc {

DataLog::DataLog(char const * header, char const * filename, int const nData, bool rt_plot)
{
  rt_plot_ = rt_plot;
  if (rt_plot_) 
  {
    // Write data file info to master file for plotting gui.
    std::ofstream masterFile;
    path = string(WBC_DATA_DIR) + string("plotFiles.dat");
    masterFile.open(path.c_str(), std::ios::app);
    masterFile << filename << " ";
    masterFile << nData << "\n";
    masterFile.close();
  }

  cerr << "WBC_DATA_DIR:" << string(WBC_DATA_DIR) << endl << endl;
  filepath = string(WBC_DATA_DIR) + string(filename);
  dataFile.open(filepath.c_str());
  dataFile.precision( 12 );
  dataFile << "#  ";
  dataFile << header << "\n";

  // Only need to open/close file for each time step if plotting in real-time.
  if (rt_plot_)    
    dataFile.close();
}

DataLog::~DataLog()
{
  closeFile();
  
  // Clear contents of master file.
  std::ofstream masterFile;
  masterFile.open(path.c_str(), std::ios::in);
  masterFile.close();
}
    
void DataLog::appendData( const wbc::Time& t, const SAIVector &data)
{
  if (rt_plot_)    
    dataFile.open(filepath.c_str(), std::ios::app); //'a');
  
  // Write time.
  dataFile << t.dValue() << "\t";

  // Write data.
  for( int i = 0; i < data.size(); i++ ) {
    dataFile << data[i] << "\t";
  }
  // End of line
  dataFile << endl;

  if (rt_plot_)    
    dataFile.close();
}

void DataLog::appendData( const wbc::Time& t, const double data)
{
  SAIVector v(1);
  v[0] = data;
  appendData(t, v);
}

void DataLog::closeFile()
{
  if (dataFile.is_open()) {
    dataFile.close();
  }
}

}
