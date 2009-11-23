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
 * \file       DataLog.hpp
 * \author     Ellen Klingbeil
 */

#ifndef WBC_DATALOG_HPP
#define WBC_DATALOG_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <cstring>
#include <err.h>
#include <unistd.h>
#include <stdlib.h>

#include <wbc/util/saiTime.hpp>
#include <saimatrix/SAIVector.h>


namespace wbc {

const std::string WBC_DATA_DIR = "../../rtviz/data_files/";

class DataLog
{
  public:
    DataLog(char const * header, char const * filename, int const nData, bool rt_plot = true);
    ~DataLog();
    
    //! Write data
    void appendData( const wbc::Time& t, const SAIVector &);
    void appendData( const wbc::Time& t, const double);
    void closeFile();
  
  private:
    std::ofstream dataFile;
    std::string path;
    std::string filepath;
    wbc::Time time;
    bool rt_plot_;
};

}

#endif
