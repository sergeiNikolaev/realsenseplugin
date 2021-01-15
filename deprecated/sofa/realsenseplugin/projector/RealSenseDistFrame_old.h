/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/opencvplugin/BaseOpenCVStreamer.h>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>

namespace sofa {

namespace rgbdtracking {

using DataCallback = core::objectmodel::DataCallback ;

/*!
 * \brief The RealSenseDistFrame class
 * Used for distance frame usage as sofa data by components
 */
class RealSenseDistFrame_old {
public :
    typedef struct {
        size_t _width ;
        size_t _height ;
        float* frame ;
    } RealSenseDistStruct ;
    RealSenseDistStruct _distdata ;

    RealSenseDistFrame_old () {}

    RealSenseDistFrame_old (RealSenseDistStruct diststr) {
        _distdata = diststr ;
    }

    operator RealSenseDistStruct & () {
        return getFrame() ;
    }
    operator const RealSenseDistStruct & () {
        return getFrame() ;
    }

    inline size_t width() const {
        return _distdata._width ;
    }

    inline size_t height() const {
        return _distdata._height;
    }

    inline float* data() {
        return _distdata.frame ;
    }

    inline RealSenseDistStruct & getFrame () {
        return _distdata ;
    }

    friend std::istream& operator >> ( std::istream& in, RealSenseDistFrame_old&  )
    {
        return in;
    }

    friend std::ostream& operator << ( std::ostream& out, const RealSenseDistFrame_old&  )
    {
        return out;
    }
} ;

/*!
 * \brief The RealSenseDistFrameExporter class
 * exports distance frames to file for offline processing
 */
class RealSenseDistFrameExporter_old : public core::objectmodel::BaseObject
{

public:
    SOFA_CLASS( RealSenseDistFrameExporter_old , core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<std::string>  d_filename ;
    DataCallback c_distframe ;

    Data<RealSenseDistFrame_old>  d_distframe ;
    DataCallback c_filename ;

    Data<int> d_fpf; // frame per file

    RealSenseDistFrameExporter_old()
        : d_filename (initData(&d_filename, "filename", "output filename"))
        , d_distframe (initData(&d_distframe, "distframe", "link to distFrame data"))
        , d_fpf(initData(&d_fpf, 18000, "fpf", "frame per file"))
    {
        c_distframe.addInput(&d_distframe);
        c_distframe.addCallback(std::bind(&RealSenseDistFrameExporter_old::saveFrame, this));
        filestream = nullptr ;
        frame_count = 0 ;
        file_id = 0 ;
    }

    ~RealSenseDistFrameExporter_old() {
        std::fclose(filestream) ;
    }

    void updateFileStream () {
        if (filestream == nullptr) {
            std::string filename = processFileName() ;
            filestream = std::fopen(filename.c_str(), "wb") ;
            frame_count = 0 ;
        }
    }

    ///\brief write the frame in an opened binary file stream
    void saveFrameToStream() {
        RealSenseDistFrame_old distframe = d_distframe.getValue() ;
        RealSenseDistFrame_old::RealSenseDistStruct diststruct = distframe.getFrame();

        // write width and height
        std::fwrite(&diststruct._width, sizeof(size_t), 1, filestream) ;
        std::fwrite(&diststruct._height, sizeof(size_t), 1, filestream) ;

        // write frame data
        std::fwrite (
            diststruct.frame,
            sizeof(float),
            diststruct._width * diststruct._height,
            filestream
        ) ;
    }

    ///\brief Calledback each time distance frame changes
    void saveFrame () {
        updateFileStream();
        if (filestream == nullptr) {
            std::cerr << "stream is unopened. check rights on file" << std::endl ;
            return ;
        }

        saveFrameToStream();

        if (++frame_count >= d_fpf.getValue()) {
            std::fclose(filestream) ;
            filestream = nullptr ;
        }
    }

    std::string processFileName () {
    // reads file name from data and processes it
    // (add digit before extension)
        std::string
            extension = d_filename.getValue() , // starts as whole filename but ends up as extension
            delimiter = "." ;
        std::string filename = extension.substr(0, extension.find_last_of(delimiter)) ;
        extension.erase(0, extension.find_last_of(delimiter)) ;
        return filename + std::to_string(++file_id) + extension ;
    }

protected :
    std::FILE* filestream ;
    size_t frame_count, file_id ;

} ;

/*!
 * \brief The RealSenseDistFrameStreamer class
 * Streams through distance frames in a file
 * Implements BaseOpenCVStreamer
 */
class RealSenseDistFrameStreamer_old : public opencvplugin::streamer::BaseOpenCVStreamer
{

public:
    SOFA_CLASS( RealSenseDistFrameStreamer_old, opencvplugin::streamer::BaseOpenCVStreamer);
    typedef opencvplugin::streamer::BaseOpenCVStreamer Inherited;

    Data<std::string>  d_filename ;
    Data<RealSenseDistFrame_old>  d_distframe ;

    DataCallback c_filename ;
    std::FILE* filestream ;

    RealSenseDistFrameStreamer_old()
        : Inherited()
        , d_filename (initData(&d_filename, "filename", "output filename"))
        , d_distframe (initData(&d_distframe, "distframe", "link to distFrame data"))
    {
        c_filename.addInput(&d_filename);
        c_filename.addCallback(std::bind(&RealSenseDistFrameStreamer_old::updateFileStream, this));
        filestream = nullptr ;
    }

    virtual void decodeImage(cv::Mat & /*img*/) {
        readFrame();
    }

    void updateFileStream () {
        if (filestream == nullptr) {
            filestream = std::fopen(d_filename.getValue().c_str(), "rb") ;
        }
    }

    void readFrame () {
//        updateFileStream();
        if (filestream == nullptr) {
            std::cerr << "(RealSenseDistFrameStreamer) stream is unopened. check stream state before passing to function" << std::endl ;
            return ;
        }

        if (std::feof(filestream)) {
            std::cout << "(RealSenseDistFrameStreamer) End of dist file reached" << std::endl ;
            return ;
        }

        RealSenseDistFrame_old::RealSenseDistStruct diststruct ;
        // write width and height
        std::fread(&diststruct._width, sizeof(size_t), 1, filestream) ;
        std::fread(&diststruct._height, sizeof(size_t), 1, filestream) ;

        // write frame data
        diststruct.frame = new float[diststruct._width * diststruct._height] ;
        std::fread (
            diststruct.frame,
            sizeof(float),
            diststruct._width * diststruct._height,
            filestream
        ) ;
        RealSenseDistFrame_old distFrm (diststruct) ;
        d_distframe.setValue(distFrm);
    }
} ;

} // namespace rgbd

}

