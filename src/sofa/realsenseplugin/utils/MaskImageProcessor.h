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

#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/defaulttype/DataTypeInfo.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <sofa/opencvplugin/BaseOpenCVTracker.h>
#include <sofa/opencvplugin/BaseOpenCVStreamer.h>

#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/opencvplugin/BaseOpenCVData.h>
#include <sofa/core/visual/VisualParams.h>

#include <GL/glew.h>



namespace sofa
{

namespace realsenseplugin
{

/*!
 * \brief The RealSenseOfflineReader class
 * Loading image mask and extract points from it
 */
class MaskImageProcessor : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS( MaskImageProcessor, core::objectmodel::BaseObject );
	typedef core::objectmodel::BaseObject Inherited;

    Data<defaulttype::Vector2> d_resolution;

    /// \brief path to mask images or video
    sofa::core::objectmodel::DataFileName d_path_mask;

    /// mask image
    Data<opencvplugin::ImageData> d_maskImage;
    cv::Mat maskImageInternal;

    /// \brief output pointcloud
    Data<helper::vector<defaulttype::Vector2> > d_out;

    /// \brief resp. opencv color and depth streams
    cv::VideoCapture _reader_mask;

    Data<opencvplugin::CornersData > d_corners;
    Data<opencvplugin::ImageData> d_currImage;
    Data<bool> d_draw;

    /// \brief pause
    bool paused;

    MaskImageProcessor()
        : Inherited()
        , d_resolution(initData(&d_resolution, defaulttype::Vector2(640, 480), "resolution", "realsense camera resolution"))
        , d_path_mask(initData(&d_path_mask, "pathmask", "path to mask files"))
        , d_maskImage(initData(&d_maskImage, "maskImage", "image mask"))
        , d_out(initData(&d_out, "out", "out image points"))
        , d_corners(initData(&d_corners, "corners", "data link to corners"))
        , d_currImage(initData(&d_currImage, "imageCurr", "Current image"))
        , d_draw(initData(&d_draw, true, "draw","boolean for drawing selected points"))
        , paused(false)
    {
        this->f_listening.setValue(true);
    }

    /*!
     * \brief init
     * open color, depth and pointcloud video/file streams
     */
    void init() {
        openmask();

        // read next mask image
        maskImageInternal = readmask();

        // inversion to process white objects on black background
        d_maskImage.setValue(opencvplugin::ImageData(maskImageInternal));

        // detect contours
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat greyImage, binaryMask;
        cv::cvtColor(maskImageInternal, greyImage, cv::COLOR_BGR2GRAY);
        cv::threshold(greyImage, binaryMask, 127, 255, cv::THRESH_BINARY_INV);
        //std::string ty = type2str( binaryMask.type() );
        //std::cout << "Matrix: " << ty.c_str() << " " << binaryMask.cols << " " << binaryMask.rows << std::endl;
        cv::findContours(binaryMask, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        // compute boundboxes for contours and keep their centers
        helper::WriteAccessor<Data<helper::vector<defaulttype::Vector2> >> vMaskPoints = d_out;
        vMaskPoints.resize(contours.size());
        for(size_t index = 0; index < contours.size(); index++) {
            defaulttype::Vector2 maxBound = defaulttype::Vector2(-1e100, -1e100);
            defaulttype::Vector2 minBound = defaulttype::Vector2(1e100, 1e100);
            std::vector<cv::Point> & cont = contours[index];
            for( auto &elem : cont) {
                if (elem.x > maxBound[0]) { maxBound[0] = elem.x; }
                if (elem.y > maxBound[1]) { maxBound[1] = elem.y; }
                if (elem.x < minBound[0]) { minBound[0] = elem.x; }
                if (elem.y < minBound[1]) { minBound[1] = elem.y; }
            }
            vMaskPoints[index] = (maxBound + minBound) / 2.0;
        }
    }


    ~MaskImageProcessor() {
        if (_reader_mask.isOpened()) _reader_mask.release();
    }


    /*!
     * \brief properly open mask stream
     */
    inline void openmask() {
        if (_reader_mask.isOpened()) _reader_mask.release();
        if (d_path_mask.getFullPath().empty()) return ;
        _reader_mask = cv::VideoCapture(d_path_mask.getFullPath()) ;
    }

    void handleEvent(sofa::core::objectmodel::Event* event) {
        if(sofa::simulation::AnimateBeginEvent::checkEventType(event)) {
            if (paused) {
            // don't read frame
                return;
            }

            // read next mask image
            maskImageInternal = readmask();

            // inversion to process white objects on black background
            d_maskImage.setValue(opencvplugin::ImageData(maskImageInternal));

            // detect contours
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::Mat greyImage, binaryMask;
            cv::cvtColor(maskImageInternal, greyImage, cv::COLOR_BGR2GRAY);
            cv::threshold(greyImage, binaryMask, 127, 255, cv::THRESH_BINARY_INV);
            //std::string ty = type2str( binaryMask.type() );
            //std::cout << "Matrix: " << ty.c_str() << " " << binaryMask.cols << " " << binaryMask.rows << std::endl;
            cv::findContours(binaryMask, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
            std::cout << "Contours amount: " << contours.size() << std::endl;

            // compute boundboxes for contours and keep their centers
            helper::WriteAccessor<Data<helper::vector<defaulttype::Vector2> >> vMaskPoints = d_out;
            for(size_t index = 0; index < contours.size(); index++) {
                defaulttype::Vector2 maxBound = defaulttype::Vector2(-1e100, -1e100);
                defaulttype::Vector2 minBound = defaulttype::Vector2(1e100, 1e100);
                std::vector<cv::Point> & cont = contours[index];
                for( auto &elem : cont) {
                    if (elem.x > maxBound[0]) { maxBound[0] = elem.x; }
                    if (elem.y > maxBound[1]) { maxBound[1] = elem.y; }
                    if (elem.x < minBound[0]) { minBound[0] = elem.x; }
                    if (elem.y < minBound[1]) { minBound[1] = elem.y; }
                }
                vMaskPoints[index] = (maxBound + minBound) / 2.0;
            }
        }

        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == ' ') {
                paused = ! paused ;
            }
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        defaulttype::Vector3 C = d_corners.getValue().getOrigin();
        defaulttype::Vector3 X = d_corners.getValue().getX();
        defaulttype::Vector3 Y = d_corners.getValue().getY();
        double w = d_currImage.getValue().getImage().cols;
        double h = d_currImage.getValue().getImage().rows;

        /// draw points
        if (f_listening.getValue() && d_draw.getValue()) {
            helper::vector<defaulttype::Vector2> out = d_out.getValue();
            for (auto & pt : out) {
                sofa::defaulttype::Vector3 point = C + X * (pt[0]/w) + Y * (pt[1]/h) ;
                //std::cout << "Output point: " << point << std::endl;
                vparams->drawTool()->drawSphere(point, 0.0038, helper::types::RGBAColor(0, 0, 255, 255));
            }
        }
    }

    /*!
     * \brief readmask
     * \return read mask frame from opened stream
     */
    cv::Mat readmask() {
        cv::Mat out;
        if (!_reader_mask.isOpened()) return out;
        _reader_mask.grab();
        _reader_mask.retrieve(out);
        return out;
    }

    std::string type2str(int type) {
      std::string r;

      uchar depth = type & CV_MAT_DEPTH_MASK;
      uchar chans = 1 + (type >> CV_CN_SHIFT);

      switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
      }

      r += "C";
      r += (chans+'0');

      return r;
    }
};

}  // namespace realsenseplugin

}  // namespace sofa


