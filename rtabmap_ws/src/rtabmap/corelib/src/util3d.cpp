/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

namespace rtabmap
{

namespace util3d
{

cv::Mat bgrFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud, bool bgrOrder)
{
    cv::Mat frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

    for(unsigned int h = 0; h < cloud.height; h++)
    {
        for(unsigned int w = 0; w < cloud.width; w++)
        {
            if(bgrOrder)
            {
                frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
                frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
            }
            else
            {
                frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
                frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
            }
        }
    }
    return frameBGR;
}

// return float image in meter
cv::Mat depthFromCloud(
        const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
        float & fx,
        float & fy,
        bool depth16U)
{
    cv::Mat frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
    fx = 0.0f; // needed to reconstruct the cloud
    fy = 0.0f; // needed to reconstruct the cloud
    for(unsigned int h = 0; h < cloud.height; h++)
    {
        for(unsigned int w = 0; w < cloud.width; w++)
        {
            float depth = cloud.at(h*cloud.width + w).z;
            if(depth16U)
            {
                depth *= 1000.0f;
                unsigned short depthMM = 0;
                if(depth <= (float)USHRT_MAX)
                {
                    depthMM = (unsigned short)depth;
                }
                frameDepth.at<unsigned short>(h,w) = depthMM;
            }
            else
            {
                frameDepth.at<float>(h,w) = depth;
            }

            // update constants
            if(fx == 0.0f &&
               uIsFinite(cloud.at(h*cloud.width + w).x) &&
               uIsFinite(depth) &&
               w != cloud.width/2 &&
               depth > 0)
            {
                fx = cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth);
                if(depth16U)
                {
                    fx*=1000.0f;
                }
            }
            if(fy == 0.0f &&
               uIsFinite(cloud.at(h*cloud.width + w).y) &&
               uIsFinite(depth) &&
               h != cloud.height/2 &&
               depth > 0)
            {
                fy = cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth);
                if(depth16U)
                {
                    fy*=1000.0f;
                }
            }
        }
    }
    return frameDepth;
}

// return (unsigned short 16bits image in mm) (float 32bits image in m)
void rgbdFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
        cv::Mat & frameBGR,
        cv::Mat & frameDepth,
        float & fx,
        float & fy,
        bool bgrOrder,
        bool depth16U)
{
    frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
    frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

    fx = 0.0f; // needed to reconstruct the cloud
    fy = 0.0f; // needed to reconstruct the cloud
    for(unsigned int h = 0; h < cloud.height; h++)
    {
        for(unsigned int w = 0; w < cloud.width; w++)
        {
            //rgb
            if(bgrOrder)
            {
                frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
                frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
            }
            else
            {
                frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
                frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
            }

            //depth
            float depth = cloud.at(h*cloud.width + w).z;
            if(depth16U)
            {
                depth *= 1000.0f;
                unsigned short depthMM = 0;
                if(depth <= (float)USHRT_MAX)
                {
                    depthMM = (unsigned short)depth;
                }
                frameDepth.at<unsigned short>(h,w) = depthMM;
            }
            else
            {
                frameDepth.at<float>(h,w) = depth;
            }

            // update constants
            if(fx == 0.0f &&
               uIsFinite(cloud.at(h*cloud.width + w).x) &&
               uIsFinite(depth) &&
               w != cloud.width/2 &&
               depth > 0)
            {
                fx = 1.0f/(cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth));
                if(depth16U)
                {
                    fx/=1000.0f;
                }
            }
            if(fy == 0.0f &&
               uIsFinite(cloud.at(h*cloud.width + w).y) &&
               uIsFinite(depth) &&
               h != cloud.height/2 &&
               depth > 0)
            {
                fy = 1.0f/(cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth));
                if(depth16U)
                {
                    fy/=1000.0f;
                }
            }
        }
    }
}

Eigen::Vector3f projectDepthTo3DRay(
        const cv::Size & imageSize,
        float x, float y,
        float cx, float cy,
        float fx, float fy)
{
    Eigen::Vector3f ray;

    // Use correct principal point from calibration
    cx = cx > 0.0f ? cx : float(imageSize.width/2) - 0.5f; //cameraInfo.K.at(2)
    cy = cy > 0.0f ? cy : float(imageSize.height/2) - 0.5f; //cameraInfo.K.at(5)

    // Fill in XYZ
    ray[0] = (x - cx) / fx;
    ray[1] = (y - cy) / fy;
    ray[2] = 1.0f;

    return ray;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDisparity(
        const cv::Mat & imageDisparity,
        const StereoCameraModel & model,
        int decimation,
        float maxDepth,
        float minDepth,
        std::vector<int> * validIndices)
{
    UASSERT(imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1);
    UASSERT(decimation >= 1);

    if(imageDisparity.rows % decimation != 0 || imageDisparity.cols % decimation != 0)
    {
        int oldDecimation = decimation;
        while(decimation >= 1)
        {
            if(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0)
            {
                break;
            }
            --decimation;
        }

        if(imageDisparity.rows % oldDecimation != 0 || imageDisparity.cols % oldDecimation != 0)
        {
            UWARN("Decimation (%d) is not valid for current image size (depth=%dx%d). Highest compatible decimation used=%d.", oldDecimation, imageDisparity.cols, imageDisparity.rows, decimation);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //cloud.header = cameraInfo.header;
    cloud->height = imageDisparity.rows/decimation;
    cloud->width  = imageDisparity.cols/decimation;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    if(validIndices)
    {
        validIndices->resize(cloud->size());
    }

    int oi = 0;
    if(imageDisparity.type()==CV_16SC1)
    {
        for(int h = 0; h < imageDisparity.rows && h/decimation < (int)cloud->height; h+=decimation)
        {
            for(int w = 0; w < imageDisparity.cols && w/decimation < (int)cloud->width; w+=decimation)
            {
                float disp = float(imageDisparity.at<short>(h,w))/16.0f;
                cv::Point3f pt = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
                if(pt.z >= minDepth && (maxDepth <= 0.0f || pt.z <= maxDepth))
                {
                    cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(pt.x, pt.y, pt.z);
                    if(validIndices)
                    {
                        validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
                    }
                }
                else
                {
                    cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }
    else
    {
        for(int h = 0; h < imageDisparity.rows && h/decimation < (int)cloud->height; h+=decimation)
        {
            for(int w = 0; w < imageDisparity.cols && w/decimation < (int)cloud->width; w+=decimation)
            {
                float disp = imageDisparity.at<float>(h,w);
                cv::Point3f pt = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
                if(pt.z > minDepth && (maxDepth <= 0.0f || pt.z <= maxDepth))
                {
                    cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(pt.x, pt.y, pt.z);
                    if(validIndices)
                    {
                        validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
                    }
                }
                else
                {
                    cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }
    if(validIndices)
    {
        validIndices->resize(oi);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDisparityRGB(
        const cv::Mat & imageRgb,
        const cv::Mat & imageDisparity,
        const StereoCameraModel & model,
        int decimation,
        float maxDepth,
        float minDepth,
        std::vector<int> * validIndices)
{
    UASSERT(!imageRgb.empty() && !imageDisparity.empty());
    UASSERT(imageRgb.rows == imageDisparity.rows &&
            imageRgb.cols == imageDisparity.cols &&
            (imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1));
    UASSERT(imageRgb.channels() == 3 || imageRgb.channels() == 1);
    UASSERT(decimation >= 1);

    if(imageDisparity.rows % decimation != 0 || imageDisparity.cols % decimation != 0)
    {
        int oldDecimation = decimation;
        while(decimation >= 1)
        {
            if(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0)
            {
                break;
            }
            --decimation;
        }

        if(imageDisparity.rows % oldDecimation != 0 || imageDisparity.cols % oldDecimation != 0)
        {
            UWARN("Decimation (%d) is not valid for current image size (depth=%dx%d). Highest compatible decimation used=%d.", oldDecimation, imageDisparity.cols, imageDisparity.rows, decimation);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    bool mono;
    if(imageRgb.channels() == 3) // BGR
    {
        mono = false;
    }
    else // Mono
    {
        mono = true;
    }

    //cloud.header = cameraInfo.header;
    cloud->height = imageRgb.rows/decimation;
    cloud->width  = imageRgb.cols/decimation;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    if(validIndices)
    {
        validIndices->resize(cloud->size());
    }

    int oi=0;
    for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
    {
        for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
        {
            pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
            if(!mono)
            {
                pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
                pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
                pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
            }
            else
            {
                unsigned char v = imageRgb.at<unsigned char>(h,w);
                pt.b = v;
                pt.g = v;
                pt.r = v;
            }

            float disp = imageDisparity.type()==CV_16SC1?float(imageDisparity.at<short>(h,w))/16.0f:imageDisparity.at<float>(h,w);
            cv::Point3f ptXYZ = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
            if(util3d::isFinite(ptXYZ) && ptXYZ.z >= minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
            {
                pt.x = ptXYZ.x;
                pt.y = ptXYZ.y;
                pt.z = ptXYZ.z;
                if(validIndices)
                {
                    validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
                }
            }
            else
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    if(validIndices)
    {
        validIndices->resize(oi);
    }
    return cloud;
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform, bool filterNaNs)
{
    return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    int oi = 0;
    if(indices.get())
    {
        laserScan = cv::Mat(1, (int)indices->size(), CV_32FC3);
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            int index = indices->at(i);
            if(!filterNaNs || pcl::isFinite(cloud.at(index)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZ pt = pcl::transformPoint(cloud.at(index), transform3f);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                }
                else
                {
                    ptr[0] = cloud.at(index).x;
                    ptr[1] = cloud.at(index).y;
                    ptr[2] = cloud.at(index).z;
                }
            }
        }
    }
    else
    {
        laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC3);
        for(unsigned int i=0; i<cloud.size(); ++i)
        {
            if(!filterNaNs || pcl::isFinite(cloud.at(i)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZ pt = pcl::transformPoint(cloud.at(i), transform3f);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                }
                else
                {
                    ptr[0] = cloud.at(i).x;
                    ptr[1] = cloud.at(i).y;
                    ptr[2] = cloud.at(i).z;
                }
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZ);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform, bool filterNaNs)
{
    return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi=0;
    if(indices.get())
    {
        laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(6));
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            int index = indices->at(i);
            if(!filterNaNs || (pcl::isFinite(cloud.at(index)) &&
                    uIsFinite(cloud.at(index).normal_x) &&
                    uIsFinite(cloud.at(index).normal_y) &&
                    uIsFinite(cloud.at(index).normal_z)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointNormal pt = util3d::transformPoint(cloud.at(index), transform);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                    ptr[3] = pt.normal_x;
                    ptr[4] = pt.normal_y;
                    ptr[5] = pt.normal_z;
                }
                else
                {
                    ptr[0] = cloud.at(index).x;
                    ptr[1] = cloud.at(index).y;
                    ptr[2] = cloud.at(index).z;
                    ptr[3] = cloud.at(index).normal_x;
                    ptr[4] = cloud.at(index).normal_y;
                    ptr[5] = cloud.at(index).normal_z;
                }
            }
        }
    }
    else
    {
        laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(6));
        for(unsigned int i=0; i<cloud.size(); ++i)
        {
            if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
                    uIsFinite(cloud.at(i).normal_x) &&
                    uIsFinite(cloud.at(i).normal_y) &&
                    uIsFinite(cloud.at(i).normal_z)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointNormal pt = util3d::transformPoint(cloud.at(i), transform);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                    ptr[3] = pt.normal_x;
                    ptr[4] = pt.normal_y;
                    ptr[5] = pt.normal_z;
                }
                else
                {
                    ptr[0] = cloud.at(i).x;
                    ptr[1] = cloud.at(i).y;
                    ptr[2] = cloud.at(i).z;
                    ptr[3] = cloud.at(i).normal_x;
                    ptr[4] = cloud.at(i).normal_y;
                    ptr[5] = cloud.at(i).normal_z;
                }
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
    UASSERT(cloud.size() == normals.size());
    cv::Mat laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(6));
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi =0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointNormal pt;
                pt.x = cloud.at(i).x;
                pt.y = cloud.at(i).y;
                pt.z = cloud.at(i).z;
                pt.normal_x = normals.at(i).normal_x;
                pt.normal_y = normals.at(i).normal_y;
                pt.normal_z = normals.at(i).normal_z;
                pt = util3d::transformPoint(pt, transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.z;
                ptr[3] = pt.normal_x;
                ptr[4] = pt.normal_y;
                ptr[5] = pt.normal_z;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
                ptr[2] = cloud.at(i).z;
                ptr[3] = normals.at(i).normal_x;
                ptr[4] = normals.at(i).normal_y;
                ptr[5] = normals.at(i).normal_z;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const Transform & transform, bool filterNaNs)
{
    return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    int oi=0;
    if(indices.get())
    {
        laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(4));
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            int index = indices->at(i);
            if(!filterNaNs || pcl::isFinite(cloud.at(index)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZRGB pt = pcl::transformPoint(cloud.at(index), transform3f);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                }
                else
                {
                    ptr[0] = cloud.at(index).x;
                    ptr[1] = cloud.at(index).y;
                    ptr[2] = cloud.at(index).z;
                }
                int * ptrInt = (int*)ptr;
                ptrInt[3] = int(cloud.at(index).b) | (int(cloud.at(index).g) << 8) | (int(cloud.at(index).r) << 16);
            }
        }
    }
    else
    {
        laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(4));
        for(unsigned int i=0; i<cloud.size(); ++i)
        {
            if(!filterNaNs || pcl::isFinite(cloud.at(i)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZRGB pt = pcl::transformPoint(cloud.at(i), transform3f);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                }
                else
                {
                    ptr[0] = cloud.at(i).x;
                    ptr[1] = cloud.at(i).y;
                    ptr[2] = cloud.at(i).z;
                }
                int * ptrInt = (int*)ptr;
                ptrInt[3] = int(cloud.at(i).b) | (int(cloud.at(i).g) << 8) | (int(cloud.at(i).r) << 16);
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGB);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform, bool filterNaNs)
{
    return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    int oi=0;
    if(indices.get())
    {
        laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(4));
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            int index = indices->at(i);
            if(!filterNaNs || pcl::isFinite(cloud.at(index)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZI pt = pcl::transformPoint(cloud.at(index), transform3f);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                }
                else
                {
                    ptr[0] = cloud.at(index).x;
                    ptr[1] = cloud.at(index).y;
                    ptr[2] = cloud.at(index).z;
                }
                ptr[3] = cloud.at(index).intensity;
            }
        }
    }
    else
    {
        laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(4));
        for(unsigned int i=0; i<cloud.size(); ++i)
        {
            if(!filterNaNs || pcl::isFinite(cloud.at(i)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZI pt = pcl::transformPoint(cloud.at(i), transform3f);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                }
                else
                {
                    ptr[0] = cloud.at(i).x;
                    ptr[1] = cloud.at(i).y;
                    ptr[2] = cloud.at(i).z;
                }
                ptr[3] = cloud.at(i).intensity;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZI);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
    UASSERT(cloud.size() == normals.size());
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(7));
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi = 0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || pcl::isFinite(cloud.at(i)))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointXYZRGBNormal pt;
                pt.x = cloud.at(i).x;
                pt.y = cloud.at(i).y;
                pt.z = cloud.at(i).z;
                pt.normal_x = normals.at(i).normal_x;
                pt.normal_y = normals.at(i).normal_y;
                pt.normal_z = normals.at(i).normal_z;
                pt = util3d::transformPoint(pt, transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.z;
                ptr[4] = pt.normal_x;
                ptr[5] = pt.normal_y;
                ptr[6] = pt.normal_z;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
                ptr[2] = cloud.at(i).z;
                ptr[4] = normals.at(i).normal_x;
                ptr[5] = normals.at(i).normal_y;
                ptr[6] = normals.at(i).normal_z;
            }
            int * ptrInt = (int*)ptr;
            ptrInt[3] = int(cloud.at(i).b) | (int(cloud.at(i).g) << 8) | (int(cloud.at(i).r) << 16);
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGBNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const Transform & transform, bool filterNaNs)
{
    return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi = 0;
    if(indices.get())
    {
        laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(7));
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            int index = indices->at(i);
            if(!filterNaNs || (pcl::isFinite(cloud.at(index)) &&
                    uIsFinite(cloud.at(index).normal_x) &&
                    uIsFinite(cloud.at(index).normal_y) &&
                    uIsFinite(cloud.at(index).normal_z)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZRGBNormal pt = util3d::transformPoint(cloud.at(index), transform);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                    ptr[4] = pt.normal_x;
                    ptr[5] = pt.normal_y;
                    ptr[6] = pt.normal_z;
                }
                else
                {
                    ptr[0] = cloud.at(index).x;
                    ptr[1] = cloud.at(index).y;
                    ptr[2] = cloud.at(index).z;
                    ptr[4] = cloud.at(index).normal_x;
                    ptr[5] = cloud.at(index).normal_y;
                    ptr[6] = cloud.at(index).normal_z;
                }
                int * ptrInt = (int*)ptr;
                ptrInt[3] = int(cloud.at(index).b) | (int(cloud.at(index).g) << 8) | (int(cloud.at(index).r) << 16);
            }
        }
    }
    else
    {
        laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(7));
        for(unsigned int i=0; i<cloud.size(); ++i)
        {
            if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
                    uIsFinite(cloud.at(i).normal_x) &&
                    uIsFinite(cloud.at(i).normal_y) &&
                    uIsFinite(cloud.at(i).normal_z)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZRGBNormal pt = util3d::transformPoint(cloud.at(i), transform);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                    ptr[4] = pt.normal_x;
                    ptr[5] = pt.normal_y;
                    ptr[6] = pt.normal_z;
                }
                else
                {
                    ptr[0] = cloud.at(i).x;
                    ptr[1] = cloud.at(i).y;
                    ptr[2] = cloud.at(i).z;
                    ptr[4] = cloud.at(i).normal_x;
                    ptr[5] = cloud.at(i).normal_y;
                    ptr[6] = cloud.at(i).normal_z;
                }
                int * ptrInt = (int*)ptr;
                ptrInt[3] = int(cloud.at(i).b) | (int(cloud.at(i).g) << 8) | (int(cloud.at(i).r) << 16);
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGBNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
    UASSERT(cloud.size() == normals.size());
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(7));
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointXYZINormal pt;
                pt.x = cloud.at(i).x;
                pt.y = cloud.at(i).y;
                pt.z = cloud.at(i).z;
                pt.normal_x = normals.at(i).normal_x;
                pt.normal_y = normals.at(i).normal_y;
                pt.normal_z = normals.at(i).normal_z;
                pt = util3d::transformPoint(pt, transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.z;
                ptr[4] = pt.normal_x;
                ptr[5] = pt.normal_y;
                ptr[6] = pt.normal_z;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
                ptr[2] = cloud.at(i).z;
                ptr[4] = normals.at(i).normal_x;
                ptr[5] = normals.at(i).normal_y;
                ptr[6] = normals.at(i).normal_z;
            }
            ptr[3] = cloud.at(i).intensity;
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZINormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform, bool filterNaNs)
{
    return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi = 0;
    if(indices.get())
    {
        laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(7));
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            int index = indices->at(i);
            if(!filterNaNs || (pcl::isFinite(cloud.at(index)) &&
                    uIsFinite(cloud.at(index).normal_x) &&
                    uIsFinite(cloud.at(index).normal_y) &&
                    uIsFinite(cloud.at(index).normal_z)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZINormal pt = util3d::transformPoint(cloud.at(index), transform);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                    ptr[4] = pt.normal_x;
                    ptr[5] = pt.normal_y;
                    ptr[6] = pt.normal_z;
                }
                else
                {
                    ptr[0] = cloud.at(index).x;
                    ptr[1] = cloud.at(index).y;
                    ptr[2] = cloud.at(index).z;
                    ptr[4] = cloud.at(index).normal_x;
                    ptr[5] = cloud.at(index).normal_y;
                    ptr[6] = cloud.at(index).normal_z;
                }
                ptr[3] = cloud.at(i).intensity;
            }
        }
    }
    else
    {
        laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(7));
        for(unsigned int i=0; i<cloud.size(); ++i)
        {
            if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
                    uIsFinite(cloud.at(i).normal_x) &&
                    uIsFinite(cloud.at(i).normal_y) &&
                    uIsFinite(cloud.at(i).normal_z)))
            {
                float * ptr = laserScan.ptr<float>(0, oi++);
                if(!nullTransform)
                {
                    pcl::PointXYZINormal pt = util3d::transformPoint(cloud.at(i), transform);
                    ptr[0] = pt.x;
                    ptr[1] = pt.y;
                    ptr[2] = pt.z;
                    ptr[4] = pt.normal_x;
                    ptr[5] = pt.normal_y;
                    ptr[6] = pt.normal_z;
                }
                else
                {
                    ptr[0] = cloud.at(i).x;
                    ptr[1] = cloud.at(i).y;
                    ptr[2] = cloud.at(i).z;
                    ptr[4] = cloud.at(i).normal_x;
                    ptr[5] = cloud.at(i).normal_y;
                    ptr[6] = cloud.at(i).normal_z;
                }
                ptr[3] = cloud.at(i).intensity;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZINormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC2);
    bool nullTransform = transform.isNull();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || pcl::isFinite(cloud.at(i)))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointXYZ pt = pcl::transformPoint(cloud.at(i), transform3f);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
            }
        }

    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXY);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC3);
    bool nullTransform = transform.isNull();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || pcl::isFinite(cloud.at(i)))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointXYZI pt = pcl::transformPoint(cloud.at(i), transform3f);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.intensity;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
                ptr[2] = cloud.at(i).intensity;
            }
        }

    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYI);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(5));
    bool nullTransform = transform.isNull();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
                uIsFinite(cloud.at(i).normal_x) &&
                uIsFinite(cloud.at(i).normal_y) &&
                uIsFinite(cloud.at(i).normal_z)))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointNormal pt = util3d::transformPoint(cloud.at(i), transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.normal_x;
                ptr[3] = pt.normal_y;
                ptr[4] = pt.normal_z;
            }
            else
            {
                const pcl::PointNormal & pt = cloud.at(i);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.normal_x;
                ptr[3] = pt.normal_y;
                ptr[4] = pt.normal_z;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYNormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
    UASSERT(cloud.size() == normals.size());
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(5));
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointNormal pt;
                pt.x = cloud.at(i).x;
                pt.y = cloud.at(i).y;
                pt.z = cloud.at(i).z;
                pt.normal_x = normals.at(i).normal_x;
                pt.normal_y = normals.at(i).normal_y;
                pt.normal_z = normals.at(i).normal_z;
                pt = util3d::transformPoint(pt, transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.normal_x;
                ptr[3] = pt.normal_y;
                ptr[4] = pt.normal_z;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
                ptr[2] = normals.at(i).normal_x;
                ptr[3] = normals.at(i).normal_y;
                ptr[4] = normals.at(i).normal_z;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYNormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform, bool filterNaNs)
{
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(6));
    bool nullTransform = transform.isNull();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
                uIsFinite(cloud.at(i).normal_x) &&
                uIsFinite(cloud.at(i).normal_y) &&
                uIsFinite(cloud.at(i).normal_z)))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointXYZINormal pt = util3d::transformPoint(cloud.at(i), transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.intensity;
                ptr[3] = pt.normal_x;
                ptr[4] = pt.normal_y;
                ptr[5] = pt.normal_z;
            }
            else
            {
                const pcl::PointXYZINormal & pt = cloud.at(i);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.intensity;
                ptr[3] = pt.normal_x;
                ptr[4] = pt.normal_y;
                ptr[5] = pt.normal_z;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYINormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
    UASSERT(cloud.size() == normals.size());
    cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(6));
    bool nullTransform = transform.isNull() || transform.isIdentity();
    int oi=0;
    for(unsigned int i=0; i<cloud.size(); ++i)
    {
        if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
        {
            float * ptr = laserScan.ptr<float>(0, oi++);
            if(!nullTransform)
            {
                pcl::PointXYZINormal pt;
                pt.x = cloud.at(i).x;
                pt.y = cloud.at(i).y;
                pt.z = cloud.at(i).z;
                pt.normal_x = normals.at(i).normal_x;
                pt.normal_y = normals.at(i).normal_y;
                pt.normal_z = normals.at(i).normal_z;
                pt = util3d::transformPoint(pt, transform);
                ptr[0] = pt.x;
                ptr[1] = pt.y;
                ptr[2] = pt.intensity;
                ptr[3] = pt.normal_x;
                ptr[4] = pt.normal_y;
                ptr[5] = pt.normal_z;
            }
            else
            {
                ptr[0] = cloud.at(i).x;
                ptr[1] = cloud.at(i).y;
                ptr[2] = cloud.at(i).intensity;
                ptr[3] = normals.at(i).normal_x;
                ptr[4] = normals.at(i).normal_y;
                ptr[5] = normals.at(i).normal_z;
            }
        }
    }
    if(oi == 0)
    {
        return LaserScan();
    }
    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYINormal);
}

pcl::PCLPointCloud2::Ptr laserScanToPointCloud2(const LaserScan & laserScan, const Transform & transform)
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    if(laserScan.isEmpty())
    {
        return cloud;
    }

    if(laserScan.format() == LaserScan::kXY || laserScan.format() == LaserScan::kXYZ)
    {
        pcl::toPCLPointCloud2(*laserScanToPointCloud(laserScan, transform), *cloud);
    }
    else if(laserScan.format() == LaserScan::kXYI || laserScan.format() == LaserScan::kXYZI)
    {
        pcl::toPCLPointCloud2(*laserScanToPointCloudI(laserScan, transform), *cloud);
    }
    else if(laserScan.format() == LaserScan::kXYNormal || laserScan.format() == LaserScan::kXYZNormal)
    {
        pcl::toPCLPointCloud2(*laserScanToPointCloudNormal(laserScan, transform), *cloud);
    }
    else if(laserScan.format() == LaserScan::kXYINormal || laserScan.format() == LaserScan::kXYZINormal)
    {
        pcl::toPCLPointCloud2(*laserScanToPointCloudINormal(laserScan, transform), *cloud);
    }
    else if(laserScan.format() == LaserScan::kXYZRGB)
    {
        pcl::toPCLPointCloud2(*laserScanToPointCloudRGB(laserScan, transform), *cloud);
    }
    else if(laserScan.format() == LaserScan::kXYZRGBNormal)
    {
        pcl::toPCLPointCloud2(*laserScanToPointCloudRGBNormal(laserScan, transform), *cloud);
    }
    else
    {
        UERROR("Unknown conversion from LaserScan format %d to PointCloud2.", laserScan.format());
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const LaserScan & laserScan, const Transform & transform)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    output->resize(laserScan.size());
    output->is_dense = true;
    bool nullTransform = transform.isNull();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    for(int i=0; i<laserScan.size(); ++i)
    {
        output->at(i) = util3d::laserScanToPoint(laserScan, i);
        if(!nullTransform)
        {
            output->at(i) = pcl::transformPoint(output->at(i), transform3f);
        }
    }
    return output;
}

pcl::PointCloud<pcl::PointNormal>::Ptr laserScanToPointCloudNormal(const LaserScan & laserScan, const Transform & transform)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
    output->resize(laserScan.size());
    output->is_dense = true;
    bool nullTransform = transform.isNull();
    for(int i=0; i<laserScan.size(); ++i)
    {
        output->at(i) = laserScanToPointNormal(laserScan, i);
        if(!nullTransform)
        {
            output->at(i) = util3d::transformPoint(output->at(i), transform);
        }
    }
    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserScanToPointCloudRGB(const LaserScan & laserScan, const Transform & transform,  unsigned char r, unsigned char g, unsigned char b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    output->resize(laserScan.size());
    output->is_dense = true;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    for(int i=0; i<laserScan.size(); ++i)
    {
        output->at(i) = util3d::laserScanToPointRGB(laserScan, i, r, g, b);
        if(!nullTransform)
        {
            output->at(i) = pcl::transformPoint(output->at(i), transform3f);
        }
    }
    return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr laserScanToPointCloudI(const LaserScan & laserScan, const Transform & transform,  float intensity)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    output->resize(laserScan.size());
    output->is_dense = true;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    Eigen::Affine3f transform3f = transform.toEigen3f();
    for(int i=0; i<laserScan.size(); ++i)
    {
        output->at(i) = util3d::laserScanToPointI(laserScan, i, intensity);
        if(!nullTransform)
        {
            output->at(i) = pcl::transformPoint(output->at(i), transform3f);
        }
    }
    return output;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr laserScanToPointCloudRGBNormal(const LaserScan & laserScan, const Transform & transform,  unsigned char r, unsigned char g, unsigned char b)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    output->resize(laserScan.size());
    output->is_dense = true;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    for(int i=0; i<laserScan.size(); ++i)
    {
        output->at(i) = util3d::laserScanToPointRGBNormal(laserScan, i, r, g, b);
        if(!nullTransform)
        {
            output->at(i) = util3d::transformPoint(output->at(i), transform);
        }
    }
    return output;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserScanToPointCloudINormal(const LaserScan & laserScan, const Transform & transform,  float intensity)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
    output->resize(laserScan.size());
    output->is_dense = true;
    bool nullTransform = transform.isNull() || transform.isIdentity();
    for(int i=0; i<laserScan.size(); ++i)
    {
        output->at(i) = util3d::laserScanToPointINormal(laserScan, i, intensity);
        if(!nullTransform)
        {
            output->at(i) = util3d::transformPoint(output->at(i), transform);
        }
    }
    return output;
}

pcl::PointXYZ laserScanToPoint(const LaserScan & laserScan, int index)
{
    UASSERT(!laserScan.isEmpty() && index < laserScan.size());
    pcl::PointXYZ output;
    const float * ptr = laserScan.data().ptr<float>(0, index);
    output.x = ptr[0];
    output.y = ptr[1];
    if(!laserScan.is2d())
    {
        output.z = ptr[2];
    }
    return output;
}

pcl::PointNormal laserScanToPointNormal(const LaserScan & laserScan, int index)
{
    UASSERT(!laserScan.isEmpty() && index < laserScan.size());
    pcl::PointNormal output;
    const float * ptr = laserScan.data().ptr<float>(0, index);
    output.x = ptr[0];
    output.y = ptr[1];
    if(!laserScan.is2d())
    {
        output.z = ptr[2];
    }
    if(laserScan.hasNormals())
    {
        int offset = laserScan.getNormalsOffset();
        output.normal_x = ptr[offset];
        output.normal_y = ptr[offset+1];
        output.normal_z = ptr[offset+2];
    }
    return output;
}

pcl::PointXYZRGB laserScanToPointRGB(const LaserScan & laserScan, int index, unsigned char r, unsigned char g, unsigned char b)
{
    UASSERT(!laserScan.isEmpty() && index < laserScan.size());
    pcl::PointXYZRGB output;
    const float * ptr = laserScan.data().ptr<float>(0, index);
    output.x = ptr[0];
    output.y = ptr[1];
    if(!laserScan.is2d())
    {
        output.z = ptr[2];
    }

    if(laserScan.hasRGB())
    {
        std::uint32_t * ptrInt = (std::uint32_t*)ptr;
        int indexRGB = laserScan.getRGBOffset();
        output.rgba = ptrInt[indexRGB];
    }
    else if(laserScan.hasIntensity())
    {
        // package intensity float -> rgba
        int * ptrInt = (int*)ptr;
        int indexIntensity = laserScan.getIntensityOffset();
        output.r = (unsigned char)(ptrInt[indexIntensity] & 0xFF);
        output.g = (unsigned char)((ptrInt[indexIntensity] >> 8) & 0xFF);
        output.b = (unsigned char)((ptrInt[indexIntensity] >> 16) & 0xFF);
        output.a = (unsigned char)((ptrInt[indexIntensity] >> 24) & 0xFF);
    }
    else
    {
        output.r = r;
        output.g = g;
        output.b = b;
    }
    return output;
}

pcl::PointXYZI laserScanToPointI(const LaserScan & laserScan, int index, float intensity)
{
    UASSERT(!laserScan.isEmpty() && index < laserScan.size());
    pcl::PointXYZI output;
    const float * ptr = laserScan.data().ptr<float>(0, index);
    output.x = ptr[0];
    output.y = ptr[1];
    if(!laserScan.is2d())
    {
        output.z = ptr[2];
    }

    if(laserScan.hasIntensity())
    {
        int offset = laserScan.getIntensityOffset();
        output.intensity = ptr[offset];
    }
    else
    {
        output.intensity = intensity;
    }

    return output;
}

pcl::PointXYZRGBNormal laserScanToPointRGBNormal(const LaserScan & laserScan, int index, unsigned char r, unsigned char g, unsigned char b)
{
    UASSERT(!laserScan.isEmpty() && index < laserScan.size());
    pcl::PointXYZRGBNormal output;
    const float * ptr = laserScan.data().ptr<float>(0, index);
    output.x = ptr[0];
    output.y = ptr[1];
    if(!laserScan.is2d())
    {
        output.z = ptr[2];
    }

    if(laserScan.hasRGB())
    {
        int * ptrInt = (int*)ptr;
        int indexRGB = laserScan.getRGBOffset();
        output.b = (unsigned char)(ptrInt[indexRGB] & 0xFF);
        output.g = (unsigned char)((ptrInt[indexRGB] >> 8) & 0xFF);
        output.r = (unsigned char)((ptrInt[indexRGB] >> 16) & 0xFF);
    }
    else if(laserScan.hasIntensity())
    {
        int * ptrInt = (int*)ptr;
        int indexIntensity = laserScan.getIntensityOffset();
        output.r = (unsigned char)(ptrInt[indexIntensity] & 0xFF);
        output.g = (unsigned char)((ptrInt[indexIntensity] >> 8) & 0xFF);
        output.b = (unsigned char)((ptrInt[indexIntensity] >> 16) & 0xFF);
        output.a = (unsigned char)((ptrInt[indexIntensity] >> 24) & 0xFF);
    }
    else
    {
        output.r = r;
        output.g = g;
        output.b = b;
    }

    if(laserScan.hasNormals())
    {
        int offset = laserScan.getNormalsOffset();
        output.normal_x = ptr[offset];
        output.normal_y = ptr[offset+1];
        output.normal_z = ptr[offset+2];
    }

    return output;
}

pcl::PointXYZINormal laserScanToPointINormal(const LaserScan & laserScan, int index, float intensity)
{
    UASSERT(!laserScan.isEmpty() && index < laserScan.size());
    pcl::PointXYZINormal output;
    const float * ptr = laserScan.data().ptr<float>(0, index);
    output.x = ptr[0];
    output.y = ptr[1];
    if(!laserScan.is2d())
    {
        output.z = ptr[2];
    }

    if(laserScan.hasIntensity())
    {
        int offset = laserScan.getIntensityOffset();
        output.intensity = ptr[offset];
    }
    else
    {
        output.intensity = intensity;
    }

    if(laserScan.hasNormals())
    {
        int offset = laserScan.getNormalsOffset();
        output.normal_x = ptr[offset];
        output.normal_y = ptr[offset+1];
        output.normal_z = ptr[offset+2];
    }

    return output;
}

void getMinMax3D(const cv::Mat & laserScan, cv::Point3f & min, cv::Point3f & max)
{
    UASSERT(!laserScan.empty());
    UASSERT(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(4) || laserScan.type() == CV_32FC(5) || laserScan.type() == CV_32FC(6) || laserScan.type() == CV_32FC(7));

    const float * ptr = laserScan.ptr<float>(0, 0);
    min.x = max.x = ptr[0];
    min.y = max.y = ptr[1];
    bool is3d = laserScan.channels() >= 3 && laserScan.channels() != 5;
    min.z = max.z = is3d?ptr[2]:0.0f;
    for(int i=1; i<laserScan.cols; ++i)
    {
        ptr = laserScan.ptr<float>(0, i);

        if(ptr[0] < min.x) min.x = ptr[0];
        else if(ptr[0] > max.x) max.x = ptr[0];

        if(ptr[1] < min.y) min.y = ptr[1];
        else if(ptr[1] > max.y) max.y = ptr[1];

        if(is3d)
        {
            if(ptr[2] < min.z) min.z = ptr[2];
            else if(ptr[2] > max.z) max.z = ptr[2];
        }
    }
}
void getMinMax3D(const cv::Mat & laserScan, pcl::PointXYZ & min, pcl::PointXYZ & max)
{
    cv::Point3f minCV, maxCV;
    getMinMax3D(laserScan, minCV, maxCV);
    min.x = minCV.x;
    min.y = minCV.y;
    min.z = minCV.z;
    max.x = maxCV.x;
    max.y = maxCV.y;
    max.z = maxCV.z;
}

// inspired from ROS image_geometry/src/stereo_camera_model.cpp
cv::Point3f projectDisparityTo3D(
        const cv::Point2f & pt,
        float disparity,
        const StereoCameraModel & model)
{
    if(disparity > 0.0f && model.baseline() > 0.0f && model.left().fx() > 0.0f)
    {
        //Z = baseline * f / (d + cx1-cx0);
        float c = 0.0f;
        if(model.right().cx()>0.0f && model.left().cx()>0.0f)
        {
            c = model.right().cx() - model.left().cx();
        }
        float W = model.baseline()/(disparity + c);
        return cv::Point3f((pt.x - model.left().cx())*W, (pt.y - model.left().cy())*W, model.left().fx()*W);
    }
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    return cv::Point3f(bad_point, bad_point, bad_point);
}

cv::Point3f projectDisparityTo3D(
        const cv::Point2f & pt,
        const cv::Mat & disparity,
        const StereoCameraModel & model)
{
    UASSERT(!disparity.empty() && (disparity.type() == CV_32FC1 || disparity.type() == CV_16SC1));
    int u = int(pt.x+0.5f);
    int v = int(pt.y+0.5f);
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    if(uIsInBounds(u, 0, disparity.cols) &&
       uIsInBounds(v, 0, disparity.rows))
    {
        float d = disparity.type() == CV_16SC1?float(disparity.at<short>(v,u))/16.0f:disparity.at<float>(v,u);
        return projectDisparityTo3D(pt, d, model);
    }
    return cv::Point3f(bad_point, bad_point, bad_point);
}

// Register point cloud to camera (return registered depth image)
cv::Mat projectCloudToCamera(
        const cv::Size & imageSize,
        const cv::Mat & cameraMatrixK,
        const cv::Mat & laserScan,     // assuming laser scan points are already in /base_link coordinate
        const rtabmap::Transform & cameraTransform) // /base_link -> /camera_link
{
    UASSERT(!cameraTransform.isNull());
    UASSERT(!laserScan.empty());
    UASSERT(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(4) || laserScan.type() == CV_32FC(5) || laserScan.type() == CV_32FC(6) || laserScan.type() == CV_32FC(7));
    UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);

    float fx = cameraMatrixK.at<double>(0,0);
    float fy = cameraMatrixK.at<double>(1,1);
    float cx = cameraMatrixK.at<double>(0,2);
    float cy = cameraMatrixK.at<double>(1,2);

    cv::Mat registered = cv::Mat::zeros(imageSize, CV_32FC1);
    Transform t = cameraTransform.inverse();

    int count = 0;
    for(int i=0; i<laserScan.cols; ++i)
    {
        const float* ptr = laserScan.ptr<float>(0, i);

        // Get 3D from laser scan
        cv::Point3f ptScan;
        if(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC(5))
        {
            // 2D scans
            ptScan.x = ptr[0];
            ptScan.y = ptr[1];
            ptScan.z = 0;
        }
        else // 3D scans
        {
            ptScan.x = ptr[0];
            ptScan.y = ptr[1];
            ptScan.z = ptr[2];
        }
        ptScan = util3d::transformPoint(ptScan, t);

        // re-project in camera frame
        float z = ptScan.z;

        bool set = false;
        if(z > 0.0f)
        {
            float invZ = 1.0f/z;
            float dx = (fx*ptScan.x)*invZ + cx;
            float dy = (fy*ptScan.y)*invZ + cy;
            int dx_low = dx;
            int dy_low = dy;
            int dx_high = dx + 0.5f;
            int dy_high = dy + 0.5f;

            if(uIsInBounds(dx_low, 0, registered.cols) && uIsInBounds(dy_low, 0, registered.rows))
            {
                float &zReg = registered.at<float>(dy_low, dx_low);
                if(zReg == 0 || z < zReg)
                {
                    zReg = z;
                }
                set = true;
            }
            if((dx_low != dx_high || dy_low != dy_high) &&
                uIsInBounds(dx_high, 0, registered.cols) && uIsInBounds(dy_high, 0, registered.rows))
            {
                float &zReg = registered.at<float>(dy_high, dx_high);
                if(zReg == 0 || z < zReg)
                {
                    zReg = z;
                }
                set = true;
            }
        }
        if(set)
        {
            count++;
        }
    }
    UDEBUG("Points in camera=%d/%d", count, laserScan.cols);

    return registered;
}

cv::Mat projectCloudToCamera(
        const cv::Size & imageSize,
        const cv::Mat & cameraMatrixK,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan,  // assuming points are already in /base_link coordinate
        const rtabmap::Transform & cameraTransform)           // /base_link -> /camera_link
{
    UASSERT(!cameraTransform.isNull());
    UASSERT(!laserScan->empty());
    UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);

    float fx = cameraMatrixK.at<double>(0,0);
    float fy = cameraMatrixK.at<double>(1,1);
    float cx = cameraMatrixK.at<double>(0,2);
    float cy = cameraMatrixK.at<double>(1,2);

    cv::Mat registered = cv::Mat::zeros(imageSize, CV_32FC1);
    Transform t = cameraTransform.inverse();

    int count = 0;
    for(int i=0; i<(int)laserScan->size(); ++i)
    {
        // Get 3D from laser scan
        pcl::PointXYZ ptScan = laserScan->at(i);
        ptScan = util3d::transformPoint(ptScan, t);

        // re-project in camera frame
        float z = ptScan.z;
        bool set = false;
        if(z > 0.0f)
        {
            float invZ = 1.0f/z;
            float dx = (fx*ptScan.x)*invZ + cx;
            float dy = (fy*ptScan.y)*invZ + cy;
            int dx_low = dx;
            int dy_low = dy;
            int dx_high = dx + 0.5f;
            int dy_high = dy + 0.5f;
            if(uIsInBounds(dx_low, 0, registered.cols) && uIsInBounds(dy_low, 0, registered.rows))
            {
                set = true;
                float &zReg = registered.at<float>(dy_low, dx_low);
                if(zReg == 0 || z < zReg)
                {
                    zReg = z;
                }
            }
            if((dx_low != dx_high || dy_low != dy_high) &&
                uIsInBounds(dx_high, 0, registered.cols) && uIsInBounds(dy_high, 0, registered.rows))
            {
                set = true;
                float &zReg = registered.at<float>(dy_high, dx_high);
                if(zReg == 0 || z < zReg)
                {
                    zReg = z;
                }
            }
        }
        if(set)
        {
            count++;
        }
    }
    UDEBUG("Points in camera=%d/%d", count, (int)laserScan->size());

    return registered;
}

cv::Mat projectCloudToCamera(
        const cv::Size & imageSize,
        const cv::Mat & cameraMatrixK,
        const pcl::PCLPointCloud2::Ptr laserScan,  // assuming points are already in /base_link coordinate
        const rtabmap::Transform & cameraTransform)           // /base_link -> /camera_link
{
    UASSERT(!cameraTransform.isNull());
    UASSERT(!laserScan->data.empty());
    UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);

    float fx = cameraMatrixK.at<double>(0,0);
    float fy = cameraMatrixK.at<double>(1,1);
    float cx = cameraMatrixK.at<double>(0,2);
    float cy = cameraMatrixK.at<double>(1,2);

    cv::Mat registered = cv::Mat::zeros(imageSize, CV_32FC1);
    Transform t = cameraTransform.inverse();

    pcl::MsgFieldMap field_map;
    pcl::createMapping<pcl::PointXYZ> (laserScan->fields, field_map);

    int count = 0;
    if(field_map.size() == 1)
    {
        for (uint32_t row = 0; row < (uint32_t)laserScan->height; ++row)
        {
            const uint8_t* row_data = &laserScan->data[row * laserScan->row_step];
            for (uint32_t col = 0; col < (uint32_t)laserScan->width; ++col)
            {
                const uint8_t* msg_data = row_data + col * laserScan->point_step;
                pcl::PointXYZ ptScan;
                memcpy (&ptScan, msg_data + field_map.front().serialized_offset, field_map.front().size);
                ptScan = util3d::transformPoint(ptScan, t);

                // re-project in camera frame
                float z = ptScan.z;
                bool set = false;
                if(z > 0.0f)
                {
                    float invZ = 1.0f/z;
                    float dx = (fx*ptScan.x)*invZ + cx;
                    float dy = (fy*ptScan.y)*invZ + cy;
                    int dx_low = dx;
                    int dy_low = dy;
                    int dx_high = dx + 0.5f;
                    int dy_high = dy + 0.5f;
                    if(uIsInBounds(dx_low, 0, registered.cols) && uIsInBounds(dy_low, 0, registered.rows))
                    {
                        set = true;
                        float &zReg = registered.at<float>(dy_low, dx_low);
                        if(zReg == 0 || z < zReg)
                        {
                            zReg = z;
                        }
                    }
                    if((dx_low != dx_high || dy_low != dy_high) &&
                        uIsInBounds(dx_high, 0, registered.cols) && uIsInBounds(dy_high, 0, registered.rows))
                    {
                        set = true;
                        float &zReg = registered.at<float>(dy_high, dx_high);
                        if(zReg == 0 || z < zReg)
                        {
                            zReg = z;
                        }
                    }
                }
                if(set)
                {
                    count++;
                }
            }
        }
    }
    else
    {
        UERROR("field map pcl::pointXYZ not found!");
    }
    UDEBUG("Points in camera=%d/%d", count, (int)laserScan->data.size());

    return registered;
}

void fillProjectedCloudHoles(cv::Mat & registeredDepth, bool verticalDirection, bool fillToBorder)
{
    UASSERT(registeredDepth.type() == CV_32FC1);
    if(verticalDirection)
    {
        // vertical, for each column
        for(int x=0; x<registeredDepth.cols; ++x)
        {
            float valueA = 0.0f;
            int indexA = -1;
            for(int y=0; y<registeredDepth.rows; ++y)
            {
                float v = registeredDepth.at<float>(y,x);
                if(fillToBorder && y == registeredDepth.rows-1 && v<=0.0f && indexA>=0)
                {
                    v = valueA;
                }
                if(v > 0.0f)
                {
                    if(fillToBorder && indexA < 0)
                    {
                        indexA = 0;
                        valueA = v;
                    }
                    if(indexA >=0)
                    {
                        int range = y-indexA;
                        if(range > 1)
                        {
                            float slope = (v-valueA)/(range);
                            for(int k=1; k<range; ++k)
                            {
                                registeredDepth.at<float>(indexA+k,x) = valueA+slope*float(k);
                            }
                        }
                    }
                    valueA = v;
                    indexA = y;
                }
            }
        }
    }
    else
    {
        // horizontal, for each row
        for(int y=0; y<registeredDepth.rows; ++y)
        {
            float valueA = 0.0f;
            int indexA = -1;
            for(int x=0; x<registeredDepth.cols; ++x)
            {
                float v = registeredDepth.at<float>(y,x);
                if(fillToBorder && x == registeredDepth.cols-1 && v<=0.0f && indexA>=0)
                {
                    v = valueA;
                }
                if(v > 0.0f)
                {
                    if(fillToBorder && indexA < 0)
                    {
                        indexA = 0;
                        valueA = v;
                    }
                    if(indexA >=0)
                    {
                        int range = x-indexA;
                        if(range > 1)
                        {
                            float slope = (v-valueA)/(range);
                            for(int k=1; k<range; ++k)
                            {
                                registeredDepth.at<float>(y,indexA+k) = valueA+slope*float(k);
                            }
                        }
                    }
                    valueA = v;
                    indexA = x;
                }
            }
        }
    }
}

struct ProjectionInfo {
    int nodeID;
    int cameraIndex;
    pcl::PointXY uv;
    float distance;
};

bool isFinite(const cv::Point3f & pt)
{
    return uIsFinite(pt.x) && uIsFinite(pt.y) && uIsFinite(pt.z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
    {
        *cloud += *(*iter);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
    {
        *cloud+=*(*iter);
    }
    return cloud;
}

pcl::IndicesPtr concatenate(const std::vector<pcl::IndicesPtr> & indices)
{
    //compute total size
    unsigned int totalSize = 0;
    for(unsigned int i=0; i<indices.size(); ++i)
    {
        totalSize += (unsigned int)indices[i]->size();
    }
    pcl::IndicesPtr ind(new std::vector<int>(totalSize));
    unsigned int io = 0;
    for(unsigned int i=0; i<indices.size(); ++i)
    {
        for(unsigned int j=0; j<indices[i]->size(); ++j)
        {
            ind->at(io++) = indices[i]->at(j);
        }
    }
    return ind;
}

pcl::IndicesPtr concatenate(const pcl::IndicesPtr & indicesA, const pcl::IndicesPtr & indicesB)
{
    pcl::IndicesPtr ind(new std::vector<int>(*indicesA));
    ind->resize(ind->size()+indicesB->size());
    unsigned int oi = (unsigned int)indicesA->size();
    for(unsigned int i=0; i<indicesB->size(); ++i)
    {
        ind->at(oi++) = indicesB->at(i);
    }
    return ind;
}

}

}
