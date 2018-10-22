/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Ken Tossell, Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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

/** @file

    @brief Camera1394 format7 implementation

 */
#include <iostream>
#include <stdint.h>
#include "yuv.h"
#include <sensor_msgs/image_encodings.h>
#include "format7ladybug.h"
#include "modes.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

/** Start the 1394 device in Format7 mode
 *
 *  @param camera pointer to dc1394camera_t structure.
 *  @param mode currently selected Format7 video mode.
 *  @param[in,out] newconfig new configuration parameters.
 *  @return true, if successful.
 *  @post active_ true, if successful
 */
bool Format7::start(dc1394camera_t *camera, dc1394video_mode_t mode, Config &newconfig)
{
  active_ = false;

  // copy Format7 parameters for updateCameraInfo()
  roi_.x_offset = newconfig.x_offset;
  roi_.y_offset = newconfig.y_offset;
  roi_.width = newconfig.roi_width;
  roi_.height = newconfig.roi_height;

  uint32_t packet_size = newconfig.format7_packet_size;

/*******************************************************************************
 * Commented by JOAN PAU to allow debayering on format7 modes
  // Built-in libdc1394 Bayer decoding (now deprecated) is not
  // supported at all in Format7 modes.
  if (newconfig.bayer_method != "")
    {
      ROS_WARN_STREAM("Bayer method [" << newconfig.bayer_method
                      << "] not supported for Format7 modes."
                      << "  Using image_proc instead.");
      newconfig.bayer_method = "";
    }
 ******************************************************************************/
  //First set the color-------------------------------------------------------
  
  // Try to set requested color coding. Use current camera value if
  // requested coding is not supported by the camera.
  coding_ = Modes::getColorCoding(camera, mode,newconfig.format7_color_coding);

  if (DC1394_SUCCESS != dc1394_format7_set_color_coding(camera, mode,coding_))
    {
      ROS_ERROR("Could not set color coding");
      return false;
    }

   //Then set the package size, and then the rest.----------------------------
   
     uint32_t rec_packet_size;

  if (DC1394_SUCCESS
      != dc1394_format7_get_recommended_packet_size(camera, mode,
                                                    &rec_packet_size))
    {
      ROS_ERROR("Could not get default packet size");
      return false;
    }

 ROS_INFO_STREAM("Requested packet size: " << packet_size);
 ROS_INFO_STREAM("Default packet size: " << rec_packet_size);

  if (0 == packet_size)
    packet_size = rec_packet_size;

  uint32_t unit_bytes, max_bytes;

  if (DC1394_SUCCESS
      != dc1394_format7_get_packet_parameters(camera, mode,
                                              &unit_bytes, &max_bytes))
    {
      ROS_ERROR("Could not determine maximum and increment for packet size");
      return false;
    }

 ROS_INFO_STREAM("Min packet size: " << unit_bytes);
 ROS_INFO_STREAM("Max packet size: " << max_bytes);
  if (packet_size % unit_bytes
      || (max_bytes > 0 && packet_size > max_bytes))
    {
      ROS_ERROR("Invalid packet size: %d. Must be a "
                "multiple of %d, at most %d [%d]",
                packet_size, unit_bytes, max_bytes, rec_packet_size);
      return false;
    }

  if (DC1394_SUCCESS != dc1394_format7_set_packet_size(camera, mode,
                                                       packet_size))
    {
      ROS_ERROR("Could not set packet size");
      return false;
    }
    
    
    //Set image position, size and pattern -----------------------------------------------


  // scan all Format7 modes to determine the full-sensor image size,
  // from which we will calculate the binning values
  uint32_t sensor_width = 0, sensor_height = 0;

  for (int scan_mode = DC1394_VIDEO_MODE_FORMAT7_MIN;
       scan_mode <= DC1394_VIDEO_MODE_FORMAT7_MAX;
       ++scan_mode)
    {
      uint32_t scan_width, scan_height;

      // TODO: only scan modes supported by this device
      if (dc1394_format7_get_max_image_size(camera,
                                            (dc1394video_mode_t) scan_mode,
                                            &scan_width, &scan_height)
          != DC1394_SUCCESS)
        continue;

      if (scan_width > sensor_width)
        sensor_width = scan_width;

      if (scan_height > sensor_height)
        sensor_height = scan_height;
    }

  if (DC1394_SUCCESS != dc1394_format7_get_max_image_size(camera, mode,
                                                      &maxWidth_,
                                                      &maxHeight_))
    {
      ROS_ERROR("Could not get max image size");
      return false;
    }

  if ((roi_.x_offset | roi_.y_offset | roi_.width | roi_.height) == 0)
    {
      roi_.width = maxWidth_;
      roi_.height = maxHeight_;
    }

  uint32_t unit_w, unit_h, unit_x, unit_y;

  if (DC1394_SUCCESS != dc1394_format7_get_unit_size(camera, mode,
                                                     &unit_w, &unit_h))
    {
      ROS_ERROR("Could not get ROI size units");
      return false;
    }

  if (DC1394_SUCCESS != dc1394_format7_get_unit_position(camera, mode,
                                                         &unit_x, &unit_y))
    {
      ROS_ERROR("Could not get ROI position units");
      return false;
    }

  // some devices return zeros for the position units
  if (unit_x == 0) unit_x = 1;
  if (unit_y == 0) unit_y = 1;

  ROS_INFO_STREAM("Format7 unit size: ("
                  << unit_w << "x" << unit_h
                  << "), position: ("
                  << unit_x << "x" << unit_y
                  << ")");
  ROS_INFO_STREAM("Format7 region size: ("
                  << roi_.width << "x" << roi_.height
                  << "), offset: ("
                  << roi_.x_offset << ", " << roi_.y_offset
                  << ")");

  /* Reset ROI position to (0,0). If it was previously (x,y) and
   * the requested ROI size (w,h) results in (x,y) + (w,h) >
   * (max_w,max_h), we'll be unable to set up some valid ROIs
   */
  dc1394_format7_set_image_position(camera, mode, 0, 0);

  if ((roi_.width % unit_w) || (roi_.height % unit_h))
    {
      /// @todo Add some sensible recovery for bad Format7 size.
      ROS_ERROR("Requested image size invalid; (w,h) must be"
                " a multiple of (%d, %d)", unit_w, unit_h);
      return false;
    }

  if (DC1394_SUCCESS != dc1394_format7_set_image_size(camera, mode, roi_.width, roi_.height))
    {
      ROS_ERROR("Could not set size of ROI");
      return false;
    }

  if ((roi_.x_offset % unit_x) || (roi_.y_offset % unit_y))
    {
      ROS_ERROR("Requested image position invalid; (x,y) must"
                " be a multiple of (%d, %d)", unit_x, unit_y);
      return false;
    }

  if (DC1394_SUCCESS != dc1394_format7_set_image_position(camera, mode, roi_.x_offset,roi_.y_offset))
    {
      ROS_ERROR("Could not set position of ROI");
      return false;
    }

  if (coding_ == DC1394_COLOR_CODING_RAW8
      || coding_ == DC1394_COLOR_CODING_RAW16)
    {
      if (DC1394_SUCCESS != dc1394_format7_get_color_filter(camera, mode,
                                                            &BayerPattern_))
        {
          ROS_ERROR("Could not determine color pattern");
          return false;
        }
    }
    
   dc1394operation_mode_t	operation_mode;
  
   if (DC1394_SUCCESS !=  dc1394_video_get_operation_mode(camera,&operation_mode))
        {
          ROS_ERROR("Could not determine operation mode");
          return false;
        }
  if(operation_mode==DC1394_OPERATION_MODE_LEGACY){
      ROS_INFO_STREAM("Operation mode legacy ");
  }
  else if (operation_mode==DC1394_OPERATION_MODE_1394B){
      ROS_INFO_STREAM("Operation mode B");
  }
//      dc1394error_t err;
//err=dc1394_format7_set_roi(camera, DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_RAW8, 9792, 0,0, 1616, 7392);
//DC1394_ERR_RTN(err,"Could not set ROI");

  active_ = true;                       // Format7 mode is active
  mode_ = mode;
  
    uint32_t current_bpp;

  if (DC1394_SUCCESS != dc1394_format7_get_packet_size(camera, mode_, &current_bpp))
  {
   printf("Could not get packet size\n");
  }
  
      printf("Current bpp: %i \n",current_bpp);
      
  return true;
}

/** stop Format7 processing */
void Format7::stop(void)
{
  active_ = false;
}

extern std::string bayer_string(dc1394color_filter_t pattern, unsigned int bits);

//===================SEND RAW====================
/** Unpack Format7 data for an Image frame */
void Format7::unpackData(sensor_msgs::ImagePtr image[6], uint8_t *capture_buffer, std::string publish_monocolour)
{
  int image_size;
  int adr, cam;
  switch (mode_)
    {
   case DC1394_VIDEO_MODE_FORMAT7_0:
      for (cam=0;cam<6;cam++) {
         int width=1616;
         int height=1232;
         adr=0x000+cam*width*height*1;
		 image[cam]->width = width;
		 image[cam]->height= height;
		 image[cam]->step = image[cam]->width;
	     image_size = image[cam]->height * image[cam]->step;
		if(!publish_monocolour.compare(std::string("BW"))==0){
			std::vector<unsigned char> buffer(image_size);
			cv::Mat Reconstructed=cv::Mat::zeros(1232, 1616, CV_8U);
			memcpy(Reconstructed.data, (unsigned char *) capture_buffer+adr, image_size);
			// = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);

			if(Reconstructed.data==NULL){
				printf("Error imdecode\n");
			}

			//Send through ROS using cv_bridge
			cv_bridge::CvImage cv_image;
			cv_image.encoding=sensor_msgs::image_encodings::BGR8;
			cv::Mat ReconstructedColor;
			cv::cvtColor(Reconstructed, ReconstructedColor, CV_BayerBG2BGR);
			cv_image.image=ReconstructedColor;
			image[cam]=cv_image.toImageMsg();
		}
		else{
		     image[cam]->encoding = bayer_string(BayerPattern_, 8);
		     image[cam]->data.resize(image_size);
		     memcpy(&(image[cam]->data[0]), capture_buffer+adr, image_size);
		}
      }
      break;
	//Debayer
/*
	case DC1394_VIDEO_MODE_FORMAT7_7:
		int k;
        unsigned int adr, jpgadr, jpgsize;
         for (cam=0;cam<6;cam++) {
			cv::Mat imgMat[4]=cv::Mat::zeros(616, 808, CV_8U); //height, width
            for (k=0;k<4;k++) {
                adr=0x340+cam*32+k*8;
                jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
                    (((unsigned int)*(capture_buffer+adr+1))<<16)+
                    (((unsigned int)*(capture_buffer+adr+2))<<8)+
                    (((unsigned int)*(capture_buffer+adr+3)));
                adr+=4;
                jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
                    (((unsigned int)*(capture_buffer+adr+1))<<16)+
                    (((unsigned int)*(capture_buffer+adr+2))<<8)+
                    (((unsigned int)*(capture_buffer+adr+3)));

				std::vector<unsigned char> buffer(jpgsize);
				memcpy(&(buffer[0]), (unsigned char *) capture_buffer+jpgadr, jpgsize);

				imgMat[k] = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);
				
				if(! imgMat[k].data )                              // Check for invalid input
				{
					std::cout <<  "Could not decode the image" << std::endl ;
				}
				
            }
            //Reconstruct the original image from the four channels! RGGB
            cv::Mat Reconstructed=cv::Mat::zeros(1232, 1616, CV_8U);
			int x,y;
			for(y = 0; y < 1232; y++){
				uint8_t* row = Reconstructed.ptr<uint8_t>(y);
				if(y % 2 == 0){
					uint8_t* i0 = imgMat[0].ptr<uint8_t>(y / 2);
					uint8_t* i1 = imgMat[1].ptr<uint8_t>(y / 2);

					for(x = 0; x < 1616; ){
					    //R
					    row[x] = i0[x / 2];
					    x++;

					    //G1
					    row[x] = i1[x / 2];
					    x++;
					}
				}
				else {
					uint8_t* i2 = imgMat[2].ptr<uint8_t>(y / 2);
					uint8_t* i3 = imgMat[3].ptr<uint8_t>(y / 2);

					for(x = 0; x < 1616; ){
					    //G2
					    row[x] = i2[x / 2];
					    x++;

					    //B
					    row[x] = i3[x / 2];
					    x++;
					}
				}
			}

			//Debayer
			//cv::Mat ReconstructedColor;
			//cv::cvtColor(Reconstructed, ReconstructedColor, CV_BayerBG2BGR);

			//Send through ROS using cv_bridge
			cv_bridge::CvImage cv_image;
			cv_image.encoding=sensor_msgs::image_encodings::BAYER_RGGB8;
			//cv_image.encoding=sensor_msgs::image_encodings::BGR8;
			cv_image.image=Reconstructed;
			//cv_image.image=ReconstructedColor;

			image[cam]=cv_image.toImageMsg();
        }
      break;
*/
	case DC1394_VIDEO_MODE_FORMAT7_7:
		// Monochrome test
		if(publish_monocolour.compare(std::string("G"))==0){
			int k;
		    unsigned int adr, jpgadr, jpgsize;
		     for (cam=0;cam<6;cam++) {
				cv::Mat imgMat[2]=cv::Mat::zeros(616, 808, CV_8U); //height, width
				//Only interested in greens
		        for (k=1;k<3;k++) {
		            adr=0x340+cam*32+k*8;
		            jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
		                (((unsigned int)*(capture_buffer+adr+1))<<16)+
		                (((unsigned int)*(capture_buffer+adr+2))<<8)+
		                (((unsigned int)*(capture_buffer+adr+3)));
		            adr+=4;
		            jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
		                (((unsigned int)*(capture_buffer+adr+1))<<16)+
		                (((unsigned int)*(capture_buffer+adr+2))<<8)+
		                (((unsigned int)*(capture_buffer+adr+3)));

					std::vector<unsigned char> buffer(jpgsize);
					memcpy(&(buffer[0]), (unsigned char *) capture_buffer+jpgadr, jpgsize);

					imgMat[k-1] = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);
				
					if(! imgMat[k-1].data )                              // Check for invalid input
					{
						std::cout <<  "Could not decode the image" << std::endl ;
					}
				
		        }

		        //Reconstruct full image from the two channels! G1G2
		        cv::Mat Reconstructed=cv::Mat::zeros(1232, 1616, CV_8U);
				int x,y;
				for(y = 0; y < 1232; y++){
					uint8_t* row = Reconstructed.ptr<uint8_t>(y);
					if(y % 2 == 0){
						//G1 row.
						uint8_t* previous = imgMat[1].ptr<uint8_t>(y / 2);
						uint8_t* current = imgMat[0].ptr<uint8_t>(y / 2);
						uint8_t* following = imgMat[1].ptr<uint8_t>(y / 2);
						for(x = 0; x < 1616; x++){
							//Is G1 pixel??
							if(x % 2 ==1){
								//yes
								row[x]=current[x/2];
							}
							else{
								//no. Need interpolation
								int n=0; //Count how many numbers taken into account in the interpolation
								int sum=0; //Sum total of interpolation.
								if(x-1>0){
									sum+=current[x/2];
									n++;
								}
								if(x+1<1616){
									sum+=current[x/2+1];
									n++;
								}
								if(y-1>0){
									sum+=previous[x/2];
									n++;
								}
								if(y+1<1232){
									sum+=following[(x)/2];
									n++;
								}
								row[x]=sum/n;
							}
						}
					}
					else {
						uint8_t* previous = imgMat[0].ptr<uint8_t>(y / 2);
						uint8_t* current = imgMat[1].ptr<uint8_t>(y / 2);
						uint8_t* following = imgMat[0].ptr<uint8_t>(y / 2);

						for(x = 0; x < 1616; x++){
							//Is G2 pixel??
							if(x % 2 ==0){
								//yes
								row[x]=current[x/2];
							}
							else{
								//no. Need interpolation
								int n=0; //Count how many numbers taken into account in the interpolation
								int sum=0; //Sum total of interpolation.
								if(x-1>0){
									sum+=current[x/2];
									n++;
								}
								if(x+1<1616){
									sum+=current[x/2+1];
									n++;
								}
								if(y-1>0){
									sum+=previous[x/2];
									n++;
								}
								if(y+1<1232){
									sum+=following[x/2];
									n++;
								}
								row[x]=sum/n;
							}
						}
					}
				}

				//Send through ROS using cv_bridge
				cv_bridge::CvImage cv_image;
				cv_image.encoding=sensor_msgs::image_encodings::MONO8;
				cv_image.image=Reconstructed;
				image[cam]=cv_image.toImageMsg();
		    }
	  }
		//Reconstruct colour image
		if(publish_monocolour.compare(std::string("C"))==0){
			int k;
			unsigned int adr, jpgadr, jpgsize;
			for (cam=0;cam<6;cam++) {
				cv::Mat imgMat[4]=cv::Mat::zeros(616, 808, CV_8U); //height, width
				for (k=0;k<4;k++) {
					adr=0x340+cam*32+k*8;
					jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
					(((unsigned int)*(capture_buffer+adr+1))<<16)+
					(((unsigned int)*(capture_buffer+adr+2))<<8)+
					(((unsigned int)*(capture_buffer+adr+3)));
					adr+=4;
					jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
					(((unsigned int)*(capture_buffer+adr+1))<<16)+
					(((unsigned int)*(capture_buffer+adr+2))<<8)+
					(((unsigned int)*(capture_buffer+adr+3)));

					std::vector<unsigned char> buffer(jpgsize);
					memcpy(&(buffer[0]), (unsigned char *) capture_buffer+jpgadr, jpgsize);

					imgMat[3-k] = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);

					if(! imgMat[k].data ){	    // Check for invalid input
						std::cout <<  "Could not decode the image" << std::endl ;
					}
				}
				//Reconstruct the original image from the four channels! RGGB
				cv::Mat Reconstructed=cv::Mat::zeros(1232,1616, CV_8U);
				int x,y;
				for(y = 0; y < 1232; y++){
					uint8_t* row = Reconstructed.ptr<uint8_t>(y);
					if(y % 2 == 0){
						uint8_t* i0 = imgMat[0].ptr<uint8_t>(y / 2);
						uint8_t* i1 = imgMat[1].ptr<uint8_t>(y / 2);
						for(x = 0; x < 1616; ){
							//R
							row[x] = i0[x / 2];
							x++;
							//G1
							row[x] = i1[x / 2];
							x++;
						}
					}
					else {
						uint8_t* i2 = imgMat[2].ptr<uint8_t>(y / 2);
						uint8_t* i3 = imgMat[3].ptr<uint8_t>(y / 2);
						for(x = 0; x < 1616; ){
							//G2
							row[x] = i2[x / 2];
							x++;
							//B
							row[x] = i3[x / 2];
							x++;
						}
					}
				}
				//Debayer
				cv::Mat ReconstructedColor;
				cv::cvtColor(Reconstructed, ReconstructedColor, CV_BayerBG2BGR);
				//Now we have the image in colour!

				//Send through ROS using cv_bridge
				cv_bridge::CvImage cv_image;
				cv_image.encoding=sensor_msgs::image_encodings::RGB8;
				cv_image.image=ReconstructedColor;

				image[cam]=cv_image.toImageMsg();
			}
     }
	  //Send 4 different channels test.
	  else if(publish_monocolour.compare(std::string(""))==0){
		int k;
        unsigned int adr, jpgadr, jpgsize;
         for (cam=0;cam<6;cam++) {
			cv::Mat imgMat[4]=cv::Mat::zeros(616, 808, CV_8U); //height, width
			//Only interested in greens

            //Send an image with the 4 channels. R G1 G2 B
            cv::Mat AllChannels=cv::Mat::zeros(2464,808, CV_8U);

            for (k=0;k<4;k++) {
                adr=0x340+cam*32+k*8;
                jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
                    (((unsigned int)*(capture_buffer+adr+1))<<16)+
                    (((unsigned int)*(capture_buffer+adr+2))<<8)+
                    (((unsigned int)*(capture_buffer+adr+3)));
                adr+=4;
                jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
                    (((unsigned int)*(capture_buffer+adr+1))<<16)+
                    (((unsigned int)*(capture_buffer+adr+2))<<8)+
                    (((unsigned int)*(capture_buffer+adr+3)));

					std::vector<unsigned char> buffer(jpgsize);
					memcpy(&(buffer[0]), (unsigned char *) capture_buffer+jpgadr, jpgsize);

					imgMat[k] = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);
				
					if(! imgMat[k].data )                              // Check for invalid input
					{
						std::cout <<  "Could not decode the image" << std::endl ;
					}
					imgMat[k].copyTo(AllChannels(cv::Rect(0,616*k, imgMat[k].cols, imgMat[k].rows)));
				}	

			//Send through ROS using cv_bridge
			cv_bridge::CvImage cv_image;
			cv_image.encoding=sensor_msgs::image_encodings::MONO8;
			cv_image.image=AllChannels;

			image[cam]=cv_image.toImageMsg();
        }
	  }
	  else{
		ROS_ERROR("Wrong configuration or bug in the driver code.");
	  }
      break;


    default:
      ROS_ERROR_STREAM("Driver bug: unknown Format7 mode:"<< mode_);
      ROS_BREAK();
    }
}
//===================SEND COMPRESSED====================
/** Unpack Format7 compressed data for an Image frame and save it as compressed */
void Format7::unpackCompressedData(sensor_msgs::CompressedImagePtr image[6], uint8_t *capture_buffer, std::string publish_monocolour)
{
  switch (mode_)
    {
    case DC1394_VIDEO_MODE_FORMAT7_6:
        int cam;
        unsigned int adr, jpgadr, jpgsize;
        for (cam=0;cam<6;cam++) {	
            adr=0x340+cam*32;
            jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
                 (((unsigned int)*(capture_buffer+adr+1))<<16)+
                 (((unsigned int)*(capture_buffer+adr+2))<<8)+
                 (((unsigned int)*(capture_buffer+adr+3)));
            adr+=4;
            jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
                 (((unsigned int)*(capture_buffer+adr+1))<<16)+
                 (((unsigned int)*(capture_buffer+adr+2))<<8)+
                 (((unsigned int)*(capture_buffer+adr+3)));
			image[cam]->data.resize(jpgsize);
				memcpy(&(image[cam]->data[0]), capture_buffer+jpgadr, jpgsize);
				
				std::vector<unsigned short int> buffer(jpgsize);
				memcpy(&(buffer[0]), capture_buffer+jpgadr, jpgsize);
			    cv::Mat imgbuf = cv::Mat(1616, 1000, CV_8U, &(buffer[0]));
				cv::Mat imgMat = cv::imdecode(imgbuf, CV_LOAD_IMAGE_GRAYSCALE);
			    imgbuf.~Mat();
				if(imgMat.empty()){
					printf("Image is empty\n");
				}
				else{
					cv::Size size=imgMat.size();
					printf("%d,%d\n",size.width,size.height);
					int type=imgMat.type();
					printf("Type:%i\n",type);
					std::cout << int(imgMat.at<unsigned char>(0,0)) << std::endl;
				}

      }
      break;
	case DC1394_VIDEO_MODE_FORMAT7_7:
		//Reconstruct colour image
		if(publish_monocolour.compare(std::string("C"))==0){
			int k;
			unsigned int adr, jpgadr, jpgsize;
			for (cam=0;cam<6;cam++) {
				cv::Mat imgMat[4]=cv::Mat::zeros(616, 808, CV_8U); //height, width
				for (k=0;k<4;k++) {
					adr=0x340+cam*32+k*8;
					jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
					(((unsigned int)*(capture_buffer+adr+1))<<16)+
					(((unsigned int)*(capture_buffer+adr+2))<<8)+
					(((unsigned int)*(capture_buffer+adr+3)));
					adr+=4;
					jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
					(((unsigned int)*(capture_buffer+adr+1))<<16)+
					(((unsigned int)*(capture_buffer+adr+2))<<8)+
					(((unsigned int)*(capture_buffer+adr+3)));

					std::vector<unsigned char> buffer(jpgsize);
					memcpy(&(buffer[0]), (unsigned char *) capture_buffer+jpgadr, jpgsize);

					imgMat[k] = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);

					if(! imgMat[k].data ){	    // Check for invalid input
						std::cout <<  "Could not decode the image" << std::endl ;
					}
				}
				//Reconstruct the original image from the four channels! RGGB
				cv::Mat Reconstructed=cv::Mat::zeros(1232,1616, CV_8U);
				int x,y;
				for(y = 0; y < 1232; y++){
					uint8_t* row = Reconstructed.ptr<uint8_t>(y);
					if(y % 2 == 0){
						uint8_t* i0 = imgMat[0].ptr<uint8_t>(y / 2);
						uint8_t* i1 = imgMat[1].ptr<uint8_t>(y / 2);
						for(x = 0; x < 1616; ){
							//R
							row[x] = i0[x / 2];
							x++;
							//G1
							row[x] = i1[x / 2];
							x++;
						}
					}
					else {
						uint8_t* i2 = imgMat[2].ptr<uint8_t>(y / 2);
						uint8_t* i3 = imgMat[3].ptr<uint8_t>(y / 2);
						for(x = 0; x < 1616; ){
							//G2
							row[x] = i2[x / 2];
							x++;
							//B
							row[x] = i3[x / 2];
							x++;
						}
					}
				}
				//Debayer
				cv::Mat ReconstructedColor;
				cv::cvtColor(Reconstructed, ReconstructedColor, CV_BayerBG2BGR);
				//Now we have the image in colour!

				std::vector<unsigned char> buffer; //Buffer for the image
				std::vector<int> qualityType;
				qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
				qualityType.push_back(80); //Jpeg quality

				//Compress
				cv::imencode(".jpg", ReconstructedColor, buffer, qualityType);

				//Send through ROS using cv_bridge
				//cv_bridge::CvImage cv_image;
				//cv_image.encoding=sensor_msgs::image_encodings::RGB8;
				//cv_image.image=buffer;

				image[cam]->format="jpeg";
				image[cam]->data=buffer;
			}
		}
		else if(publish_monocolour.compare(std::string("R"))==0){
			int k;
			unsigned int adr, jpgadr, jpgsize;
			for (cam=0;cam<6;cam++) {
				cv::Mat imgMat[1]=cv::Mat::zeros(616, 808, CV_8U); //height, width
				//RED is CHANNEL 0
				for (k=0;k<1;k++) {
					adr=0x340+cam*32+k*8;
					jpgadr=(((unsigned int)*(capture_buffer+adr))<<24)+
					(((unsigned int)*(capture_buffer+adr+1))<<16)+
					(((unsigned int)*(capture_buffer+adr+2))<<8)+
					(((unsigned int)*(capture_buffer+adr+3)));
					adr+=4;
					jpgsize=(((unsigned int)*(capture_buffer+adr))<<24)+
					(((unsigned int)*(capture_buffer+adr+1))<<16)+
					(((unsigned int)*(capture_buffer+adr+2))<<8)+
					(((unsigned int)*(capture_buffer+adr+3)));

					std::vector<unsigned char> buffer(jpgsize);
					memcpy(&(buffer[0]), (unsigned char *) capture_buffer+jpgadr, jpgsize);
					
					image[cam]->format="jpeg";
					image[cam]->data=buffer;
				}
			}
		}
	break;
    default:
      ROS_ERROR_STREAM("Driver bug: unknown Format7 mode:"<< mode_);
      ROS_BREAK();
    }
 }
  
/** check whether CameraInfo matches current Format7 image size
 *
 *  @pre active_ is true.
 *  @param cinfo CameraInfo message to check
 *  @return true if camera dimensions match calibration
 *
 *  @post fields filled in (if successful):
 *    roi (region of interest)
 */
bool Format7::checkCameraInfo(const sensor_msgs::CameraInfo &cinfo)
{
  // see if the (full) image size matches the calibration
  if (cinfo.width == maxWidth_ && cinfo.height == maxHeight_)
    {
      return true;
    }
  // or if the ROI size matches the calibration
  else if (cinfo.width == roi_.width && cinfo.height == roi_.height)
    {
      return true;
    }
  else
    {
      ROS_WARN_STREAM_THROTTLE(30, "Calibrated image size ("
                               << cinfo.width << "x" << cinfo.height
                               << ") matches neither full Format7 size ("
                               << maxWidth_ << "x" << maxHeight_
                               << ") nor ROI size ("
                               << roi_.width << "x" << roi_.height << ")");
      return false;
    }
}

/** set operational data fields in CameraInfo message
 *
 *  @pre active_ is true.
 *  @param cinfo CameraInfo message to update
 *  @return true if camera dimensions match calibration
 *
 *  @post fields filled in (if successful):
 *    roi (region of interest)
 */
void Format7::setOperationalParameters(sensor_msgs::CameraInfo &cinfo)
{
  // copy the operational data determined during start()
  cinfo.binning_x = binning_x_;
  cinfo.binning_y = binning_y_;
  cinfo.roi = roi_;

  // set do_rectify depending on current calibration parameters
  cinfo.roi.do_rectify = false;

  if (cinfo.K[0] == 0.0)
    return;				// uncalibrated

  bool roiMatchesCalibration = (cinfo.width == roi_.width
				&& cinfo.height == roi_.height);

  if (cinfo.width == maxWidth_ && cinfo.height == maxHeight_)
    {
      // calibration matches full image size
      if (!roiMatchesCalibration)
	{
	  // ROI is not full image: tell image_pipeline to rectify
	  cinfo.roi.do_rectify = true;
	}
    }
  else
    {
      // calibration differs from full image
      if (!roiMatchesCalibration)
	{
	  // calibrated size is neither full image nor current ROI:
	  //   tell image_pipeline to rectify the data.
	  cinfo.roi.do_rectify = true;
	}
    }
}
