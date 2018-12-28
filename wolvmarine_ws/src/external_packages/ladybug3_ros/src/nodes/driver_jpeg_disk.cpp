#include "driver_jpeg_disk.hpp"

//Camera parameters.

#define VIDEO_MODE DC1394_VIDEO_MODE_FORMAT7_7
#define OPERATION_MODE DC1394_OPERATION_MODE_1394B
#define ISO_SPEED DC1394_ISO_SPEED_800
#define BPP 8160
#define DMA_RING_BUFFER 10 //Buffer of images
#define WIDTH 808
#define HEIGHT 5000 //MAX 14784   No more fps smaller than 5000. Increase, decrease related to fps and jpeg quality.
#define MAX_BUFFER 150

DriverJpeg::DriverJpeg(int argc, char **argv)
: preview_(false), saveImagesThread_(boost::bind(&DriverJpeg::save_images, this))
{
    //Node handler
    directory_=argv[1]; //Relative path to the directory
    previewTime_ =atof(argv[2]);
    image_laterals_pub_ = nh_.advertise<sensor_msgs::Image>("preview_laterals", 1);
    image_bottom_pub_ = nh_.advertise<sensor_msgs::Image>("preview_bottom", 1);
    timer_ = nh_.createTimer(ros::Duration(previewTime_), &DriverJpeg::set_preview_flag, this);
    timer_.start();
}
DriverJpeg::~DriverJpeg(){
}
int DriverJpeg::run(){

    dc1394error_t err;
    dc1394camera_t *camera;
    dc1394video_frame_t *frame;
    dc1394_t * d;
    dc1394camera_list_t * list;

    d = dc1394_new ();
    if (!d)
        return 1;
    err=dc1394_camera_enumerate (d, &list);
    DC1394_ERR_RTN(err,"Failed to enumerate cameras");

    if (list->num == 0) {
        dc1394_log_error("No cameras found");
        return 1;
    }

    camera = dc1394_camera_new (d, list->ids[0].guid);
    if (!camera) {
        dc1394_log_error("Failed to initialize camera with guid %llx",list->ids[0].guid);
        return 1;
    }
    dc1394_camera_free_list (list);
    printf("Using camera \"%s %s\"\n",camera->vendor,camera->model);
    // setup video mode, etc...

    err=dc1394_video_set_operation_mode(camera, OPERATION_MODE);
    DC1394_ERR_RTN(err,"Could not set B mode");
    err=dc1394_video_set_iso_speed(camera, ISO_SPEED);
    DC1394_ERR_RTN(err,"Could not set 800Mbps speed");
    err=dc1394_video_set_mode(camera, VIDEO_MODE);
    DC1394_ERR_RTN(err,"Could not set video mode");
    
    err=dc1394_format7_set_roi(camera, VIDEO_MODE, DC1394_COLOR_CODING_MONO8, BPP, 0,0, WIDTH, HEIGHT);
    DC1394_ERR_RTN(err,"Could not set ROI");

    // setup capture
    err=dc1394_capture_setup(camera, DMA_RING_BUFFER, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_ERR_RTN(err,"Could not setup capture");
    err=dc1394_video_set_transmission(camera, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not start transmission");

    double total_time=0;
    int max_busy=0;
    //DEBUG
    bool first_image=true;
    int prev_ladybug_id, id_ladybug;

    while (ros::ok()) {
        dc1394video_frame_t *frame;
        // capture frame
        dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
        DC1394_ERR_RTN(err,"Could not dequeue a frame")
 
        //Save first timestamp as laduybug does not give absoulte time.
        if(first_image){
        	time_start_= (double) frame->timestamp/1000000;
            first_image=false;
        }
        
        mtx_.lock();
        //push frame to buffer
        if(buffer_.size()>max_busy){
            max_busy=buffer_.size();
        }
        if(buffer_.size()<MAX_BUFFER){
            //We need to make a copy of the iamge because it's overwritten in ever cycle of the dma buffer.
            int image_bytes=frame->image_bytes; //It must be the same than specified at the top (808*5000)
            int padding_bytes=frame->padding_bytes;
            unsigned char* image_copy = (unsigned char *) malloc (image_bytes+padding_bytes);
            memcpy (image_copy, frame->image, (image_bytes+padding_bytes));
            buffer_.push_back(image_copy);
            
        }
        else{
            printf("=======WARNING: Buffer is full! Dropping frames =========\n");
            std::cout << "Processing frame: " << nframes_ << std::endl;
            buffer_full_++;
            usleep(10000);
        }
        total_time=total_diff_;
        mtx_.unlock();

        // release frame
        err=dc1394_capture_enqueue(camera, frame);
        DC1394_ERR_RTN(err,"Could not enqueue a frame");
        ros::spinOnce();
    }
    // stop capture
    err=dc1394_video_set_transmission(camera, DC1394_OFF);
    DC1394_ERR_RTN(err,"Could not stop transmission");
    err=dc1394_capture_stop(camera);
    DC1394_ERR_RTN(err,"Could not stop capture");
    dc1394_camera_free (camera);
    dc1394_free (d);
    //First frame doesn't count for computations of fps.
	std::cout << "Captured "<<nframes_ << " frames in "<< total_diff_<< "sec. Average: "<< (nframes_-1)/total_diff_<<" fps Max diff Ladybug: " << max_diff_ladybug_ << "sec." << std::endl;
    std::cout << "Max images queued in buffer: " << max_busy << std::endl;
    std::cout << "Lost frames: " << total_lost_frames_ << ". Due to buffer full: " << buffer_full_ << std::endl;
    return 0;
}

void DriverJpeg::set_preview_flag(const ros::TimerEvent& event){
    //std::cout << "Timeout" << std::endl;
    preview_=true;
}

void DriverJpeg::save_images(){


    double start_ladybug;
    double prev_ladybug;
    total_diff_=0; // Difference of time
    max_diff_ladybug_=0; // Difference of time
    int prev_ladybug_id;
    int lost_frames=0;
    total_lost_frames_=0;
    FILE *fd;

    //dc1394video_frame_t *frame;
    //unsigned char * image;    
    char filename[256];
    nframes_=0; //Captured frames.

	//Variables to retrieve directory path.
	long size;
	char *buf;
	char *basename;

	size = pathconf(".", _PC_PATH_MAX);
	if ((buf = (char *)malloc((size_t)size)) != NULL)
		basename = getcwd(buf, (size_t)size);
	
	//char directory;
	char directoryPath[200];
	//directory="test";

    sprintf(directoryPath,"%s/%s",basename,directory_);

    int status;
    status = mkdir(directoryPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(status==-1){
		printf("Cannot create directory or directory exists. Please choose another one or delete it.\n");
		exit(1);
	}

	// Allocate space for 6 cameras red channel for preview
    // Allocate space for 6 cameras red channel
	cv::Mat imgMat[6]=cv::Mat::zeros(616, 808, CV_8U); //height, width
	cv::Mat imgDisplay[6]=cv::Mat::zeros(154, 202, CV_8U); //height, width

    bool first_image=true;
    int cam, k=0;
    unsigned int jpgadr, jpgsize, adr;

    while(true){
        unsigned char * image=NULL;
        mtx_.lock();
        int frames=buffer_.size();
        if(frames>0){
            image=buffer_.front();
            buffer_.erase (buffer_.begin(),buffer_.begin()+1);
        }
        mtx_.unlock();
        if(image!=NULL){           
            //Get Ladybug timestamp
            adr=0x0018;
            unsigned int seconds=(((unsigned int)*(image+adr))<<24)+
                        (((unsigned int)*(image+adr+1))<<16)+
                        (((unsigned int)*(image+adr+2))<<8)+
                       (((unsigned int)*(image+adr+3)));
            
            adr=0x001C;
            unsigned int microseconds=(((unsigned int)*(image+adr))<<24)+
                        (((unsigned int)*(image+adr+1))<<16)+
                        (((unsigned int)*(image+adr+2))<<8)+
                       (((unsigned int)*(image+adr+3)));
            double timestamp_ladybug=seconds+microseconds/1000000.0;
               
            adr=0x0020; //Frame id
            int id_ladybug=(int) ((((unsigned int)*(image+adr))<<24)+
                        (((unsigned int)*(image+adr+1))<<16)+
                        (((unsigned int)*(image+adr+2))<<8)+
                       (((unsigned int)*(image+adr+3))));

            // Save the image in separated files
            for (cam=0;cam<6;cam++) {
                for (k=0;k<4;k++) {
                    adr=0x340+cam*32+k*8;
                    jpgadr=(((unsigned int)*(image+adr))<<24)+
                        (((unsigned int)*(image+adr+1))<<16)+
                        (((unsigned int)*(image+adr+2))<<8)+
                        (((unsigned int)*(image+adr+3)));
                    adr+=4;
                    jpgsize=(((unsigned int)*(image+adr))<<24)+
                        (((unsigned int)*(image+adr+1))<<16)+
                        (((unsigned int)*(image+adr+2))<<8)+
                        (((unsigned int)*(image+adr+3)));

                    if (jpgsize!=0) {

                        sprintf(filename,"%s/%s%d_%s%05d_%d.jpg",directoryPath,"camera",cam,"frame",nframes_,k);
                        //printf("%s\n",filename);
                        fd=fopen(filename,"w");
                        fwrite((unsigned char *)(jpgadr+image),jpgsize,1,fd);
                        fclose(fd);
                        
                        if(preview_ && k==0){
                            std::vector<unsigned char> buffer(jpgsize);
                            memcpy(&(buffer[0]), (unsigned char *) image+jpgadr, jpgsize);
                            imgMat[cam] = cv::imdecode(buffer, CV_LOAD_IMAGE_GRAYSCALE);
                        }
                    }
                }
            }
            //std::cout <<id_ladybug << std::endl;
            if(first_image){
                start_ladybug=timestamp_ladybug;//We keep a copy of first (non absoulte ladybug timestamp)
                timestamp_ladybug=time_start_;
                first_image=false;
            }
            else{
                timestamp_ladybug+=time_start_-start_ladybug;
                lost_frames=id_ladybug-prev_ladybug_id-1;
                if(lost_frames!=0){
                    total_lost_frames_+=lost_frames;
                    std::cout << "==============WARNING: " << lost_frames << " FRAMES LOST==============" << std::endl;
                }
                // compute time delay
                double diff_ladybug=timestamp_ladybug-prev_ladybug;
                if(max_diff_ladybug_<diff_ladybug){
                    max_diff_ladybug_=diff_ladybug;
                }
                //printf("Time elapsed between frames: %f sec\n",diff);        
                //printf("Time elapsed between frames ladybug: %f sec\n",diff_ladybug);   
            }
            prev_ladybug_id=id_ladybug;
            
            //Save timestamp
            sprintf(filename,"%s/%s%05d.txt",directoryPath,"frame",nframes_);
            fd=fopen(filename,"w");
            //fprintf(fd,"%lf",time_sec);
            fprintf(fd,"%lf",timestamp_ladybug);
            fclose(fd);
            
            total_diff_=timestamp_ladybug-time_start_;
            prev_ladybug=timestamp_ladybug;
            
            std::cout << std::setw(5) << std::setfill('0') << nframes_ << " Buffer: " << std::setw(5) << std::setfill('0') << frames <<" (" << MAX_BUFFER << ") \r" << std::flush;
            nframes_++;
            
            if(preview_){
               
                //Show reduced image.
                //Send an image with the 6 reduced images.
                cv::Mat AllCameras=cv::Mat::zeros(imgDisplay[0].cols,imgDisplay[0].rows*5, CV_8U);
                //Images in 1x5 mosaic
                for (int cam=0;cam<5;cam++) {
                    int r=4-cam;
                    cv::resize(imgMat[cam], imgDisplay[cam], imgDisplay[cam].size(), 0, 0, cv::INTER_LINEAR);
                    cv::Mat transposed=imgDisplay[cam].clone();
                    cv::transpose(transposed,transposed);
                    cv::flip(transposed,transposed,0);
                    transposed.copyTo(AllCameras(cv::Rect(transposed.cols*r,0, transposed.cols, transposed.rows)));
                }
                cv::Mat bottomImage(imgDisplay[0].rows,imgDisplay[0].cols,CV_8U);
                cv::resize(imgMat[5], bottomImage, bottomImage.size(), 0, 0, cv::INTER_LINEAR);
                //Send using cv_bridge
                cv_bridge::CvImage cv_image;
                cv_image.encoding=sensor_msgs::image_encodings::MONO8;
                cv_image.image=AllCameras;
                image_laterals_pub_.publish(cv_image.toImageMsg());
                cv_image.image=bottomImage;
                image_bottom_pub_.publish(cv_image.toImageMsg());
                preview_=false;
            }
            
            free(image);
        }
        else if(!ros::ok()){
            return;
        }
        else{
            usleep(60000);//Sleep 60ms
        }
    }
}

int main(int argc, char **argv)
{
	if (argc!=3) {
        std::cerr << "Usage :"<< argv[0] <<" <Directory> <PreviewTime>"  << std::endl;
        std::cerr << "This will save all the frames (24 images per frame) in the specified directory ( must be new and relative path). Time between preview images" << std::endl;
        return 1;
	}


  // Set up ROS.
  ros::init(argc, argv, "listener");
  
  // Create a new NodeExample object.
  DriverJpeg *object = new DriverJpeg(argc,argv);
  object->run();

  return 0;
} // end main()
