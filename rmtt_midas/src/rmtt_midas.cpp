// MIT License

// Copyright (c) 2020 Alexey

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// @article{Ranftl2020,
// 	author    = {Ren\'{e} Ranftl and Katrin Lasinger and David Hafner and Konrad Schindler and Vladlen Koltun},
// 	title     = {Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer},
// 	journal   = {IEEE Transactions on Pattern Analysis and Machine Intelligence (TPAMI)},
// 	year      = {2020},
// }


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <initializer_list>

#include <torch/script.h> // One-stop header.

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

// includes for OpenCV >= 3.x
#ifndef CV_VERSION_EPOCH
#include <opencv2/core/types.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif

// OpenCV includes for OpenCV 2.x
#ifdef CV_VERSION_EPOCH
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/core/version.hpp>
#endif

// PCL includes for depth image to point cloud
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera intrinsics
const double camera_factor = 255;



static const std::string OPENCV_WINDOW = "Image window";

class Midas
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cloud_pub; 

    torch::jit::script::Module module;
    torch::Device device;

    auto ToTensor(cv::Mat img, bool show_output = false, bool unsqueeze = false, int unsqueeze_dim = 0)
    {
        //std::cout << "image shape: " << img.size() << std::endl;
        at::Tensor tensor_image = torch::from_blob(img.data, { img.rows, img.cols, 3 }, at::kByte);

        if (unsqueeze)
        {
            tensor_image.unsqueeze_(unsqueeze_dim);
            //std::cout << "tensors new shape: " << tensor_image.sizes() << std::endl;
        }

        if (show_output)
        {
            std::cout << tensor_image.slice(2, 0, 1) << std::endl;
        }
        //std::cout << "tenor shape: " << tensor_image.sizes() << std::endl;
        return tensor_image;
    }

    auto ToInput(at::Tensor tensor_image)
    {
        // Create a vector of inputs.
        return std::vector<torch::jit::IValue>{tensor_image};
    }

    auto ToCvImage(at::Tensor tensor, int cv_type = CV_8UC3)
    {
        int width = tensor.sizes()[0];
        int height = tensor.sizes()[1];
        try
        {
            cv::Mat output_mat;
            if (cv_type == CV_8UC4 || cv_type == CV_8UC3 || cv_type == CV_8UC2 || cv_type == CV_8UC1) {
                cv::Mat cv_image(cv::Size{ height, width }, cv_type, tensor.data_ptr<uchar>());
                output_mat = cv_image;
            }
            else if (cv_type == CV_32FC4 || cv_type == CV_32FC3 || cv_type == CV_32FC2 || cv_type == CV_32FC1) {
                cv::Mat cv_image(cv::Size{ height, width }, cv_type, tensor.data_ptr<float>());
                output_mat = cv_image;
            }
            else if (cv_type == CV_64FC4 || cv_type == CV_64FC3 || cv_type == CV_64FC2 || cv_type == CV_64FC1) {
                cv::Mat cv_image(cv::Size{ height, width }, cv_type, tensor.data_ptr<double>());
                output_mat = cv_image;
            }

            //show_image(output_mat, "converted image from tensor");
            return output_mat.clone();
        }
        catch (const c10::Error& e)
        {
            std::cout << "an error has occured : " << e.msg() << std::endl;
        }
        return cv::Mat(height, width, CV_8UC3);
    }

    std::string image_topic, depth_topic, model_name, cloud_topic, frame_id, kdevice;
    double camera_cx, camera_cy, camera_fx, camera_fy;
    bool out_orig_size;
    int net_width, net_height;
    torch::NoGradGuard guard;
    at::Tensor mean, std;
    at::Tensor output, tensor;

public:
    Midas()
        : nh_(), it_(nh_), device(torch::Device(torch::kCPU))
    {     
        ros::param::param<std::string>("~image_topic", image_topic, "image_topic");
        ros::param::param<std::string>("~depth_topic", depth_topic, "midas_topic");
        ros::param::param<std::string>("~model_name", model_name, "model-small-traced.pt");
        ros::param::param<std::string>("~cloud_topic", cloud_topic, "cloud_topic");
        ros::param::param<std::string>("~frame_id", frame_id, "frame_id");
        ros::param::param<std::string>("~kdevice", kdevice, "kdevice");
        ros::param::param<bool>("~out_orig_size", out_orig_size, true);
        ros::param::param<int>("~net_width", net_width, 256);
        ros::param::param<int>("~net_height", net_height, 256);
        ros::param::param<double>("~camera_cx", camera_cx, camera_cx);
        ros::param::param<double>("~camera_cy", camera_cy, camera_cy);
        ros::param::param<double>("~camera_fx", camera_fx, camera_fx);
        ros::param::param<double>("~camera_fy", camera_fy, camera_fy);
        std::cout << "midas settings:" << std::endl;
        std::cout << "\t-input_topic = " << image_topic << std::endl;
        std::cout << "\t-output_topic = " << depth_topic << std::endl;
        std::cout << "\t-cloud_topic = " << cloud_topic << std::endl;
        std::cout << "\t-model_name = " << model_name << std::endl;
        std::cout << "\t-out_orig_size = " << out_orig_size << std::endl;
        std::cout << "\t-net_width = " << net_width << std::endl;
        std::cout << "\t-net_height = " << net_height << std::endl;
        std::cout << "\t-frame_id = " << frame_id << std::endl;
        std::cout << "\t-camera_cx = " << camera_cx << std::endl;
        std::cout << "\t-camera_cy = " << camera_cy << std::endl;
        std::cout << "\t-camera_fx = " << camera_fx << std::endl;
        std::cout << "\t-camera_fy = " << camera_fy << std::endl;

        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe(image_topic, 1, &Midas::imageCb, this);
        image_pub_ = it_.advertise(depth_topic, 1);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1000);

        std::cout << "Try to load torchscript model...\n";
        
        try {
            // Deserialize the ScriptModule from a file using torch::jit::load().
            module = torch::jit::load(model_name);
        }
        catch (const c10::Error& e) {
            std::cerr << "error loading the model!\n";
            exit(0);
        }

        std::cout << "down ";

        try {
            module.eval();
            torch::jit::getProfilingMode() = false;
            torch::jit::setGraphExecutorOptimize(true);

            mean = torch::tensor({ 0.485, 0.456, 0.406 });
            std = torch::tensor({ 0.229, 0.224, 0.225 });

            if(kdevice == "kCUDA"){
                if (torch::hasCUDA()) {
                    std::cout << "and cuda is available \n" << std::endl;
                    at::globalContext().setBenchmarkCuDNN(true);
                    device = torch::Device(torch::kCUDA);
                    module.to(device);
                    mean = mean.to(device);
                    std = std.to(device);
                }
            }
            else if(kdevice == "kCPU"){
                std::cout << "and switch device to cpu... " << std::endl;
                device = torch::Device(torch::kCPU);
                module.to(device);
                mean = mean.to(device);
                std = std.to(device);
                std::cout << "down and cpu is available \n" << std::endl;

            }
            else{
                ROS_ERROR("wrong device!");
            }
        }
        catch (const c10::Error& e)
        {
            std::cerr << " module initialization: " << e.msg() << std::endl;
        }
    }

    ~Midas()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // sensor_msgs::Image to cv::Mat
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // pre-processing
        auto tensor_cpu = ToTensor(cv_ptr->image);           // OpenCV-image -> Libtorch-tensor

        try {
            tensor = tensor_cpu.to(device); // move to device (CPU or GPU)      

            tensor = tensor.toType(c10::kFloat);
            tensor = tensor.permute({ 2, 0, 1 });   // HWC -> CHW
            tensor = tensor.unsqueeze(0);
            tensor = at::upsample_bilinear2d(tensor, { net_height, net_width }, true);  // resize
            tensor = tensor.squeeze(0);
            tensor = tensor.permute({ 1, 2, 0 });   // CHW -> HWC
                                                                
            tensor = tensor.div(255).sub(mean).div(std);    // normalization
            tensor = tensor.permute({ 2, 0, 1 });   // HWC -> CHW
            tensor.unsqueeze_(0);                   // CHW -> NCHW
        }
        catch (const c10::Error& e)
        {
            std::cerr << " pre-processing exception: " << e.msg() << std::endl;
            return;
        }
        
        auto input_to_net = ToInput(tensor);                    // input to the network

        // inference
        try {
            output = module.forward(input_to_net).toTensor();   // run inference
        }
        catch (const c10::Error& e)
        {
            std::cerr << " module.forward() exception: " << e.msg() << std::endl;
            return;
        }
        
        output = output.detach().to(torch::kF32);

        // move to CPU temporary
        at::Tensor output_tmp = output;
        output_tmp = output_tmp.to(torch::kCPU);

        // normalization
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::min();

        for (int i = 0; i < net_width * net_height; ++i) {
            float val = output_tmp.data_ptr<float>()[i];
            if (min_val > val) min_val = val;
            if (max_val < val) max_val = val;
        }
        float range_val = max_val - min_val;
               
        output = output.sub(min_val).div(range_val).mul(255.0F).clamp(0, 255).to(torch::kF32);   // .to(torch::kU8);

        // resize to the original size if required
        if (out_orig_size) {
            try {
                output = at::upsample_bilinear2d(output.unsqueeze(0), { cv_ptr->image.size().height, cv_ptr->image.size().width }, true);
                output = output.squeeze(0);
            }
            catch (const c10::Error& e)
            {
                std::cout << " upsample_bilinear2d() exception: " << e.msg() << std::endl;
                return;
            }
        }
        output = output.permute({ 1, 2, 0 }).to(torch::kCPU);
        
        
        int cv_type = CV_32FC1; // CV_8UC1;
        auto cv_img = ToCvImage(output, cv_type);
        
        // to pcl
        PointCloud::Ptr cloud(new PointCloud);
        sensor_msgs::PointCloud2 cloud_msg;

        // 遍历图像
        for(int m = 0; m < cv_img.rows; m++){
            for(int n = 0; n < cv_img.cols; n++)
            {
                float d = cv_img.ptr<float>(m)[n];
                if (d == 0)
                    continue;
                
                // 深度存在，则向点云中增加一点，并计算此点的坐标: 设置x为正向
                PointT p;
                p.x =  (350 - d) / camera_factor;
                p.y = -(n - camera_cx) * p.x / camera_fx;
			    p.z = -(m - camera_cy) * p.x / camera_fy;
                
                // 获取此点的颜色信息
                p.r = cv_ptr->image.data[m*cv_ptr->image.step + n*cv_ptr->image.channels()];
                p.g = cv_ptr->image.data[m*cv_ptr->image.step + n*cv_ptr->image.channels() + 1];
                p.b = cv_ptr->image.data[m*cv_ptr->image.step + n*cv_ptr->image.channels() + 2];

                cloud->points.push_back(p);
            }
        }

        cloud->height = 1;
        cloud->width = cloud->points.size();
        // ROS_INFO("point cloud size: %d.", cloud->points.size());
	    cloud->is_dense = false;

        // 转化点云消息并发布
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = msg->header.stamp;

        sensor_msgs::Image img_msg;
        try {
            // cv::Mat -> sensor_msgs::Image
            std_msgs::Header header;        // empty header
            header.seq = 0;                 // user defined counter
            header.stamp = ros::Time::now();// time
            //cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cv_img);
            cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, cv_img);
                        
            img_bridge.toImageMsg(img_msg); // cv_bridge -> sensor_msgs::Image
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Output modified video and cloud stream 
        image_pub_.publish(img_msg);
        cloud_pub.publish(cloud_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "midas", ros::init_options::AnonymousName);
    std::cout << "*********************************setting up for midas*********************************" << std::endl;
    Midas ic;
    ros::spin();
    return 0;
}